"""Bounded FP64 Vulkan interface. Shader state includes the entire optimizer.

Only startup constants and small progress snapshots cross the Python boundary.
Builds are content-addressed and process-locked; no dependency installation.
"""
import ctypes as C
import fcntl
import hashlib
from functools import lru_cache
from pathlib import Path
import shutil
import subprocess

import numpy as np
from .. import core

HERE = Path(__file__).resolve().parent
BUILD = HERE.parents[1]/'build'/'vulkan_timing'
SHADERS = ('eval', 'cost', 'choose', 'gradient', 'update', 'dual')
SOURCES = ('device.py', 'optimizer.py', 'engine.cpp', 'common.glsl',
           *(s+'.comp' for s in SHADERS))
PTR = C.POINTER(C.c_double)


class GPUUnavailable(RuntimeError):
    """No usable compiled Vulkan FP64 device; caller may explicitly use CPU."""


class GPUExecutionError(RuntimeError):
    """Device/solver execution failed, NOT permission for a hidden CPU solve."""


def fingerprint():
    return hashlib.sha256(b''.join((HERE/p).read_bytes() for p in SOURCES)).hexdigest()


@lru_cache(maxsize=4)
def build(signature):
    target = BUILD/signature
    try:
        target.mkdir(parents=True, exist_ok=True)
        with (BUILD/'build.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            if (target/'engine.so').is_file() and all((target/(s+'.spv')).is_file() for s in SHADERS):
                return target
            if not shutil.which('glslc') or not shutil.which('g++'):
                raise GPUUnavailable('Vulkan build tools missing: glslc and g++ are required')
            for shader in SHADERS:
                subprocess.run(['glslc', '--target-env=vulkan1.2', '-O', '-I', str(HERE),
                    str(HERE/(shader+'.comp')), '-o', str(target/(shader+'.spv'))],
                    check=True, capture_output=True, text=True, timeout=45)
            temporary = target/'engine.tmp.so'
            subprocess.run(['g++', '-O3', '-std=c++17', '-shared', '-fPIC',
                str(HERE/'engine.cpp'), '-lvulkan', '-o', str(temporary)],
                check=True, capture_output=True, text=True, timeout=90)
            temporary.replace(target/'engine.so')
            return target
    except (OSError, subprocess.SubprocessError) as exc:
        detail = getattr(exc, 'stderr', None) or str(exc)
        raise GPUUnavailable('Cannot build Vulkan timing backend: '+detail[-2000:]) from exc


class Device:
    def __init__(self, B, q, weights, x):
        B = np.asarray(B, dtype=np.float64)
        q = np.asarray(q, dtype=np.float64)
        self.m, self.n = B[0].shape
        n, m = self.n, self.m
        if B.shape != (3,m,n) or q.shape != (3,m,7) or not 4<=n<=192 or not 1<=m<=10000:
            raise ValueError('Unsupported bounded GPU timing dimensions')
        if not all(np.isfinite(a).all() for a in (B,q,weights,x)):
            raise ValueError('Nonfinite GPU timing input')
        self.p = None
        folder = build(fingerprint())
        try:
            self.lib = C.CDLL(str(folder/'engine.so'))
        except OSError as exc:
            raise GPUUnavailable('Vulkan loader unavailable: '+str(exc)) from exc
        lib = self.lib
        lib.vg_create.argtypes = [C.c_uint32,C.c_uint32,PTR,C.c_size_t,C.c_char_p]
        lib.vg_create.restype = C.c_void_p
        lib.vg_destroy.argtypes = [C.c_void_p]
        lib.vg_step.argtypes = [C.c_void_p,PTR]; lib.vg_step.restype = C.c_int
        lib.vg_dual.argtypes = [C.c_void_p,C.c_double]; lib.vg_dual.restype = C.c_int
        lib.vg_error.restype = C.c_char_p
        for name in ('vg_device','vg_identity'):
            getattr(lib,name).argtypes = [C.c_void_p]
            getattr(lib,name).restype = C.c_char_p
        support = np.any(B!=0, axis=0)
        lo = np.zeros(n); hi = np.zeros(n)
        for j in range(n):
            rows = np.flatnonzero(support[:,j])
            if len(rows): lo[j],hi[j] = rows[0],rows[-1]+1
        compact = np.zeros((m,13))
        for r in range(m):
            indices = np.flatnonzero(support[r])
            first = min(int(indices[0]),n-4)
            if indices[-1]>=first+4:
                raise ValueError('Expected locally supported cubic B-splines')
            compact[r,0] = first
            compact[r,1:] = B[:,r,first:first+4].ravel()
        # Layout is shared with common.glsl: metadata, vectors/history, constants,
        # 21 multipliers per grid point, and 24 parallel line-search candidates.
        offset = 64+23*n+24+48
        data = np.r_[np.zeros(offset), B.transpose(0,2,1).ravel(), q.ravel(),
                     lo, hi, compact.ravel(), np.zeros(21*m+24*m*5)]
        data[0] = 1.
        data[13:16] = core.LIMITS
        data[64:64+n] = x
        data[64+5*n:64+6*n] = weights
        self.p = lib.vg_create(n,m,data.ctypes.data_as(PTR),len(data),str(folder).encode())
        if not self.p:
            raise GPUUnavailable(lib.vg_error().decode())
        self.name = lib.vg_device(self.p).decode()
        self.identity = lib.vg_identity(self.p).decode()
        self.snapshot = np.empty(64+3*n,dtype=np.float64)

    def step(self):
        """32 complete GPU optimizer iterations; return x and progress only."""
        if self.lib.vg_step(self.p,self.snapshot.ctypes.data_as(PTR)):
            raise GPUExecutionError(self.lib.vg_error().decode())
        if not np.isfinite(self.snapshot).all():
            raise GPUExecutionError('Nonfinite Vulkan optimizer state')
        return self.snapshot[64:64+self.n].copy(),self.snapshot[:64].copy()

    def dual(self, rho):
        if self.lib.vg_dual(self.p,rho):
            raise GPUExecutionError(self.lib.vg_error().decode())

    def close(self):
        if self.p:
            self.lib.vg_destroy(self.p)
            self.p = None

    def __enter__(self): return self

    def __exit__(self,*exc): self.close()


def probe():
    """A tiny device creation, no trajectory optimization or hardware access."""
    from scipy.interpolate import BSpline
    basis=BSpline(np.r_[np.zeros(4),np.ones(4)],np.eye(4),3)
    grid=np.array([0.,.5,1.])
    with Device(np.array([basis(grid,nu=d) for d in range(3)]),
                np.zeros((3,3,7)),np.ones(4)/4,np.ones(4)) as gpu:
        return dict(name=gpu.name,identity=gpu.identity,precision='float64')
