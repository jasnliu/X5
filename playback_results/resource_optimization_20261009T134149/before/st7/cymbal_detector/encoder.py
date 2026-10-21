from __future__ import annotations

import hashlib
from pathlib import Path

import torch
from torch import Tensor, nn

from .frontend import LogMelFrontend
from .vendor.efficientsed import make_fmn06


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class FrozenAudioEncoder(nn.Module):
    feature_hop_seconds = 0.01
    feature_version = 3

    def __init__(self, weights: str | Path, expected_sha256: str | None = None) -> None:
        super().__init__()
        self.weights_path = Path(weights)
        if expected_sha256:
            actual = sha256_file(self.weights_path)
            if actual != expected_sha256:
                raise RuntimeError(f"Encoder checksum mismatch: expected {expected_sha256}, got {actual}")
        self.frontend = LogMelFrontend()
        self.backbone = make_fmn06()
        raw = torch.load(self.weights_path, map_location="cpu", weights_only=True)
        state = {key.removeprefix("net.model."): value for key, value in raw.items() if key.startswith("net.model.")}
        missing, unexpected = self.backbone.load_state_dict(state, strict=False)
        if missing or unexpected:
            raise RuntimeError(f"Could not load fmn06 encoder cleanly; missing={missing}, unexpected={unexpected}")
        self.eval()
        for parameter in self.parameters():
            parameter.requires_grad_(False)

    @property
    def output_dim(self) -> int:
        return self.backbone.output_dim + 256

    @torch.inference_mode()
    def forward(self, waveform: Tensor) -> Tensor:
        self.eval()
        logmel = self.frontend(waveform)
        semantic = self.backbone(logmel)
        # Keep high-resolution attack cues that the broad semantic receptive
        # field can smooth away. Both local paths use the same log-mel input.
        local = logmel.squeeze(1).transpose(1, 2)
        semantic = torch.nn.functional.interpolate(
            semantic.transpose(1, 2), size=local.shape[1], mode="linear", align_corners=False
        ).transpose(1, 2)
        previous = torch.nn.functional.pad(local[:, :-1, :], (0, 0, 1, 0))
        positive_flux = torch.relu(local - previous)
        return torch.cat((semantic, local, positive_flux), dim=-1)
