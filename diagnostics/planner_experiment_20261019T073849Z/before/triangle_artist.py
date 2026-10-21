"""Vectorized Matplotlib projection of complete, opaque triangle surfaces.

The stock 3D collection sorts/builds each face in Python. Keeping all robot
surfaces must not cost that loop on every animation frame. This is the same
painter projection/order, specialized to triangles, not geometry decimation.
"""
import numpy as np
from matplotlib.collections import PolyCollection
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


class TriangleCollection(Poly3DCollection):
    def do_3d_projection(self):
        # _vec, axes.M and the 2D color caches are the same hooks used by the
        # installed Matplotlib 3D collection. Public set_verts/set_facecolors
        # and visibility controls remain unchanged.
        if not self._vec.shape[1]:
            self._facecolors2d=np.empty((0,4));self._edgecolors2d=np.empty((0,4))
            PolyCollection.set_verts(self,[])
            return np.nan
        projected=self.axes.M@self._vec
        projected=projected[:3]/projected[3]
        xyz=projected.T.reshape(-1,3,3)
        order=np.argsort(-xyz[:,:,2].mean(axis=1),kind='stable')
        colors=self._facecolor3d
        self._facecolors2d=colors[order] if len(colors)>1 else np.repeat(colors,len(order),axis=0)
        self._edgecolors2d=np.empty((0,4))
        PolyCollection.set_verts(self,xyz[order,:,:2],closed=True)
        return projected[2].min()
