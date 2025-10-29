from nav_msgs.msg import OccupancyGrid

import numpy as np
from typing import Optional, Tuple



class MapConversions:
    def __init__(self, resolution: float, width: int, height: int, x0: float, y0: float):
        """
        Create an object from parameters.

        inputs:
            boundary    edges of the environment in the order (xmin, ymin, xmax, ymax) [m]
            resolution  size of the cells in the occupancy grid [m]
        """
        # cell size
        self.s = resolution
        # number of columns (width) and rows (height)
        self.Nc = width
        self.Nr = height
        # lower-left origin
        self.x0 = x0
        self.y0 = y0
        # precompute max bounds
        self.xmax = x0 + self.Nc * self.s
        self.ymax = y0 + self.Nr * self.s

    @classmethod
    def from_msg(cls, occ_msg):
        # occ_msg: nav_msgs.msg.OccupancyGrid
        res = occ_msg.info.resolution
        width = occ_msg.info.width
        height = occ_msg.info.height
        x0 = occ_msg.info.origin.position.x
        y0 = occ_msg.info.origin.position.y
        return cls(res, width, height, x0, y0)

    def _in_bounds_xy(self, x: float, y: float) -> bool:
        # Allow exact right/top boundary
        return (self.x0 <= x <= self.xmax) and (self.y0 <= y <= self.ymax)

    def _in_bounds_sub(self, r: int, c: int) -> bool:
        return 0 <= r < self.Nr and 0 <= c < self.Nc

    def _in_bounds_ind(self, i: int) -> bool:
        return 0 <= i < self.Nr * self.Nc
    
    def xy2sub(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        if not self._in_bounds_xy(x, y):
            return None
        # Adjust exact right/top boundaries to be in last cell
        if x == self.xmax:
            c = self.Nc - 1
        else:
            c = int((x - self.x0) // self.s)
        if y == self.ymax:
            r = self.Nr - 1
        else:
            r = int((y - self.y0) // self.s)
        return (r, c) if self._in_bounds_sub(r, c) else None

    def sub2xy(self, r: int, c: int) -> Optional[Tuple[float, float]]:
        if not self._in_bounds_sub(r, c):
            return None
        x = self.x0 + self.s * (c + 0.5)
        y = self.y0 + self.s * (r + 0.5)
        return (x, y)
    
    def ind2sub(self, i: int) -> Optional[Tuple[int, int]]:
        if not self._in_bounds_ind(i):
            return None
        r = i // self.Nc
        c = i % self.Nc
        return (r, c)

    def sub2ind(self, r: int, c: int) -> Optional[int]:
        if not self._in_bounds_sub(r, c):
            return None
        return r * self.Nc + c

    def xy2ind(self, x: float, y: float) -> Optional[int]:
        sub = self.xy2sub(x, y)
        if sub is None:
            return None
        return self.sub2ind(*sub)

    def ind2xy(self, i: int) -> Optional[Tuple[float, float]]:
        sub = self.ind2sub(i)
        if sub is None:
            return None
        return self.sub2xy(*sub)


