# occupancy_grid/occupancy_grid/occupancy_grid_map.py
from typing import List, Tuple
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from .map_conversions import MapConversions

OCC_UNKNOWN = -1
OCC_FREE = 0
OCC_OCC = 100

class OccupancyGridMap(MapConversions):
    def __init__(self, resolution: float, width: int, height: int, x0: float, y0: float):
        super().__init__(resolution, width, height, x0, y0)
        self.data = [OCC_FREE] * (self.Nr * self.Nc)

    @classmethod
    def empty_from_boundary(cls, boundary: Tuple[float, float, float, float], resolution: float):
        xmin, ymin, xmax, ymax = boundary
        width = int(round((xmax - xmin) / resolution))
        height = int(round((ymax - ymin) / resolution))
        return cls(resolution, width, height, xmin, ymin)

    def add_block(self, xmin: float, ymin: float, xmax: float, ymax: float):
        # Compute inclusive subscript bounds
        top_left = self.xy2sub(xmin, ymax)  # ymax maps to last row where needed
        bot_right = self.xy2sub(xmax, ymin)
        if top_left is None or bot_right is None:
            # Clip to map bounds
            xmin = max(xmin, self.x0)
            ymin = max(ymin, self.y0)
            xmax = min(xmax, self.xmax)
            ymax = min(ymax, self.ymax)
            top_left = self.xy2sub(xmin, ymax)
            bot_right = self.xy2sub(xmax, ymin)
            if top_left is None or bot_right is None:
                return  # completely outside
        r_min = min(top_left[0], bot_right[0])
        r_max = max(top_left[0], bot_right[0])
        c_min = min(top_left[1], bot_right[1])
        c_max = max(top_left[1], bot_right[1])

        for r in range(r_min, r_max + 1):
            base = r * self.Nc
            for c in range(c_min, c_max + 1):
                self.data[base + c] = OCC_OCC

    def to_msg(self, frame_id: str) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.frame_id = frame_id

        info = MapMetaData()
        info.resolution = self.s
        info.width = self.Nc
        info.height = self.Nr
        origin = Pose()
        origin.position.x = self.x0
        origin.position.y = self.y0
        origin.orientation.w = 1.0
        info.origin = origin

        msg.info = info
        msg.data = self.data[:]  # copy
        return msg

    def is_occupied_xy(self, x: float, y: float) -> bool:
        ind = self.xy2ind(x, y)
        if ind is None:
            return False
        return self.data[ind] == OCC_OCC
