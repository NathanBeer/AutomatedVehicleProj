"""Bridges the CARLA HD map to the discrete grid expected by algorithms.py."""
from __future__ import annotations

import math
from typing import List, Tuple

import carla

from grid import Coord

_MARGIN_CELLS = 12

class WorldGrid:
    """
    Discretises a rectangular slice of the CARLA world into a 2-D grid.
    """

    def __init__(
        self,
        world: carla.World,
        start: carla.Location,
        goal: carla.Location,
        cell_size: float = 2.0,
    ) -> None:
        self.cell_size = cell_size
        self._map = world.get_map()
        self.grid, self.origin, self.rows, self.cols = self._build(start, goal)

    def _build(
        self,
        start: carla.Location,
        goal: carla.Location,
    ) -> Tuple[List[List[int]], Tuple[float, float], int, int]:
        margin = self.cell_size * _MARGIN_CELLS
        x_min = min(start.x, goal.x) - margin
        x_max = max(start.x, goal.x) + margin
        y_min = min(start.y, goal.y) - margin
        y_max = max(start.y, goal.y) + margin

        cols = math.ceil((x_max - x_min) / self.cell_size) + 1
        rows = math.ceil((y_max - y_min) / self.cell_size) + 1
        origin: Tuple[float, float] = (x_min, y_min)

        grid: List[List[int]] = [[1] * cols for _ in range(rows)]

        for wp in self._map.generate_waypoints(self.cell_size):
            loc = wp.transform.location
            if not (x_min <= loc.x <= x_max and y_min <= loc.y <= y_max):
                continue
            r, c = self._to_rc(loc, origin)
            if 0 <= r < rows and 0 <= c < cols:
                grid[r][c] = 0

        return grid, origin, rows, cols

    def _to_rc(
        self, loc: carla.Location, origin: Tuple[float, float] | None = None
    ) -> Coord:
        ox, oy = origin if origin is not None else self.origin
        return (
            int((loc.y - oy) / self.cell_size),
            int((loc.x - ox) / self.cell_size),
        )

    def world_to_grid(self, loc: carla.Location) -> Coord:
        return self._to_rc(loc)

    def grid_to_world(self, coord: Coord) -> carla.Location:
        r, c = coord
        ox, oy = self.origin
        x = ox + (c + 0.5) * self.cell_size
        y = oy + (r + 0.5) * self.cell_size
        wp = self._map.get_waypoint(
            carla.Location(x=x, y=y, z=0.5), project_to_road=True
        )
        return wp.transform.location if wp else carla.Location(x=x, y=y, z=0.3)