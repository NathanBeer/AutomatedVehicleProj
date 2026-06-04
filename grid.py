from __future__ import annotations
from dataclasses import dataclass
from typing import List, Tuple

Coord = Tuple[int, int]  # (row, col)


@dataclass(frozen=True)
class Config:
    allow_diagonal: bool = False
    avoid_corner_cutting: bool = True  # only matters if allow_diagonal=True


def in_bounds(grid: List[List[int]], p: Coord) -> bool:
    r, c = p
    return 0 <= r < len(grid) and 0 <= c < len(grid[0])


def is_free(grid: List[List[int]], p: Coord) -> bool:
    r, c = p
    return grid[r][c] == 0


def get_neighbors(grid: List[List[int]], p: Coord, cfg: Config):
    r, c = p
    out = [
        ((r - 1, c), 1.0),
        ((r + 1, c), 1.0),
        ((r, c - 1), 1.0),
        ((r, c + 1), 1.0),
    ]

    if cfg.allow_diagonal:
        diag = 1.41421356237
        out += [
            ((r - 1, c - 1), diag),
            ((r - 1, c + 1), diag),
            ((r + 1, c - 1), diag),
            ((r + 1, c + 1), diag),
        ]

    valid = []
    for nxt, cost in out:
        if not in_bounds(grid, nxt) or not is_free(grid, nxt):
            continue

        if cfg.allow_diagonal and cfg.avoid_corner_cutting:
            nr, nc = nxt
            if abs(nr - r) == 1 and abs(nc - c) == 1:
                if not (is_free(grid, (r, nc)) and is_free(grid, (nr, c))):
                    continue

        valid.append((nxt, cost))
    return valid
