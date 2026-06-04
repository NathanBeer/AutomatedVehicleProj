from __future__ import annotations

import heapq
from typing import Dict, List, Tuple

from grid import Config, Coord, get_neighbors, in_bounds, is_free


def reconstruct_path(came_from: Dict[Coord, Coord], start: Coord, goal: Coord) -> List[Coord]:
    cur = goal
    path = [cur]
    while cur != start:
        cur = came_from[cur]
        path.append(cur)
    path.reverse()
    return path


def heuristic(a: Coord, b: Coord, allow_diagonal: bool) -> float:
    (r1, c1), (r2, c2) = a, b
    dr, dc = abs(r1 - r2), abs(c1 - c2)
    if not allow_diagonal:
        return dr + dc
    return (dr + dc) + (1.41421356237 - 2) * min(dr, dc)


def astar(grid: List[List[int]], start: Coord, goal: Coord, cfg: Config) -> List[Coord]:
    if not in_bounds(grid, start) or not in_bounds(grid, goal):
        raise ValueError("Start/goal out of bounds.")
    if not is_free(grid, start) or not is_free(grid, goal):
        return []

    open_heap: List[Tuple[float, int, Coord]] = []
    counter = 0

    came_from: Dict[Coord, Coord] = {}
    g_score: Dict[Coord, float] = {start: 0.0}

    heapq.heappush(open_heap, (heuristic(start, goal, cfg.allow_diagonal), counter, start))
    closed = set()

    while open_heap:
        _, _, current = heapq.heappop(open_heap)

        if current == goal:
            return reconstruct_path(came_from, start, goal)

        if current in closed:
            continue
        closed.add(current)

        for nxt, step_cost in get_neighbors(grid, current, cfg):
            tentative_g = g_score[current] + step_cost
            if tentative_g < g_score.get(nxt, float("inf")):
                came_from[nxt] = current
                g_score[nxt] = tentative_g
                counter += 1
                f = tentative_g + heuristic(nxt, goal, cfg.allow_diagonal)
                heapq.heappush(open_heap, (f, counter, nxt))

    return []


def dijkstra(grid: List[List[int]], start: Coord, goal: Coord, cfg: Config) -> List[Coord]:
    if not in_bounds(grid, start) or not in_bounds(grid, goal):
        raise ValueError("Start/goal out of bounds.")
    if not is_free(grid, start) or not is_free(grid, goal):
        return []

    open_heap: List[Tuple[float, int, Coord]] = []
    counter = 0

    came_from: Dict[Coord, Coord] = {}
    dist: Dict[Coord, float] = {start: 0.0}

    heapq.heappush(open_heap, (0.0, counter, start))
    visited = set()

    while open_heap:
        d, _, current = heapq.heappop(open_heap)

        if current == goal:
            return reconstruct_path(came_from, start, goal)

        if current in visited:
            continue
        visited.add(current)

        for nxt, step_cost in get_neighbors(grid, current, cfg):
            nd = d + step_cost
            if nd < dist.get(nxt, float("inf")):
                dist[nxt] = nd
                came_from[nxt] = current
                counter += 1
                heapq.heappush(open_heap, (nd, counter, nxt))

    return []
