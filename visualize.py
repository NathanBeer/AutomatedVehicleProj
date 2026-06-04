from __future__ import annotations
from typing import List

import matplotlib.pyplot as plt

from grid import Coord


def plot(grid: List[List[int]], start: Coord, goal: Coord,
         path_astar: List[Coord], path_dijkstra: List[Coord]) -> None:
    plt.imshow(grid, cmap="gray_r")
    plt.scatter(start[1], start[0], c="green", s=100, label="Start")
    plt.scatter(goal[1], goal[0], c="red", s=100, label="Goal")

    if path_astar:
        xs = [p[1] for p in path_astar]
        ys = [p[0] for p in path_astar]
        plt.plot(xs, ys, linewidth=3, label=f"A* (steps={len(path_astar)-1})")

    if path_dijkstra:
        xs = [p[1] for p in path_dijkstra]
        ys = [p[0] for p in path_dijkstra]
        plt.plot(xs, ys, linewidth=2, linestyle="--", label=f"Dijkstra (steps={len(path_dijkstra)-1})")

    plt.legend()
    plt.gca().invert_yaxis()
    plt.title("Path Planning: A* vs Dijkstra")
    plt.show()
