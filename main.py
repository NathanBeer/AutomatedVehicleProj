from grid import Config
from algorithms import astar, dijkstra
from visualize import plot

def main() -> None:
    grid = [
        [0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0],
        [0, 0, 0, 1, 0],
        [0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0],
    ]

    start = (0, 0)
    goal = (4, 4)

    cfg = Config(allow_diagonal=False)

    path_a = astar(grid, start, goal, cfg)
    path_d = dijkstra(grid, start, goal, cfg)

    print(f"A*: steps={len(path_a)-1 if path_a else 'no path'}")
    print(f"Dijkstra: steps={len(path_d)-1 if path_d else 'no path'}")

    plot(grid, start, goal, path_a, path_d)

if __name__ == "__main__":
    main()
