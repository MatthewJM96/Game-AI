import argparse
from os.path import join as path_join

from mazelib import Maze
from mazelib.generate.CellularAutomaton import CellularAutomaton
from mazelib.solve.ShortestPath import ShortestPath


def generate_maps(tag: str, dest: str, count: int, xsize: int, ysize: int):
    for i in range(count):
        # Initialise ith maze.
        m = Maze(i)
        
        # Set up generator and generate.
        #   CellularAutomaton is used as it often gives multiple
        #   possible solutions for larger map sizes.
        m.generator = CellularAutomaton(xsize, ysize)
        m.generate()
        m.generate_entrances(False, False)

        with open(path_join(dest, f"{tag}.{i}.unsolved.map"), "w") as f:
            f.write(str(m))

        m.solver = ShortestPath()
        m.solve()

        with open(path_join(dest, f"{tag}.{i}.solved.map"), "w") as f:
            f.write(str(m))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--tag", type=str, required=True)
    parser.add_argument("--dest", type=str, default=".")
    parser.add_argument("--count", type=int, default=10)
    parser.add_argument("--xsize", type=int, default=20)
    parser.add_argument("--ysize", type=int, default=20)

    args = parser.parse_args()

    generate_maps(args.tag, args.dest, args.count, args.xsize, args.ysize)
