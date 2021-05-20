import argparse
from csv import reader
import math
from typing import Optional

import matplotlib
matplotlib.use("Agg")

import matplotlib.animation as anim
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns


def plot_graph(data_source: str, output: str, xsize: Optional[int], ysize: Optional[int], min: int, max: int):
    results = []
    with open(data_source, "r") as f:
        csv = reader(f)
        for row in csv:
            row_as_floats = []
            for entry in row:
                row_as_floats.append(float(entry))
            results.append(row_as_floats)

    if xsize is None or ysize is None:
        print("No dimensions provided.")
        xsize = ysize = math.sqrt(len(results[0]))
        if xsize.is_integer():
            print("Shape square-able, assuming it is indeed square.")
        else:
            print("Shape not square-able, need dimensions specified.")
            exit(-1)

    results_as_matrix = []
    for result in results:
        results_as_matrix.append(np.reshape(result, (33, 33)))

    fig = plt.figure()
    ax = sns.heatmap(results_as_matrix[0], vmin=min, vmax=max)

    def init():
        plt.clf()
        ax = sns.heatmap(results_as_matrix[0], vmin=min, vmax=max)

    def animate(i):
        plt.clf()
        ax = sns.heatmap(results_as_matrix[i], vmin=min, vmax=max)

    animation = anim.FuncAnimation(fig, animate, init_func=init, frames=len(results_as_matrix), repeat=False, interval=0)

    animation.save(output, writer="imagemagick", fps=60)


def plot_graphs(algorithm: str, count: int):
    for i in range(count):
        plot_graph(f"results/{i}.{algorithm}.pheromone_result.csv", f"results/{i}.{algorithm}.pheromone_result.gif")
        plot_graph(f"results/{i}.{algorithm}.ants_result.csv", f"results/{i}.{algorithm}.ants_result.gif")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--algo", type=str, required=True)
    parser.add_argument("--count", type=int, required=True)
    parser.add_argument("--xsize", type=Optional[int], default=None)
    parser.add_argument("--ysize", type=Optional[int], default=None)
    parser.add_argument("--min", type=int, default=0)
    parser.add_argument("--max", type=int, default=1000)

    args = parser.parse_args()

    plot_graphs(args.algo, args.count, args.xsize, args.ysize, args.min, args.max)
