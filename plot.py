import argparse
from csv import reader

import matplotlib
matplotlib.use("Agg")

import matplotlib.animation as anim
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns


def plot_graph(data_source: str, output: str):
    results = []
    with open(data_source, "r") as f:
        csv = reader(f)
        for row in csv:
            row_as_floats = []
            for entry in row:
                row_as_floats.append(float(entry))
            results.append(row_as_floats)

    results_as_matrix = []
    for result in results:
        results_as_matrix.append(np.reshape(result, (53, 53)))

    fig = plt.figure()
    ax = sns.heatmap(results_as_matrix[0], vmin=0, vmax=200)

    def init():
        plt.clf()
        ax = sns.heatmap(results_as_matrix[0], vmin=0, vmax=200)

    def animate(i):
        plt.clf()
        ax = sns.heatmap(results_as_matrix[i], vmin=0, vmax=200)

    animation = anim.FuncAnimation(fig, animate, init_func=init, frames=2000, repeat=False, interval=0)

    animation.save(output, writer="imagemagick", fps=60)


def plot_graphs(algorithm: str, count: int):
    for i in range(count):
        plot_graph(f"results/{i}.{algorithm}.pheromone_result.csv", f"results/{i}.{algorithm}.pheromone_result.gif")
        plot_graph(f"results/{i}.{algorithm}.ants_result.csv", f"results/{i}.{algorithm}.ants_result.gif")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--algo", type=str, required=True)
    parser.add_argument("--count", type=int, required=True)

    args = parser.parse_args()

    plot_graphs(args.algo, args.count)
