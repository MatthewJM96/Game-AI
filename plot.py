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
        results_as_matrix.append(np.reshape(result, (8, 8)))

    # im = plt.imshow(results_as_matrix[0], cmap="hot", interpolation="nearest")
    # plt.show()

    fig = plt.figure()
    ax = sns.heatmap(results_as_matrix[0], vmin=0, vmax=25)

    def init():
        plt.clf()
        ax = sns.heatmap(results_as_matrix[0], vmin=0, vmax=25)

    def animate(i):
        plt.clf()
        ax = sns.heatmap(results_as_matrix[i], vmin=0, vmax=25)

    animation = anim.FuncAnimation(fig, animate, init_func=init, frames=1000, repeat=False, interval=0)

    # plt.show()

    animation.save(output, writer="imagemagick", fps=60)

plot_graph("results/pheromone_result.csv", "results/pheromone_result.gif")
plot_graph("results/ants_result.csv", "results/ants_result.gif")
