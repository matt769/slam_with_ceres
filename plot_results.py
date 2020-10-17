import numpy as np
import matplotlib.pyplot as plt
import argparse
import os.path


def plot_trajectory(directory):
    true, _ = read_graph(os.path.join(directory, "true_poses.txt"))
    before, _ = read_graph(os.path.join(directory, "noisy_poses.txt"))
    after, _ = read_graph(os.path.join(directory, "optimised_poses.txt"))
    true_xyz = true[:, 1:4]
    before_xyz = before[:, 1:4]
    after_xyz = after[:, 1:4]

    fig = plt.figure(figsize=plt.figaspect(0.33))
    plt.title('Trajectory')
    ax1 = fig.add_subplot(131, projection='3d')
    ax1.scatter(true_xyz[:, 0], true_xyz[:, 1], true_xyz[:, 2], c='g', marker='o')
    ax1.set_title("True")
    ax2 = fig.add_subplot(132, projection='3d')
    ax2.scatter(before_xyz[:, 0], before_xyz[:, 1], before_xyz[:, 2], c='r', marker='o')
    ax2.set_title("Before")
    ax3 = fig.add_subplot(133, projection='3d')
    ax3.scatter(after_xyz[:, 0], after_xyz[:, 1], after_xyz[:, 2], c='b', marker='o')
    ax3.set_title("After")

    plt.savefig("plot.jpg")


def read_graph(filename):
    if not os.path.exists(filename):
        raise ValueError(f"File does not exist: {filename}")

    nodes = list()
    edges = list()
    read_node = False
    read_edge = False
    file = open(filename)
    for line in file.readlines():
        if not read_node and not read_edge and line[0:5] == "Nodes":
            read_node = True
            continue
        elif read_node and not read_edge and line[0:5] == "Edges":
            read_node = False
            read_edge = True
            continue
        # else should be a line of data
        if read_node:
            nodes.append([float(x) for x in line.split()])
        if read_edge:
            edges.append([float(x) for x in line.split()])
    nodes = np.array(nodes)
    edges = np.array(edges)

    return nodes, edges


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plot the trajectory before and after optimisation.')
    parser.add_argument('directory', help='Directory containing the before and after files')
    args = parser.parse_args()
    if not os.path.isdir(args.directory):
        raise ValueError(f"Provided directory not valid directory : {args.directory}")
    plot_trajectory(args.directory)
