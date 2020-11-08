import numpy as np
import matplotlib.pyplot as plt
import argparse
import os.path


def extract_loop_closures(nodes, edges):
    # TODO how should this work if I have multiple loop closures?
    #  loop and plot? or is there some alternating line chart type?
    # look for loop closures (non-consecutive node indices)
    loops = list()
    for edge in edges:
        if int(edge[0]) + 1 != int(edge[1]):
            # print(f"Loop closure from {edge[0]} to {edge[1]}")
            loops.append((nodes[int(edge[0])], nodes[int(edge[1])]))
    return loops



def plot_trajectory(directory):
    true_nodes, true_edges = read_graph(os.path.join(directory, "true_poses.txt"))
    before_nodes, before_edges = read_graph(os.path.join(directory, "noisy_poses.txt"))
    after_nodes, after_edges = read_graph(os.path.join(directory, "optimised_poses.txt"))
    true_xyz = np.array(true_nodes)[:, 1:4]
    before_xyz = np.array(before_nodes)[:, 1:4]
    after_xyz = np.array(after_nodes)[:, 1:4]

    before_loops = extract_loop_closures(before_nodes, before_edges)
    after_loops = extract_loop_closures(after_nodes, after_edges)

    def plot_loops(loops, axis):
        if len(loops) > 0:
            for loop in loops:
                loop_chart_data_xyz = list()
                loop_chart_data_xyz.append(loop[0])
                loop_chart_data_xyz.append(np.array(loop[1]))
                loop_chart_data_xyz = np.array(loop_chart_data_xyz)[:, 1:4]
                axis.plot(loop_chart_data_xyz[:, 0], loop_chart_data_xyz[:, 1], loop_chart_data_xyz[:, 2], c='y', marker=None)

    fig = plt.figure(figsize=plt.figaspect(0.33))
    plt.title('Trajectory')

    ax1 = fig.add_subplot(131, projection='3d')
    ax1.plot(true_xyz[:, 0], true_xyz[:, 1], true_xyz[:, 2], c='g', marker='o')
    ax1.set_title("True")
    ax1.set_xbound(-1, 13)
    ax1.set_ybound(-1, 13)
    ax1.set_zbound(-0.5, 0.5)

    ax2 = fig.add_subplot(132, projection='3d')
    ax2.plot(before_xyz[:, 0], before_xyz[:, 1], before_xyz[:, 2], c='r', marker='o')
    plot_loops(before_loops, ax2)
    ax2.set_title("Before")
    ax2.set_xbound(-1, 13)
    ax2.set_ybound(-1, 13)
    ax2.set_zbound(-0.5, 0.5)

    ax3 = fig.add_subplot(133, projection='3d')
    ax3.plot(after_xyz[:, 0], after_xyz[:, 1], after_xyz[:, 2], c='b', marker='o')
    plot_loops(after_loops, ax3)
    ax3.set_title("After")
    ax3.set_xbound(-1, 13)
    ax3.set_ybound(-1, 13)
    ax3.set_zbound(-0.5, 0.5)

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

    return nodes, edges


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plot the trajectory before and after optimisation.')
    parser.add_argument('directory', help='Directory containing the before and after files')
    args = parser.parse_args()
    if not os.path.isdir(args.directory):
        raise ValueError(f"Provided directory not valid directory : {args.directory}")
    plot_trajectory(args.directory)
