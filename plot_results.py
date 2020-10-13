import numpy as np
import matplotlib.pyplot as plt
import argparse
import os.path

def plot_trajectory(directory):
    before = np.loadtxt(os.path.join(directory, "initial_poses.txt"))
    after = np.loadtxt(os.path.join(directory, "optimised_poses.txt"))
    before_xyz = before[:, 1:4]
    after_xyz = after[:, 1:4]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.grid(True)
    ax.set_title('Trajectory')
    ax.scatter(before_xyz[:, 0], before_xyz[:, 1], before_xyz[:, 2], c='r', marker='o', label="Before")
    ax.scatter(after_xyz[:, 0], after_xyz[:, 1], after_xyz[:, 2], c='g', marker='^', label="After")
    plt.savefig("plot.jpg")
    # print(before_xyz)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plot the trajectory before and after optimisation.')
    parser.add_argument('directory', help='Directory containing the before and after files')
    args = parser.parse_args()
    if not os.path.isdir(args.directory):
        raise ValueError(f"Provided directory not valid directory : {args.directory}")
    plot_trajectory(args.directory)
