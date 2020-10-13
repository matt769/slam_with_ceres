import numpy as np
import matplotlib.pyplot as plt
import argparse
import os.path

def plot_trajectory(directory):
    true = np.loadtxt(os.path.join(directory, "true_poses.txt"))
    before = np.loadtxt(os.path.join(directory, "noisy_poses.txt"))
    after = np.loadtxt(os.path.join(directory, "optimised_poses.txt"))
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


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plot the trajectory before and after optimisation.')
    parser.add_argument('directory', help='Directory containing the before and after files')
    args = parser.parse_args()
    if not os.path.isdir(args.directory):
        raise ValueError(f"Provided directory not valid directory : {args.directory}")
    plot_trajectory(args.directory)
