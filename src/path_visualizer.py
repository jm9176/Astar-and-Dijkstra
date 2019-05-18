import numpy as np
import matplotlib.pyplot as plt


class PathVisualizer(object):
    """Helper class to visualize the path, query and grid."""

    @staticmethod
    def viz_path(grid, query, path):
        plt.axis([0, grid.shape[0], 0, grid.shape[1]])
        plt.scatter(np.where(grid)[0], np.where(grid)[1], s=50, marker='s')

        for i in range(1,len(path)):

            plt.plot([path[i].x, path[i-1].x], [path[i].y, path[i-1].y], 'r-', linewidth = 2.0)
            if path[i-1] == query[0]:
                plt.text(path[i-1].x, path[i-1].y, 'S')
            if path[i] == query[1]:
                plt.text(path[i].x, path[i].y, 'G')
            

        plt.ion()
        plt.show()
        raw_input("Press Enter to close plot...")
        plt.close()
