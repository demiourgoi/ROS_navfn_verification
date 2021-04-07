# -*- coding: utf-8 -*-

"""
Check navigation results from ROS2 and Maude implementation
Author: Enrique Martin
"""

import argparse
import json
import re

EPSILON = 1e-5  # Epsilon to decide when two positions are the same
POT_EPSILON = 5E-3  # Epsilon to draw or not dots in the plot
JSON_REGEX = re.compile(r"(^\{.+\})")


def file_to_dict(filename):
    """Loads a file with JSON lines into a dictinary (initial, final) -> path"""
    results = dict()
    with open(filename, 'r') as json_file:
        lines = json_file.readlines()
        for line in lines:
            result = re.search(JSON_REGEX, line.strip()) 
            if not result:  # Ignores lines with debug information
                continue
            line_dict = json.loads(result.group(1))
            # Line has the following format:
            # {"initial": [1, 1], "goal": [1, 1], "duration": 6.21e-07, "length": 0, "path": [[1, 1], [1, 1]], 
            #  "navfn": [10000000000.0, 10000000000.0, 50, 10000000000.0,...] }
            initial = tuple(line_dict["initial"])
            goal = tuple(line_dict["goal"])
            path = [tuple(pose) for pose in line_dict["path"]]
            potentials = line_dict.get("navfn", None)
            results[(initial, goal)] = {"path": path, "navfn": potentials}
    return results
    

def remove_last(path):
    """Removes the last pose if duplicated. NOT NEEDED NOW"""
    if len(path) <= 1 or path[-1] != path[-2]:
        res = path
    else:
        res = path[:-1]
    return res


def detect_oscillation(path, epsilon):
    """ Detects the first oscillation in the path. An oscillation starts when path[i] ~ path[i+2] and finishes in the
        first j > i+2 such that path[j] ~/~ path[j-2], i.e., the oscillation to remove is [i+1, j)
    """
    for (i, cell) in enumerate(path):
        if i + 2 < len(path) and equal_epsilon(path[i], path[i + 2], epsilon):
            for j in range(i + 3, len(path)):
                if not equal_epsilon(path[j], path[j - 2], epsilon):
                    return i+1, j
    return None  # No oscillation found


def remove_oscillations(path, epsilon):
    """ Removes the oscillations in the path. An oscillation starts when path[i] ~ path[i+2] and finishes in the
        first j > i+2 such that path[j] ~/~ path[j-2], i.e., the oscillation is [i, j-1] and the fragment of path
        removed is [i+1, j-1]"""
    oscillation = detect_oscillation(path, epsilon)
    if oscillation is not None:
        return remove_oscillations(path[:oscillation[0]] + path[oscillation[1]:], epsilon)
    return path  # No oscillation found


def equal_epsilon(cell1, cell2, epsilon):
    """Compares if two cells are equal with a margin of epsilon"""
    x1, y1 = cell1
    x2, y2 = cell2
    ret = abs(x1-x2) <= epsilon and abs(y1-y2) <= epsilon
    return ret
            
    
def path_equal(path1, path2, epsilon):
    """ Checks if path1 and path2 are equal, using epsilon to check if two positions are equal and removing
        oscillatoins
    """
    clean_path1 = remove_oscillations(path1, epsilon)
    clean_path2 = remove_oscillations(path2, epsilon)
    if len(clean_path1) != len(clean_path2):
        return False
    pos = 0
    while pos < len(clean_path1) and equal_epsilon(clean_path1[pos], clean_path2[pos], epsilon):
        pos += 1
    return pos == len(clean_path1)


class Plotter:
    """Plotter of potentials and paths using matplotlib"""

    def __init__(self, name, draw, width):
        self.width = width
        self.mode = draw

        # This may be a dummy plotter if draw is false. The packages are only
        # imported if drawing is required.
        if draw != 'none':
            try:
                import numpy as np
                import matplotlib.pyplot as plt
                from matplotlib.patches import Rectangle
                from matplotlib.backends.backend_pdf import PdfPages
                import matplotlib
                matplotlib.use('PDF')  # Avoids loading any GUI, may increase speed

            except ImportError as ie:
                print(f'Some required packages for plotting are missing: {ie}')
                draw = 'none'

        # We want and we can draw
        if draw != 'none':
            self.np = np
            self.plt = plt
            self.Rectangle = Rectangle
            self.pdf = PdfPages(name)

        else:
            self.np = None
            self.plt = None
            self.Rectangle = None
            self.pdf = None

        # Difference between potentials
        self.dist2 = []
        self.distInf = []

    def __enter__(self):
        return self

    def __exit__(self, *args):
        if self.pdf is not None:
            self.pdf.close()

    def draw_potential(self, potarr, path, pcolor='red'):
        """Draw the potential and path produced by a backend"""

        # The maximum value not being the pseudo-infinity
        # (the minimum should always be zero)
        vmax = potarr[self.np.abs(potarr) < 1e5].max()

        # Draw the potential as a image omitting infinity
        # (otherwise, the other differences could not be seen)
        # (extent is used to make coordinates be in the corners)
        self.plt.imshow(potarr, vmax=vmax, extent=(0, potarr.shape[0], potarr.shape[1], 0))
        self.plt.colorbar()

        # Draw red squares in the infinity
        ax = self.plt.gca()

        for (x, y), value in self.np.ndenumerate(potarr):
            if abs(value) > 1e5:
                ax.add_patch(self.Rectangle([y, x], 1, 1, edgecolor='red', facecolor='red'))

        # Finally, the path
        self.plt.plot(*zip(*path), color=pcolor, marker='o')

    def draw_diff(self, diff, path1, path2):
        """Draw the given difference of two potentials and their paths"""

        support = self.np.abs(diff) < 1e5

        # The maximum and minimum value where infinity is not present
        # (in order to see differences in the output for smaller changes)
        vmax = diff[support].max()
        vmin = diff[support].min()

        self.plt.imshow(diff, vmin=vmin, vmax=vmax, extent=(0, diff.shape[0], diff.shape[1], 0))
        self.plt.colorbar()

        # Draw red (green) squares in the infinity values of Maude (ROS)
        ax = self.plt.gca()

        for (x, y), value in self.np.ndenumerate(diff):
            if abs(value) > 1e5:
                color = 'blue' if value < 0.0 else 'aqua'
                ax.add_patch(self.Rectangle([y, x], 1, 1, edgecolor=color))

        # Finally, both paths
        self.plt.plot(*zip(*path1), color='aqua')
        self.plt.plot(*zip(*path2), color='blue')

        # Plot the extra points of each path
        extra1 = [p for k, p in enumerate(path1) if k >= len(path2) or not equal_epsilon(p, path2[k], EPSILON)]
        extra2 = [p for k, p in enumerate(path2) if k >= len(path1) or not equal_epsilon(p, path1[k], EPSILON)]

        not extra1 or self.plt.scatter(*zip(*extra1), color='aqua')
        not extra2 or self.plt.scatter(*zip(*extra2), color='blue')

    def draw(self, origin, dest, potarr1_raw, potarr2_raw, path1, path2, equal):
        """ Plots potentials, paths and their differences """

        # If we are not drawing or do not have enough information
        if self.np is None or potarr2_raw is None:
            return

        # Calculate the width and height of the map
        # (it should common for all cases though)
        width = int(self.np.sqrt(len(potarr1_raw))) if self.width is None else self.width
        height = len(potarr1_raw) // width

        # Convert the potentials to NumPy arrays
        potarr1 = self.np.array(potarr1_raw)
        potarr2 = self.np.array(potarr2_raw)
        potarr1.shape = (width, height)
        potarr2.shape = (width, height)

        # Maude's potentials are transposed
        potarr2 = self.np.transpose(potarr2)

        # Difference of the two potentials and its norm
        diff = potarr1 - potarr2
        self.distInf.append(self.np.linalg.norm(diff, self.np.inf))
        self.dist2.append(self.np.linalg.norm(diff, 2))

        # Skip drawing if they are equal and the differences are negligible
        if self.mode == 'failed' and equal and self.dist2[-1] < POT_EPSILON:
            return

        # ROS potential and path
        self.draw_potential(potarr1, path1, pcolor='aqua')
        self.plt.title(f'{origin} to {dest} — ROS')
        self.pdf.savefig()
        self.plt.close()

        # Maude potential and path
        self.draw_potential(potarr2, path2, pcolor='blue')
        self.plt.title(f'{origin} to {dest} — Maude')
        self.pdf.savefig()
        self.plt.close()

        # Difference and paths
        self.draw_diff(diff, path1, path2)
        self.plt.title(f'{origin} to {dest} {"EQ" if equal else "DIFF"} ({self.dist2[-1]:.2e})')
        self.pdf.savefig()
        self.plt.close()

    def draw_summary(self):
        """Plot a summary of the data collected for each case"""

        if self.np is None:
            return

        # print(f'{max(self.distInf):.2e}', file=sys.stderr)
        self.plt.title(f'Potential distance plot')
        self.plt.boxplot([self.distInf, self.dist2], labels=['d_inf', 'd_2'])
        self.pdf.savefig()
        self.plt.close()


def main(args):
    """Compares the results stored in args.ros_output and args.maude_output, showing the result in the standard output.
       Writes the potentials to a PDF if args.draw is enabled.
    """
    dict1 = file_to_dict(args.ros_output)
    dict2 = file_to_dict(args.maude_output)
    key_set1 = set(dict1.keys())
    key_set2 = set(dict2.keys())
    
    tests = sorted(list(key_set1))
    if key_set1 != key_set2:
        print(f"{args.ros_output} and {args.maude_output} contain different tests")
        print("==================================================================")
        print(f"{args.ros_output} \\ {args.maude_output}: {key_set1 - key_set2}")
        print(f"{args.maude_output} \\ {args.ros_output}: {key_set2 - key_set1}")
        tests = sorted(list(key_set1 & key_set2))
        exit(-2)

    num_diff = 0
    epsilon = None

    with Plotter('potentials.pdf', args.draw, args.width) as plotter:
        for test in tests:
            path1 = dict1[test]["path"]
            path2 = dict2[test]["path"]

            potarr1 = dict1[test]["navfn"]
            potarr2 = dict2[test].get("navfn")

            # The epsilon constant is adapted to the map size
            if not epsilon:
                side_length = len(potarr1) ** 0.5 if potarr1 is not None else 11

                if side_length >= 12:
                    epsilon = 0.05
                elif side_length == 11:
                    epsilon = 0.005
                else:
                    epsilon = EPSILON

            equal = path_equal(path1, path2, epsilon)

            if not equal:
                num_diff += 1
                print(f'{num_diff}) Differences in the path from {test[0]} to {test[1]}')
                ros_osc = "(removed oscillations)" if detect_oscillation(path1, epsilon) is not None else ""
                maude_osc = "(removed oscillations)" if detect_oscillation(path2, epsilon) is not None else ""
                print(f'ROS {ros_osc} : {path1}')
                print(f'Maude {maude_osc}: {path2}')
                if len(path1) == len(path2):
                    print('Max. difference: ', max((abs(path1[i][j] - path2[i][j]) for i in range(len(path1)) for j in (0, 1))))
                print(f'potarr: {dict1[test]["navfn"]}\n')

                # Draw the potentials and paths only for different paths, removing any oscillation in those paths
                plotter.draw(test[0], test[1], potarr1, potarr2, remove_oscillations(path1, epsilon),
                             remove_oscillations(path2, epsilon), equal)

        plotter.draw_summary()

        if num_diff > 0:
            exit(-1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Compare ROS and Maude results')
    parser.add_argument('ros_output', help='JSON test results produced by profile_cpp')
    parser.add_argument('maude_output', help='JSON test results produced by profile_maude')
    parser.add_argument('--draw', help='Draw navigation functions and paths (requires matplotlib)', nargs='?',
                        choices=['none', 'failed', 'all'], default='none', const='failed')
    parser.add_argument('--width', '-w', help='Width of the map (if not square)', type=int)

    args = parser.parse_args()
    main(args)
