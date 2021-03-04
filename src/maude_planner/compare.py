# -*- coding: utf-8 -*-

"""
Check navigation results from ROS2 and Maude implementation
Author: Enrique Martin
"""

import argparse
import json
import math
import sys
import re

EPSILON = 1E-5
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
    res = None
    if len(path) <= 1 or path[-1] != path[-2]:
        res = path
    else:
        res = path[:-1]
    return res
    
    
def equal_epsilon(cell1, cell2, epsilon=EPSILON):
    """Compares if two cells are equal with a margin of epsilon"""
    x1, y1 = cell1
    x2, y2 = cell2
    ret = abs(x1-x2) <= epsilon and abs(y1-y2) <= epsilon
    return ret
            
    
def path_equal(path1, path2):
    """Checks if path1 and path2 are equal"""
    if len(path1) != len(path2):
        return False
    pos = 0
    while pos < len(path1) and equal_epsilon(path1[pos], path2[pos]):
        pos += 1
    return pos == len(path1)


class Plotter:
    """Plotter of potentials and paths using matplotlib"""

    def __init__(self, name, draw, width):
        self.width = width
        
        # This may be a dummy plotter if draw is false. The packages are only
        # imported if drawing is required.
        if draw:
            try:
                import numpy as np
                import matplotlib.pyplot as plt
                from matplotlib.patches import Rectangle
                from matplotlib.backends.backend_pdf import PdfPages

            except ImportError as ie:
                print(f'Some required packages for plotting are missing: {ie}')
                draw = False

        # We want and we can draw
        if draw:
            self.np = np
            self.plt = plt
            self.Rectangle = Rectangle
            self.pdf = PdfPages(name)

        else:
            self.np = None
            self.plt = None
            self.Rectangle = None
            self.pdf = None

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

    def draw_diff(self, potarr1, potarr2, path1, path2):
        """Draw the difference of the given potentials and their paths"""

        diff = potarr1 - potarr2
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
                color = 'red' if value < 0.0 else 'aqua'
                ax.add_patch(self.Rectangle([y, x], 1, 1, edgecolor=color))

        # Finally, both paths
        self.plt.plot(*zip(*path1), color='aqua')
        self.plt.plot(*zip(*path2), color='red')

        # Plot the extra points of each path
        # (we should probably ignore some epsilon)
        extra1 = [p for p in path1 if p not in path2]
        extra2 = [p for p in path2 if p not in path1]

        not extra1 or self.plt.scatter(*zip(*extra1), color='aqua')
        not extra2 or self.plt.scatter(*zip(*extra2), color='red')

    def draw(self, origin, dest, potarr1_raw, potarr2_raw, path1, path2, equal):
        """Plots potentials, paths and their differences"""

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

        # ROS potential and path
        self.draw_potential(potarr1, path1, pcolor='aqua')
        self.plt.title(f'{origin} to {dest} — ROS')
        self.pdf.savefig()
        self.plt.close()

        # Maude potential and path
        self.draw_potential(potarr2, path2, pcolor='red')
        self.plt.title(f'{origin} to {dest} — Maude')
        self.pdf.savefig()
        self.plt.close()

        # Difference and paths
        self.draw_diff(potarr1, potarr2, path1, path2)
        self.plt.title(f'{origin} to {dest} {"EQ" if equal else "DIFF"}')
        self.pdf.savefig()
        self.plt.close()


def main(args):
    """Compares the results stored in args.ros_output and args.maude_output, showing the result in the standard output.
       Writes the potentials to a PDF if args.draw is enabled.
    """
    dict1 = file_to_dict(args.ros_output)
    dict2 = file_to_dict(args.maude_output)
    key_set1 = set(dict1.keys())
    key_set2 = set(dict1.keys())
    
    tests = sorted(list(key_set1))
    if key_set1 != key_set2:
        print(f"{file1} - {file2}: {key_set1 - key_set2}")
        print(f"{file2} - {file1}: {key_set2 - key_set1}")
        tests = sorted(list(key_set1.intersect(key_set2)))

    num_diff = 0
    with Plotter('potentials.pdf', args.draw, args.width) as plotter:
        for test in tests:
            path1 = dict1[test]["path"]
            path2 = dict2[test]["path"]

            potarr1 = dict1[test]["navfn"]
            potarr2 = dict2[test].get("navfn")

            equal = path_equal(path1, path2)

            if not equal:
                num_diff += 1
                print(f'{num_diff}) Differences in the path from {test[0]} to {test[1]}')
                print(f'ROS  : {path1}')
                print(f'Maude: {path2}')
                print(f'potarr: {dict1[test]["navfn"]}\n')

            # Draw the potentials and paths
            plotter.draw(test[0], test[1], potarr1, potarr2, path1, path2, equal)
            
        if num_diff > 0:
            exit(-1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Compare ROS and Maude results')
    parser.add_argument('ros_output', help='JSON test results produced by profile_cpp')
    parser.add_argument('maude_output', help='JSON test results produced by profile_maude')
    parser.add_argument('--draw', help='Draw navigation functions and paths (requires matplotlib)', action='store_true')
    parser.add_argument('--width', '-w', help='Width of the map (if not square)', type=int)

    args = parser.parse_args()
    main(args)
