# -*- coding: utf-8 -*-

"""
Show a binary map using PIL. Obstacles are shown in blue, rest of cells in grayscale
Author: Enrique Martin
"""

import ast
import argparse
import math
import os
import sys

OBSTACLE = 254


def midpoint(x, y):
    """Middle point between other two"""

    return [(a + b) / 2 for a, b in zip(x, y)]


def calculate_cost(costmap, path):
    """Calculate the cost of a path"""
    
    cost = 0.0

    for k in range(len(path)-1):
        i, j = map(math.floor, midpoint(path[k], path[k+1]))
        cost += math.dist(path[k], path[k+1]) * costmap[j, i]
        
    return cost


def show_using_numpy(map_full_map_path, w, h, args, lines):
    import numpy as np
    import matplotlib.pyplot as plt

    costmap = np.fromfile(map_full_map_path, dtype=np.ubyte).reshape((h, w))

    # Create a colormap where obstacles and passable cells are gray
    gray_scale = [(f, f, f, 1) for f in np.linspace(1, .25, 254 - 50)]
    cm = plt.cm.colors.ListedColormap(gray_scale + [(1, 0, 0, 1)])

    w, h = costmap.shape
    plt.imshow(costmap, cmap=cm, extent=(-.5, h - .5, w - .5, -.5))

    # Overlay the path if given
    if args.path:
        path = ast.literal_eval(args.path)
        plt.plot(*zip(*path), color=args.path_color, marker='o')

        print('Path cost:', calculate_cost(costmap, path))

    # Overlay the test cases if enabled
    if args.cases:
        # Read the test cases from the txt file
        cases = [map(int, line.strip().split(' ')) for line in lines[3:]]

        for x0, y0, x, y in cases:
            plt.arrow(x0, y0, x - x0, y - y0, color=args.path_color,
                      length_includes_head=True, head_width=0.002 * w)

    # Add a title to the map if given
    if args.title:
        plt.title(args.title)

    if args.colorbar:
        plt.colorbar()

    # Show or write the result
    if args.o:
        plt.tight_layout()
        plt.savefig(args.o)
    else:
        plt.show()


def show_using_pil(map_full_map_path, w, h, args, lines):
    from PIL import Image

    with open(map_full_map_path, "rb") as mapfile:
        im = Image.new("L", (w, h))
        im.frombytes(mapfile.read())

        # Generates a color version of map with blue obstacles
        im_color = Image.new("RGB", (w, h))
        for col in range(w):
            for row in range(h):
                cell = im.getpixel((col, row))
                if cell == OBSTACLE:
                    im_color.putpixel((col, row), (102, 204, 255))
                else:
                    im_color.putpixel((col, row), (cell, cell, cell))

        ratio = round(600 / w)
        resized = im_color.resize((w*ratio, h*ratio), resample=Image.NEAREST)
        
        if args.o:
            resized.save(args.o)
        else:
            resized.show()


def show_map(args):
    with open(args.test_file, "r") as testfile:
        lines = testfile.readlines()
        w, h = (int(e) for e in lines[0].split())
        mapfilename = lines[1].strip()
        test_full_path = os.path.abspath(args.test_file)
        test_dir = os.path.dirname(test_full_path)
        map_full_map_path = os.path.join(test_dir, mapfilename)
        print(map_full_map_path)

    # Show the map using NumPy or PIL depending on the argument
    show_fn = show_using_numpy if args.numpy or args.path or args.cases else show_using_pil

    show_fn(map_full_map_path, w, h, args, lines)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Map viewer')
    parser.add_argument('test_file', help='Test specification file')
    parser.add_argument('--numpy', help='Use NumPy-Matplotlib instead of PIL to show the map',
                        action='store_true')
    parser.add_argument('--path', help='Print a path on the map, given a Python list of pairs over (implies --numpy)')
    parser.add_argument('--cases', help='Print the test cases (initial and final pose) on the map (implies --numpy)', action='store_true')
    parser.add_argument('--title', help='Print a title on top of the map')
    parser.add_argument('--colorbar', help='Add a colorbar to the picture', action='store_true')
    parser.add_argument('--path-color', help='Color of paths', default='blue')
    parser.add_argument('-o', help='Output the drawing to a file')

    show_map(parser.parse_args())
