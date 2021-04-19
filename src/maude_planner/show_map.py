# -*- coding: utf-8 -*-

"""
Show a binary map using PIL. Obstacles are shown in blue, rest of cells in grayscale
Author: Enrique Martin
"""

import ast
import argparse
import os
import sys

OBSTACLE = 254


def show_using_numpy(map_full_map_path, w, h, args):
    import numpy as np
    import matplotlib.pyplot as plt


    costmap = np.fromfile(map_full_map_path, dtype=np.ubyte).reshape((h, w))
    
    # Create a colormap where obstacles and passable cells are gray
    gray_scale = [(f, f, f, 1) for f in np.linspace(1, .25, 254 - 50)]
    cm = plt.cm.colors.ListedColormap(gray_scale + [(1, 0, 0, 1)])
    
    w, h = costmap.shape
    plt.imshow(costmap, cmap=cm, extent=(0, h, w, 0))
    
    # Overlay the path if given
    if args.path:
        path = ast.literal_eval(args.path)
        plt.plot(*zip(*path), color='blue', marker='o')
    
    # Show or write the result
    if args.o:
        plt.savefig(args.o)
    else:
        plt.show()


def show_using_pil(map_full_map_path, w, h, args):
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
    show_fn = show_using_numpy if args.numpy or args.path else show_using_pil

    show_fn(map_full_map_path, w, h, args)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Map viewer')
    parser.add_argument('test_file', help='Test specification file')
    parser.add_argument('--numpy', help='Use NumPy-Matplotlib instead of PIL to show the map',
                        action='store_true')
    parser.add_argument('--path', help='Print a path the map, given a Python list of pairs over (implies --numpy)')
    parser.add_argument('-o', help='Output the drawing to a file')

    show_map(parser.parse_args())
