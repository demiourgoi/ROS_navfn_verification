# -*- coding: utf-8 -*-

"""
Show a binary map using PIL. Obstacles are shown in blue, rest of cells in grayscale
Author: Enrique Martin
"""

import argparse
import os
import sys

OBSTACLE = 254


def show_using_numpy(map_full_map_path, w, h):
    import numpy as np
    import matplotlib.pyplot as plt

    cmap = np.fromfile(map_full_map_path, dtype=np.ubyte).reshape((w, h))

    plt.imshow(cmap)
    plt.show()


def show_using_pil(map_full_map_path, w, h):
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
    show_fn = show_using_numpy if args.numpy else show_using_pil

    show_fn(map_full_map_path, w, h)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Map viewer')
    parser.add_argument('test_file', help='Test specification file')
    parser.add_argument('--numpy', help='Use NumPy-Matplotlib instead of PIL to show the map',
                        action='store_true')

    show_map(parser.parse_args())
