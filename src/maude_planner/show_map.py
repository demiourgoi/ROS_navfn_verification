# -*- coding: utf-8 -*-

"""
Show a binary map using PIL. Obstacles are shown in blue, rest of cells in grayscale
Author: Enrique Martin
"""

from PIL import Image
import os
import sys

OBSTACLE = 254


def show_map(testfilename):
    with open(testfilename, "r") as testfile:
        lines = testfile.readlines()
        w, h = (int(e) for e in lines[0].split())
        mapfilename = lines[1].strip()
        test_full_path = os.path.abspath(testfilename)
        test_dir = os.path.dirname(test_full_path)
        map_full_map_path = os.path.join(test_dir, mapfilename)
        print(map_full_map_path)

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


if __name__ == "__main__":
    show_map(sys.argv[1])
