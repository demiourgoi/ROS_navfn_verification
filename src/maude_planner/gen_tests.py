# -*- coding: utf-8 -*-

"""
Automatic generation of maps and tests for navigation
Author: Enrique Martin
"""

# IMPORTANT: PIL follows a cartesian coordinate system, i.e., image.getpixel((x,y)) means
# x column, y row

from PIL import Image
import random

OBSTACLE = 254
FREE_INI = 50
FREE_END = 253


def store_test(im, testfilename, mapfilename, paths):
    """Generates a binary map and test file from an image and a set of paths to test"""
    with open(mapfilename, "wb") as mapfile:
        mapfile.write(im.tobytes())

    with open(testfilename, "w") as testfile:
        w, h = im.size
        testfile.write(f"{w} {h}\n")
        testfile.write(f"{mapfilename}\n")
        testfile.write("\n")
        for start, end in paths:
            testfile.write(f"{start[0]} {start[1]} {end[0]} {end[1]}\n")


def map_free_test(cols, rows, mapfilename="map.bin", testfilename="test.txt"):
    """Generates a map file (binary) with num_cols x rows cells surrounded with 
       obstacles in the border (1 pixel) and all the cells completely free
       Stores the map and all the possible paths in files
    """
    im = Image.new("L", (cols+2, rows+2), OBSTACLE)
    for y in range(1, rows+1):
        for x in range(1, cols + 1):
            im.putpixel((x, y), FREE_INI)

    paths = all_paths_image(im)
    store_test(im, testfilename, mapfilename, paths)


def map_free_random_test(cols, rows, mapfilename="map.bin", testfilename="test.txt"):
    """Generates a map file (binary) with num_cols x rows cells surrounded with
       obstacles in the border (1 pixel) and the cells free with random values
       Stores the map and all the possible paths in files
    """
    im = Image.new("L", (cols + 2, rows + 2), OBSTACLE)
    for x in range(1, rows + 1):
        for y in range(1, cols + 1):
            cell = random.choice(range(FREE_INI, FREE_END+1))
            im.putpixel((y, x), cell)

    paths = all_paths_image(im)
    store_test(im, testfilename, mapfilename, paths)


def all_paths_image(im):
    """Returns a list with all the (start, end) cells that are not in
       an obstacle
    """
    w, h = im.size
    cells = [(x0, y0) for x0 in range(w) for y0 in range(h)]
    ret = list()
    for start in cells:
        for end in cells:
            if im.getpixel(start) != OBSTACLE and im.getpixel(end) != OBSTACLE:
                ret.append((start, end))
    return ret
    

def all_paths(cell_list):
    """Generates all the combinations start-end using the cells in cell_list"""
    [print(x0, y0, x1, y1) for x0, y0 in cell_list for x1, y1 in cell_list]
    return [(start, end) for start in cell_list for end in cell_list]
    
    
def main():
    # 3x3 Descencing diagonal
    # cells = [(1,2), (1,3), (2,3), (2,1), (3,1), (3,2)]   
    # 3x3 Ascencing diagonal
    # cells = [(1,1), (1,2), (2,1), (2,3), (3,2), (3,3)]   
    # all_paths(cells)
    # map_free_test(5, 5, "5x5_free.bin", "test_5x5_free.txt")
    # map_free_random_test(3, 3, "3x3_free_random.bin", "test_3x3_free_random.txt")
    # map_free_random_test(4, 4)
    map_free_test(4, 4)


if __name__ == "__main__":
    main()
