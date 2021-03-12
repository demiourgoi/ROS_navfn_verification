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


def cells(cols, rows, ratio):
    """ Selects a random number of cells in the map with a given ratio """
    selected_cells = [(c, r) for c in range(1, cols+1) for r in range(1, rows+1)]
    return random.sample(selected_cells, int(cols*rows*ratio))


def map_obstacle_random_test(cols, rows, obstacle_ratio=0.2, mapfilename="map.bin", testfilename="test.txt"):
    """ Generates a map file (binary) with num_cols x rows cells surrounded with
        obstacles in the border (1 pixel). Cells are obstacles with 'prob_obstacle' probability and if they
        are not obstacle their value is between 50 and 253 (uniformly).
        Stores the map and all the possible paths in files
    """
    im = Image.new("L", (cols + 2, rows + 2), OBSTACLE)
    obstacles = cells(cols, rows, obstacle_ratio)
    for x in range(1, rows + 1):
        for y in range(1, cols + 1):
            if (y, x) not in obstacles:
                cell = random.choice(range(FREE_INI, FREE_END+1))
                im.putpixel((y, x), cell)
    paths = all_paths_image(im)
    store_test(im, testfilename, mapfilename, paths)


def map_spiral(cols, rows, mapfilename="map.bin", testfilename="test.txt"):
    """ Generates a map file (binary) with num_cols x rows cells surrounded with
        obstacles in the border (1 pixel). Cells forms an clockwise spiral starting from (1, 1).
        Stores the map and all the possible paths in files
    """
    im = Image.new("L", (cols + 2, rows + 2), FREE_INI)
    for x in range(0, rows + 2):
        im.putpixel((0, x), OBSTACLE)
        im.putpixel((cols+1, x), OBSTACLE)
    for y in range(0, cols + 2):
        im.putpixel((y, 0), OBSTACLE)
        im.putpixel((y, rows+1), OBSTACLE)

    pos = (1, 1)
    delta = (1, 0)
    while im.getpixel(pos) != OBSTACLE:
        while im.getpixel(next_cell(pos, delta)) != OBSTACLE:
            obstacle_right(im, pos, delta)
            pos = next_cell(pos, delta)

        delta = turn_right(delta)
        pos = next_cell(pos, delta)

    paths = all_paths_image(im)
    store_test(im, testfilename, mapfilename, paths)


def next_cell(pos, delta):
    """ Returns the next cell from pos in delta direction """
    return pos[0] + delta[0], pos[1] + delta[1]


def turn_right(delta):
    """ Returns the 90º turn from direction delta """
    right_map = {(1, 0): (0, 1),
                 (0, 1): (-1, 0),
                 (-1, 0): (0, -1),
                 (0, -1): (1, 0)}
    return right_map[delta]


def obstacle_right(im, pos, delta):
    """ Inserts and obstacle to the right of pos wrt. the direction delta if the next cell is not an obstacle """
    right_dir = turn_right(delta)
    right_cell = next_cell(pos, right_dir)
    if im.getpixel(next_cell(pos, delta)) != OBSTACLE:
        im.putpixel(right_cell, OBSTACLE)


def all_paths_image(im):
    """Returns a list with all the (start, end) cells that are not in
       an obstacle
    """
    w, h = im.size
    all_cells = [(x0, y0) for x0 in range(w) for y0 in range(h)]
    ret = list()
    for start in all_cells:
        for end in all_cells:
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
    # map_free_test(4, 4)
    map_spiral(5, 5)


if __name__ == "__main__":
    main()
