# -*- coding: utf-8 -*-

"""
Automatic generation of maps and tests for navigation
Author: Enrique Martin
"""

# IMPORTANT: PIL follows a cartesian coordinate system, i.e., image.getpixel((x,y)) means
# x column, y row

from PIL import Image
import random
import math
import yaml

OBSTACLE = 254
FREE_INI = 50
FREE_END = 253
MAX_CELL = 255
COST_FACTOR = 0.8


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


def test_from_yaml(path, num_tests):
    """ Create a test with random paths from a YAML+PNG describing a map """
    with open(path, 'r') as yaml_file:
        yaml_content = yaml.load(yaml_file, Loader=yaml.SafeLoader)
    im = Image.open(yaml_content['image'])
    adapt_pgm(im, yaml_content['occupied_thresh'], yaml_content['free_thresh'], yaml_content['negate'])
    paths = random_paths_image_close(im, num_tests, 10)
    store_test(im, 'test.txt', 'map.bin', paths)


def adapt_pgm(im, occupied_thresh, free_thresh, negate):
    """ Modifies the image setting pixel value in [FREE_INI, OBSTACLE] using thresholds.
        Ideally should follow the instructions in
        https://github.com/ros-planning/navigation2/blob/30b405c58e6d53ba8c96381416bc4679d35a1483/nav2_map_server/src/map_io.cpp#L208
        and use the negate field to check occupancy:
           // If negate is true, we consider blacker pixels free, and whiter
           // pixels occupied. Otherwise, it's vice versa.
           /// on a scale from 0.0 to 1.0, how occupied is the map cell (before thresholding)?
           double occ = (load_parameters.negate ? shade : 1.0 - shade);
    """
    w, h = im.size

    # Create 1-pixed borders
    for x in range(0, w):
        im.putpixel((x, 0), OBSTACLE)
        im.putpixel((x, h-1), OBSTACLE)
    for y in range(0, h):
        im.putpixel((0, y), OBSTACLE)
        im.putpixel((w-1, y), OBSTACLE)

    free_thresh *= 255
    occupied_thresh *= 255
    for x in range(1, w-1):
        for y in range(1, h-1):
            pixel_value = im.getpixel((x, y))  # Blacker pixel is "more obstacle" unless negate
            if not negate:
                pixel_value = 255 - pixel_value
            if pixel_value < free_thresh:
                pixel_value = FREE_INI
            elif pixel_value >= occupied_thresh:
                pixel_value = OBSTACLE
            else:
                pixel_value = min(FREE_INI + COST_FACTOR * pixel_value, FREE_END)
            im.putpixel((x, y), round(pixel_value))


def map_free_test(cols, rows, path_ratio=1.0, mapfilename="map.bin", testfilename="test.txt"):
    """ Generates a map file (binary) with num_cols x rows cells surrounded with
        obstacles in the border (1 pixel) and all the cells completely free.
        Generates a random subset of the possible valid paths to test, with ratio "path_ratio".
        Stores the map and all the possible paths in files
    """
    im = Image.new("L", (cols+2, rows+2), OBSTACLE)
    for x in range(1, cols+1):
        for y in range(1, rows + 1):
            print((x,y))
            im.putpixel((x, y), FREE_INI)

    paths = all_paths_image(im, path_ratio)
    store_test(im, testfilename, mapfilename, paths)


def map_free_random_test(cols, rows, path_ratio=1.0, mapfilename="map.bin", testfilename="test.txt"):
    """ Generates a map file (binary) with num_cols x rows cells surrounded with
        obstacles in the border (1 pixel) and the cells free with random values.
        Generates a random subset of the possible valid paths to test, with ratio "path_ratio".
        Stores the map and all the possible paths in files
    """
    im = Image.new("L", (cols + 2, rows + 2), OBSTACLE)
    for x in range(1, cols + 1):
        for y in range(1, rows + 1):
            cell = random.choice(range(FREE_INI, FREE_END+1))
            im.putpixel((x, y), cell)

    paths = all_paths_image(im, path_ratio)
    store_test(im, testfilename, mapfilename, paths)


def cells(cols, rows, ratio):
    """ Selects a random number of cells in the map with a given ratio """
    selected_cells = [(c, r) for c in range(1, cols+1) for r in range(1, rows+1)]
    return random.sample(selected_cells, int(cols*rows*ratio))


def map_obstacle_random_test(cols, rows, path_ratio=1.0, obstacle_ratio=0.2,
                             mapfilename="map.bin", testfilename="test.txt"):
    """ Generates a map file (binary) with num_cols x rows cells surrounded with
        obstacles in the border (1 pixel). Cells are obstacles with 'prob_obstacle' probability and if they
        are not obstacle their value is between 50 and 253 (uniformly).
        Generates a random subset of the possible valid paths to test, with ratio "path_ratio".
        Stores the map and all the possible paths in files
    """
    im = Image.new("L", (cols + 2, rows + 2), OBSTACLE)
    obstacles = cells(cols, rows, obstacle_ratio)
    for x in range(1, cols + 1):
        for y in range(1, rows + 1):
            if (x, y) not in obstacles:
                cell = random.choice(range(FREE_INI, FREE_END+1))
                im.putpixel((x, y), cell)
    paths = all_paths_image(im, path_ratio)
    store_test(im, testfilename, mapfilename, paths)


def map_spiral(cols, rows, path_ratio=1.0, random_cell=False, mapfilename="map.bin", testfilename="test.txt"):
    """ Generates a map file (binary) with num_cols x rows cells surrounded with
        obstacles in the border (1 pixel). Cells forms an clockwise spiral starting from (1, 1).
        Generates a random subset of the possible valid paths to test, with ratio "path_ratio".
        Stores the map and all the possible paths in files
    """
    im = Image.new("L", (cols + 2, rows + 2), FREE_INI)
    
    if random_cell:
        for x in range(1, cols + 1):
            for y in range(1, rows + 1):
                cell = random.choice(range(FREE_INI, FREE_END+1))
                im.putpixel((x, y), cell)
    
    for x in range(0, cols + 2):
        im.putpixel((x, 0), OBSTACLE)
        im.putpixel((x, rows+1), OBSTACLE)
    for y in range(0, rows + 2):
        im.putpixel((0, y), OBSTACLE)
        im.putpixel((cols+1, y), OBSTACLE)

    pos = (1, 1)
    delta = (1, 0)
    while im.getpixel(pos) != OBSTACLE:
        while im.getpixel(next_cell(pos, delta)) != OBSTACLE:
            obstacle_right(im, pos, delta)
            pos = next_cell(pos, delta)

        delta = turn_right(delta)
        pos = next_cell(pos, delta)

    paths = all_paths_image(im, path_ratio)
    store_test(im, testfilename, mapfilename, paths)


def map_radial(cols, rows, path_ratio=1.0, center=(1, 1), radius=25, mapfilename="map.bin", testfilename="test.txt"):
    """ Generates a map file (binary) with num_cols x rows cells surrounded with
        obstacles in the border (1 pixel). Generate a radial gradient from FREE_INI to FREE_END starting in
        center with radius.
        Generates a random subset of the possible valid paths to test, with ratio "path_ratio".
        Stores the map and all the possible paths in files
    """
    im = Image.new("L", (cols + 2, rows + 2), FREE_INI)
    for x in range(0, cols + 2):
        im.putpixel((x, 0), OBSTACLE)
        im.putpixel((x, rows+1), OBSTACLE)
    for y in range(0, rows + 2):
        im.putpixel((0, y), OBSTACLE)
        im.putpixel((cols+1, y), OBSTACLE)

    for x in range(1, cols + 1):
        for y in range(1, rows + 1):
            dist = math.dist(center, (x, y))
            cell_value = OBSTACLE
            if dist < radius:
                cell_value = int((dist/radius) * (FREE_END - FREE_INI) + FREE_INI)
            im.putpixel((x, y), cell_value)

    paths = all_paths_image(im, path_ratio)
    store_test(im, testfilename, mapfilename, paths)


def map_split(cols, rows, num_vert=3, num_horiz=3, path_ratio=1.0, mapfilename="map.bin", testfilename="test.txt"):
    im = Image.new("L", (cols + 2, rows + 2), OBSTACLE)
    for x in range(1, cols + 1):
        for y in range(1, rows + 1):
            cell = random.choice(range(FREE_INI, FREE_END + 1))
            im.putpixel((x, y), cell)

    # Vertical walls
    for _ in range(num_vert):
        x = random.randint(1, cols+1)
        for y in range(1, rows+1):
            im.putpixel((x, y), OBSTACLE)

    # Horizontal walls
    for _ in range(num_horiz):
        y = random.randint(1, rows+1)
        for x in range(1, cols+1):
            im.putpixel((x, y), OBSTACLE)

    paths = all_paths_image(im, path_ratio)
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


def all_paths_image(im, path_ratio=1.0):
    """ Returns a list with paths (start, end) that do not start or end in obstacles.
        Generates a random subset of the valid paths in the map with ratio path_ratio
    """
    w, h = im.size
    all_cells = [(x0, y0) for x0 in range(w) for y0 in range(h)]
    ret = list()
    for start in all_cells:
        for end in all_cells:
            if im.getpixel(start) != OBSTACLE and im.getpixel(end) != OBSTACLE:
                ret.append((start, end))
    num_paths = round(len(ret) * path_ratio)
    if len(ret) < num_paths:
        return ret
    else:
        return random.sample(ret, num_paths)


def random_paths_image(im, path_ratio=1.0):
    """ Returns a list with paths (start, end) that do not start or end in obstacles.
        Generates a random subset of the valid paths in the map with ratio path_ratio wrt. the number of valid cells
        It can generate duplicates
    """
    w, h = im.size
    all_valid_cells = [(x0, y0) for x0 in range(1, w-1) for y0 in range(1, h-1)
                       if FREE_INI <= im.getpixel((x0, y0)) <= FREE_END]
    num_paths = round(len(all_valid_cells) * path_ratio)
    ret = list()
    for _ in range(num_paths):
        start = random.choice(all_valid_cells)
        end = random.choice(all_valid_cells)
        ret.append((start, end))
    return ret


def random_paths_image_close(im, num_paths=10, dist=10):
    """ Returns a list with paths (start, end) that do not start or end in obstacles and are not very far
        Generates a random subset of the valid paths in the map with ratio path_ratio wrt. the number of valid cells
        It can generate duplicates
    """
    w, h = im.size
    all_valid_cells = [(x0, y0) for x0 in range(w) for y0 in range(h) if FREE_INI <= im.getpixel((x0, y0)) <= FREE_END]
    ret = list()
    paths = 0
    while paths < num_paths:
        start = random.choice(all_valid_cells)
        endx = start[0] + random.randint(-dist, dist)
        endy = start[1] + random.randint(-dist, dist)
        if (endx, endy) in all_valid_cells:
            ret.append((start, (endx, endy)))
            paths += 1
    return ret
    
    
def draw_pixel_if_inside(im, x, y, color):
    """ Draws a 'color' in (x,y) if inside the image """
    if x < im.width and y < im.height:
        im.putpixel((x, y), color)
    
    
def draw_column(im, x, y, size, color):
    """ Draws a square column (obstacle) in (x,y) of the image """
    for px in range(x, x+size):
        for py in range(y, y+size):
            draw_pixel_if_inside(im, px, py, color)
    
    
def map_column_test(cols, rows, column_size=1, column_sep=2, path_ratio=1.0,
                    random_cell=False, mapfilename="map.bin", testfilename="test.txt"):
    """ Generates a map file (binary) with num_cols x rows cells surrounded with
        obstacles in the border (1 pixel). It will contain columns of a given
        size and separations in a grid. 
        Generates a random subset of the possible valid paths to test, with ratio "path_ratio".
        Stores the map and all the possible paths in files
    """
    im = Image.new("L", (cols + 2, rows + 2), OBSTACLE)
    for x in range(1, cols+1):
        for y in range(1, rows + 1):
            im.putpixel((x, y), FREE_INI)
    
    if random_cell:
        for x in range(1, cols + 1):
            for y in range(1, rows + 1):
                cell = random.choice(range(FREE_INI, FREE_END+1))
                im.putpixel((x, y), cell)
    
    column_positions = [(x,y) for x in range(column_sep+1, cols+1, column_size+column_sep)
                              for y in range(column_sep+1, rows+1, column_size+column_sep)]
    for (x,y) in column_positions:
        draw_column(im, x, y, column_size, OBSTACLE)

    paths = all_paths_image(im, path_ratio)
    store_test(im, testfilename, mapfilename, paths)
    
    
def random_point(im):
    """ Returns a random point in the image (avoiding borders) """
    x = random.randint(1, im.width-1)
    y = random.randint(1, im.height-1)
    return (x,y)
    
    
def draw_wall(im, p1, p2, color): 
    # Draws a linear wall in the image from p1 to p2
    STEP = 0.5
    
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    norm = math.sqrt(dx**2 + dy**2)
    dx /= norm
    dy /= norm
    
    cx = p1[0]
    cy = p1[1]
    for _ in range(int(norm/STEP)+1):
        cx = cx + dx*STEP
        cy = cy + dy*STEP
        draw_pixel_if_inside(im, int(cx), int(cy), color)
    
    
    
    
def map_walls_test(cols, rows, num_walls=1, wall_size=1, path_ratio=1.0,
                   random_cell=False, mapfilename="map.bin", testfilename="test.txt"):
    """ Generates a map file (binary) with num_cols x rows cells surrounded with
        obstacles in the border (1 pixel). It contains random walls between two
        points of the map. 
        Generates a random subset of the possible valid paths to test, with ratio "path_ratio".
        Stores the map and all the possible paths in files
    """
    im = Image.new("L", (cols + 2, rows + 2), OBSTACLE)
    for x in range(1, cols+1):
        for y in range(1, rows + 1):
            im.putpixel((x, y), FREE_INI)
    
    if random_cell:
        for x in range(1, cols + 1):
            for y in range(1, rows + 1):
                cell = random.choice(range(FREE_INI, FREE_END+1))
                im.putpixel((x, y), cell)
    
    for _ in range(num_walls):
        p1 = random_point(im)
        p2 = random_point(im)
        draw_wall(im, p1, p2, OBSTACLE)

    paths = all_paths_image(im, path_ratio)
    store_test(im, testfilename, mapfilename, paths)    
    

def main():
    map_obstacle_random_test(10, 10, path_ratio=0.2, obstacle_ratio=0.0,
                             mapfilename="map.bin", testfilename="test.txt")



if __name__ == "__main__":
    main()
