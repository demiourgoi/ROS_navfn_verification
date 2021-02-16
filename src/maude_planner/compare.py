# -*- coding: utf-8 -*-

"""
Check navigation results from ROS2 and Maude implementation
Author: Enrique Martin
"""

import json
import sys


def file_to_dict(filename):
    """Loads a file with JSON lines into a dictinary (initial, final) -> path"""
    results = dict()
    with open(filename, 'r') as json_file:
        lines = json_file.readlines()
        for line in lines:
            line_dict = json.loads(line)
            # Line has the following format:
            # {"initial": [1, 1], "goal": [1, 1], "length": 0, "path": [[1, 1], [1, 1]]}
            initial = tuple(line_dict["initial"])
            goal = tuple(line_dict["goal"])
            path = [tuple(pose) for pose in line_dict["path"]]
            results[(initial, goal)] = path
    return results
    

def remove_last(path):
    """Removes the last pose if duplicated"""
    if len(path) <= 1 or path[-1] != path[-2]:
        return path
    else:
        return path[:-1]
            
    
def path_equal(path1, path2):
    """Checks if path1 and path2 are equal"""
    # Remove duplicates in the goal position
    path1 = remove_last(path1)
    path2 = remove_last(path2)
    
    if len(path1) != len(path2):
        return False
    pos = 0
    while pos < len(path1) and path1[pos] == path2[pos]:
      pos += 1 
    return pos == len(path1)
    
  
  
def main(file1, file2):
    """Compares the results stored in file1 and file2, showing the result in the standard output.
       I assume file1 are the results from ROS and file2 those from Maude   
    """
    dict1 = file_to_dict(file1)
    dict2 = file_to_dict(file2)
    key_set1 = set(dict1.keys())
    key_set2 = set(dict1.keys())
    
    tests = sorted(list(key_set1))
    if key_set1 != key_set2:
        print(f"{file1} - {file2}: {key_set1 - key_set2}")
        print(f"{file2} - {file1}: {key_set2 - key_set1}")
        tests = sorted(list(key_set1.intersect(key_set2)))
        
    num_diff = 0
    for test in tests:
        if not path_equal(dict1[test], dict2[test]):
            num_diff += 1
            print(f'{num_diff}) Differences in the path from {test[0]} to {test[1]}')
            print(f'ROS  : {dict1[test]}')
            print(f'Maude: {dict2[test]}\n')
        
            
if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
