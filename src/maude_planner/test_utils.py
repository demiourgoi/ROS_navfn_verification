#
# Some useful functions for all testing procedures
#

import collections
import os

# Test file information data type
TestFile = collections.namedtuple('TestFile',
                                  ('w', 'h', 'map_data', 'test_cases', 'map_bin_file'),
                                  defaults=(None, ))


def load_test(test_path, ctype=float):
	"""Load a test.txt file"""

	with open(test_path, 'r') as ftest:
		lines = ftest.readlines()

		# First line: width and height of the map
		w, h = map(int, lines[0].strip().split())

		# Second line: filename of the map, relative to the test case
		map_bin_file = lines[1].strip()
		test_full_path = os.path.abspath(ftest.name)
		test_dir = os.path.dirname(test_full_path)
		map_full_map_path = os.path.join(test_dir, map_bin_file)
		map_bin_file = map_full_map_path  # Full path

		# Read the map from the binary file
		map_data = [0] * (w * h)  # Avoid appending

		with open(map_bin_file, 'rb') as fmap:
			for i in range(w * h):
				cell = fmap.read(1)
				map_data[i] = int.from_bytes(cell, 'big')  # It's one byte, it doesn't matter big or little endian

		# The third line is empty

		# The next lines contain the test cases (start and goal coordinates)
		test_cases = list()
		for test in lines[3:]:
			if test.strip().startswith('-1'):
				continue

			x0, y0, x1, y1 = (ctype(c) for c in test.strip().split())
			test_cases.append(((x0, y0, 0.0), (x1, y1, 0.0)))  # Orientation 0 degrees

		return TestFile(w, h, map_data, test_cases, map_bin_file)


