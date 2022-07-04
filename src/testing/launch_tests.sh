#!/usr/bin/env python3
#
# Check tests in the tests/ directory, showing a final summary of erorrs
#

import argparse
import os
import shutil
import subprocess
import sys


class ROSImpl:
	"""ROS' NavFn implementation"""

	ID = 'ros'
	NAME = 'ROS'
	PATH = './profile_cpp'

	def run(self, test, outfile, navfn=True):
		subprocess.run([self.PATH] + ([] if navfn else ['-n']) + [test],
			       stdout=outfile)


class MaudeImpl:
	"""Maude's NavFn implementation"""

	ID = 'maude'
	NAME = 'Maude'
	PATH = 'profile_maude.py'

	def run(self, test, outfile, navfn=True):
		subprocess.run([sys.executable, self.PATH] + ([] if navfn else ['--no-navfn']) + [test],
		               stdout=outfile)


class DafnyImpl:
	"""Dafny's NavFn implementation"""

	ID = 'dafny'
	NAME = 'Dafny'
	PATH = './astar_navfnplanner'

	def run(self, test, outfile, navfn=True):
		subprocess.run([self.PATH] + ([] if navfn else ['--no-navfn']) + [test],
		               stdout=outfile)


class TestRunner:
	"""Run tests"""

	def __init__(self, ros_impl, impls, only_missing):
		# ROS' NavFn implementation
		self.ros_impl = ros_impl
		# Other implementation
		self.impls = impls
		# Whether to run only missing test executions
		self.only_missing = only_missing

		# Test number in the sequence
		self.counter = 0
		# Path to the tests producing errors
		self.errors = [[] for _ in self.impls]

	def run_test(self, test):
		"""Run a single test"""

		# The width of the map
		with open(test) as tfile:
			header = next(tfile).split()
			mapWidth = int(header[0])

		# Navigation functions are not included in the log files if the
		# map width in greater than 100
		navfn = mapWidth < 100

		dirname = os.path.dirname(test)
		test_name = os.path.basename(dirname)

		ros_out = os.path.join(dirname, 'ros.txt')
		impls_out = [os.path.join(dirname, f'{impl.ID}.txt') for impl in self.impls]

		# Run the tests
		been_run = [False] * len(self.impls)

		if self.ros_impl and (not self.only_missing or not os.path.exists(ros_out)):
			with open(ros_out, 'w') as rfile:
				self.ros_impl.run(test, rfile, navfn)

		for k, impl in enumerate(self.impls):
			if not self.only_missing or not os.path.exists(impls_out[k]):
				been_run[k] = True
				with open(impls_out[k], 'w') as ifile:
					impl.run(test, ifile, navfn)

		# Compare the results of the tests
		if not os.path.exists(ros_out):
			print('Warning: the ROS output is missing. Nothing to compare.')
			return

		for k, impl in enumerate(self.impls):
			if not been_run[k]:
				continue

			result = subprocess.run([sys.executable, 'compare.py', f'--width={mapWidth}',
			                         f'--name={impl.NAME}', ros_out, impls_out[k]])

			if result.returncode == 0:
				print(f'... OK with {impl.NAME}')
			else:
				self.errors[k].append(test)
				print(f'... ERROR in {impl.NAME}')
				print('Generating potentials...')

				subprocess.run([sys.executable, 'compare.py', f'--width={mapWidth}',
					        f'--name={impl.NAME}', ros_out, impls_out[k], '--draw=failed'],
					        stdout=subprocess.DEVNULL)

				if os.path.exists('potentials.pdf'):
					shutil.move('potentials.pdf',
					            os.path.join(dirname, f'{test_name}_{impl.ID}.pdf'))

	def run_tests(self, tests):
		"""Run all the test given as argument"""

		for k, test in enumerate(tests):
			print(f'{k + 1}) Testing {test}')
			self.run_test(test)

		# Show a summary of the results
		if len(tests) > 1:
			for k, impl in enumerate(self.impls):
				print(f'\nTest that failed with {impl.NAME}:')
				for test in self.errors[k]:
					print('*', test)

				print(f'Failed {len(self.errors[k])}/{len(tests)}')


def remove_dotdash(path):
	"""Remove the initial ./ in a filename"""
	return path[2:] if path.startswith('./') else path


def find_tests(dirname='.'):
	"""Find all tests files in a given directory recursively"""

	return [os.path.join(remove_dotdash(root), 'test.txt')
	        for root, _, filenames in os.walk(dirname)
	        if 'test.txt' in filenames]


def collect_tests(args):
	"""Collect test from a list of tests and directories"""

	if not args:
		return find_tests()

	tests = []

	for arg in args:
		if os.path.isdir(arg):
			tests += find_tests(arg)

		elif os.path.exists(arg):
			tests.append(arg)

		else:
			print(f'Warning: file "{arg}" does not exist and will be ignored.')

	return tests


def collect_impls(args):
	"""Collect implementations"""

	# Original ROS implementation
	ros_impl = None
	impls = []

	if not os.path.exists(ROSImpl.PATH):
		print(f'Warning: the ROS NavFn implementation ({ROSImpl.PATH})'
		      ' cannot be found and it will not be executed.')
	else:
		ros_impl = ROSImpl()

	# Other implementations
	others = ((args.maude, MaudeImpl), (args.dafny, DafnyImpl))

	for required, cls in others:
		if required and not os.path.exists(cls.PATH):
			print(f'Warning: the {cls.NAME} NavFn implementation ({cls.PATH})'
			      ' cannot be found and it will not be executed.')

		elif required:
			impls.append(cls())

	return ros_impl, impls

def main():
	import argparse

	parser = argparse.ArgumentParser(description='Check tests in the test directory, showing a final summary of errors')

	parser.add_argument('tests', help='Test files or directories to be processed', nargs='*')
	parser.add_argument('--no-maude', help='Do not test the Maude implementation', dest='maude', action='store_false')
	parser.add_argument('--no-dafny', help='Do not test the Dafny implementation', dest='dafny', action='store_false')
	parser.add_argument('--only-missing', '-m', help='Only generate and compare missing test executions', action='store_true')

	args = parser.parse_args()

	# Make the generated PDF reproducible by omitting the creation date
	# (the Matplotlib version may also be omitted)
	# (https://matplotlib.org/2.1.1/users/whats_new.html#reproducible-ps-pdf-and-svg-output)
	os.environ['SOURCE_DATE_EPOCH'] = '0'

	# Collect implementations
	ros_impl, impls = collect_impls(args)

	runner = TestRunner(ros_impl, impls, args.only_missing)
	runner.run_tests(collect_tests(args.tests))


if __name__ == '__main__':
	sys.exit(main())
