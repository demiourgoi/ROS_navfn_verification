#!/usr/bin/env python3
#
# White-box testing of ROS NavFn planner (using the LLDB APIs)
#
# Currently, we check the A* algorithms in ROS and Dafny execute the same
# number of iterations and that their stacks coincide in each of them.
#

import os
import sys

import lldb

def split_xy(n, nx):
	"""Split a linear index to X and Y coordinates"""
	return (n % nx, n // nx)

def eval2int(frame, expr):
	"""Evaluate a expression in the given frame as an integer"""
	return frame.EvaluateExpression(expr).GetValueAsSigned()


class ROSTarget:
	"""ROS implementation target"""

	def __init__(self, debugger):
		self.target = debugger.CreateTarget('profile_cpp_bin_dbg')
		self.process = None

		# Breakpoints
		self.iter_b = self.target.BreakpointCreateByLocation('navfn.cpp', 672)
		self.case_b = self.target.BreakpointCreateByLocation('profile_cpp.cc', 94)

	def launch(self, test):
		"""Launch the target with a given test file"""
		self.process = self.target.LaunchSimple([test], None, os.getcwd())

	def get_queues(self, frame):
		"""Get the queues from the ROS iteration state"""

		cycle = eval2int(frame, 'cycle')	# Iteration number
		nx = eval2int(frame, 'nx')		# Number of columns
		curT = eval2int(frame, 'curT')		# Threshold
		curPe = eval2int(frame, 'curPe')	# Current stack size
		overPe = eval2int(frame, 'overPe')	# Over stack size
		nextPe = eval2int(frame, 'nextPe')	# Number of columns

		cur_st = [split_xy(eval2int(frame, f'curP[{i}]'), nx) for i in range(curPe)]
		over_st = [split_xy(eval2int(frame, f'overP[{i}]'), nx) for i in range(overPe)]
		next_st = [split_xy(eval2int(frame, f'nextP[{i}]'), nx) for i in range(nextPe)]

		return cur_st, next_st, over_st


class DafnyTarget:
	"""Dafny implementation target"""

	def __init__(self, debugger):
		self.target = debugger.CreateTarget('astar_navfnplanner')
		self.process = None

		# Breakpoints
		self.case_b = self.target.BreakpointCreateByLocation('astar_navfnplanner.go', 619)
		self.tuple_b = self.target.BreakpointCreateByName('notify_tuple', 'astar_navfnplanner')
		self.endq_b = self.target.BreakpointCreateByName('end_queue', 'astar_navfnplanner')

	def launch(self, test):
		"""Launch the target with a given test file"""
		self.process = self.target.LaunchSimple([test], None, os.getcwd())

	def get_queues(self, new_case):
		"""Get the queues from the Dafny iteration state"""

		queues = ([], [], [])
		index = 0

		# Indices of the breakpoint that we may reach here
		endq_id = self.endq_b.GetID()
		case_id = self.case_b.GetID()

		while index < 3 and self.process.GetState() == lldb.eStateStopped:
			# Thread running the Dafny implementation
			thread = self.process.GetSelectedThread()

			if thread.GetStopReason() == lldb.eStopReasonBreakpoint:
				bp_id = thread.GetStopReasonDataAtIndex(0)

				# end_queue signals the end of a queue
				if bp_id == endq_id:
					index += 1

				# The AStar breakpoint indicates that a process has started
				elif bp_id == case_id:
					if new_case:
						new_case = False
					else:
						print('Error: test case ended prematurely in Dafny.')
						return (None,) * 3

				else:
					if new_case:
						print('Error: unexpected additional iteration in Dafny.')
						return (None,) * 3

					frame = thread.GetFrameAtIndex(0)
					queues[index].append((eval2int(frame, 'a'), eval2int(frame, 'b')))
			else:
				print('Warning: Dafny target stopped due to an unexpected cause.')

			self.process.Continue()

		# If we have not read the 3 stacks, something wrong should have happened
		if index < 3:
			print('Error: Dafny process finished before expected.')
			return (None,) * 3

		return queues


def bisimulate(ros, dafny):
	"""Run a test in both ROS and Dafny"""

	# Current test case data
	case, cycle, new_case = None, None, False
	# ID of the case breakpoint in ROS
	case_bid = ros.case_b.GetID()

	while ros.process.GetState() == lldb.eStateStopped:
		thread = ros.process.GetThreadAtIndex(0)
		frame = thread.GetFrameAtIndex(0)

		if thread.GetStopReason() != lldb.eStopReasonBreakpoint:
			print('Warning: ROS target stopped due to an unexpected cause.')

		elif thread.GetStopReasonDataAtIndex(0) == case_bid:
			case = tuple(int(eval2int(frame, f'tcase[{i}]')) for i in range(4))
			cycle = eval2int(frame, 'cycle')
			new_case = True

		else:
			cur_st, _, over_st = ros.get_queues(frame)
			cur_df, _, over_df = dafny.get_queues(new_case)

			if cur_df is None:
				break

			if cur_st != cur_df:
				print('\x1b[1;31mCURR\x1b[0m', case, cycle, cur_st, cur_df)
			if over_st != over_df:
				print('\x1b[1;31mOVER\x1b[0m', case, cycle, over_st, over_df)

			new_case = False

		ros.process.Continue()

	print(f'Finished (return codes: ROS={ros.process.GetExitStatus()}, Dafny={dafny.process.GetExitStatus()}).')

	# Kill processes that have not finished yet
	for impl in (dafny, ros):
		if impl.process.GetExitStatus() < 0:
			impl.process.Kill()


def main():
	import argparse

	parser = argparse.ArgumentParser(description='White-box differential testing for ROS and Dafny')
	parser.add_argument('tests', help='Tests to be processed', nargs='+')

	args = parser.parse_args()

	# Add directory with the ROS libraries to the library path
	if 'LD_LIBRARY_PATH' not in os.environ:
		os.environ['LD_LIBRARY_PATH'] = 'ros-libs'

	# Prevent the Go runtime to raise SIGURG signals that disrupt the debugger
	if 'GODEBUG' not in os.environ:
		os.environ['GODEBUG'] = 'asyncpreemptoff=1'

	# Debugger instance
	debugger = lldb.SBDebugger.Create()

	# Make SBProcess.Continue synchronous
	debugger.SetAsync(False)

	# Targets for ROS and Dafny
	ros = ROSTarget(debugger)
	dafny = DafnyTarget(debugger)

	# Run the tests
	multiple = len(args.tests) > 1

	for k, test in enumerate(args.tests):
		if multiple:
			print(f'{k + 1}) W-b-testing {test}')

		dafny.launch(test)
		ros.launch(test)

		bisimulate(ros, dafny)


if __name__ == '__main__':
	sys.exit(main())
