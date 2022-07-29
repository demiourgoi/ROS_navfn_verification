#
# Programmed debugging of ROS NavFn planner (using the LLVM API)
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


def eval2float(frame, expr):
	"""Evaluate a expression in the given frame as a float"""
	# Or alternatively .GetData().GetFloat(lldb.SBError(), 0)
	return float(frame.EvaluateExpression(expr).GetValue())

def eval2bool(frame, expr):
	"""Evaluate a expression in the given frame as a float"""
	return frame.EvaluateExpression(expr).GetValueAsSigned() != 0


class ROSTracer:
	"""Trace the intermediate steps of ROS NavFn for debugging"""

	def __init__(self, debugger):
		self.target = debugger.CreateTarget('profile_cpp_bin_dbg')
		self.process = None

		# Breakpoints
		self.bp_handlers = {}

		self.register_break(551, self.position_bp)
		self.register_break(672, self.iteration_bp)
		self.register_break(731, self.conclusion_bp)
		self.register_break(733, self.bad_conclusion_bp)
		self.register_break(783, self.path_bp)
		self.register_break(835, self.path_jump_bp)

	def register_break(self, line, handler):
		"""Register a callback for a breakpoint"""
		bp = self.target.BreakpointCreateByLocation('navfn.cpp', line)
		self.bp_handlers[bp.GetID()] = handler

	def handle_break(self, bp_id, *args):
		"""Handle a registered breakpoint"""
		self.bp_handlers[bp_id](*args)

	def position_bp(self, frame):
		"""Breakpoint to log the argument of NavFn::updateCellAstar"""

		n = eval2int(frame, 'n')	# Position
		nx = eval2int(frame, 'nx')	# Number of columns
		ns = eval2int(frame, 'ns')	# Number of cells (map size)

		pot = eval2float(frame, 'pot')
		dist = eval2float(frame, 'dist')

		is_below = eval2bool(frame, 'pot < curT')

		print(split_xy(n, nx), 'is assigned potential', pot - dist, 'at distance', dist,
		      '(below threshold)' if is_below else '(over threshold)')

		# Check neighbors in every possible direction
		for d, v in zip('lrud', (-1, 1, -nx, nx)):
			m = n + v

			# Mimics the conditions in updateCellAstar and push_*
			if eval2bool(frame, f'{d} > pot + {d}e') and 0 <= m < ns and eval2bool(frame, f'costarr[{m}] < 254'):
				if eval2bool(frame, f'pending[{m}]'):
					print('\tNot pushing', split_xy(m, nx), 'because of pending.')

				else:
					if eval2bool(frame, f'{d} < 1e+10'):
						extra = 'whose potential is {d}'
					else:
						extra = ''

					print('\tPushing', split_xy(m, nx), extra)

	def iteration_bp(self, frame):
		"""Breakpoint to log iterations and its queues in NavFn::propNavFnAstar"""

		cycle = eval2int(frame, 'cycle')
		curT = eval2float(frame, 'curT')
		curPe = eval2int(frame, 'curPe')
		overPe = eval2int(frame, 'overPe')
		nx = eval2int(frame, 'nx')

		print('\n--- Iteration', cycle, 'with threshold', curT)
		print('--- Current:', [split_xy(eval2int(frame, f'curP[{i}]'), nx) for i in range(curPe)])
		print('--- Over:', [split_xy(eval2int(frame, f'overP[{i}]'), nx) for i in range(overPe)])

	def conclusion_bp(self, frame):
		"""Breakpoint when the algorithm concludes"""

		print('\n--- The navigation function has been completed succesfully\n', flush=True)

	def bad_conclusion_bp(self, frame):
		"""Breakpoint when the algorithm concludes badly"""

		cycle = eval2int(frame, 'cycle')
		cycles = eval2int(frame, 'cycles')
		print(f'\n--- Finished without completing the navigation function in', cycle, 'cycles of', cycles, flush=True)

	def path_bp(self, frame):
		"""Breakpoint to log steps in the path calculation"""

		index = eval2int(frame, 'i')
		stc = eval2int(frame, 'stc')
		nx = eval2int(frame, 'nx')
		dx = eval2float(frame, 'dx')
		dy = eval2float(frame, 'dy')

		print('--- Path step', index, 'at', split_xy(stc, nx), '+', (dx, dy), flush=True)

	def path_jump_bp(self, frame):
		"""Breakpoint to explain path jumps"""

		if eval2bool(frame, 'oscillation_detected'):
			print('\tOscillation detected')

		else:
			print('\tInteger jump')

	def run(self, test):
		"""Run the target with a given test file"""
		self.process = self.target.LaunchSimple([test], None, os.getcwd())

		# Repeat the process
		while self.process.GetState() == lldb.eStateStopped:
			thread = self.process.GetThreadAtIndex(0)
			frame = thread.GetFrameAtIndex(0)

			if thread.GetStopReason() != lldb.eStopReasonBreakpoint:
				print('Warning: the debugger stopped because of an unexpected cause.', file=sys.stderr)

			self.handle_break(thread.GetStopReasonDataAtIndex(0), frame)
			self.process.Continue()


def main():
	import argparse

	parser = argparse.ArgumentParser(description='Trace the intermediate steps of the ROS NavFn execution')
	parser.add_argument('test', help='Test file to load')

	args = parser.parse_args()

	# Add directory with the ROS libraries to the library path
	if 'LD_LIBRARY_PATH' not in os.environ:
		os.environ['LD_LIBRARY_PATH'] = 'ros-libs'

	# Debugger instance
	debugger = lldb.SBDebugger.Create()

	# Make SBProcess.Continue synchronous
	debugger.SetAsync(False)

	tracer = ROSTracer(debugger)
	tracer.run(args.test)

	return 0


if __name__ == '__main__':
	sys.exit(main())
