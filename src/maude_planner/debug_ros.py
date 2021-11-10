#
# Programmed debugging of ROS NavFn planner (using the GDB API)
#

import gdb
import sys

# Stream where traces will be written
rlog = open('ros_log.txt', 'w')


def split_xy(n, nx):
	"""Split a linear index to X and Y coordinates"""

	return (n % nx, n // nx)


class PositionBreak(gdb.Breakpoint):
	"""Breakpoint to log the argument of NavFn::updateCallAstar"""

	def __init__(self):
		super().__init__('navfn.cpp:551')

	def stop(self):
		n = int(gdb.parse_and_eval('n'))	# Position
		nx = int(gdb.parse_and_eval('nx'))	# Number of columns
		ns = int(gdb.parse_and_eval('ns'))	# Number of cells (map size)

		pot = float(gdb.parse_and_eval('pot'))
		dist = float(gdb.parse_and_eval('dist'))

		is_below = bool(gdb.parse_and_eval('pot < curT'))

		print(split_xy(n, nx), 'is assigned potential', pot - dist, 'at distance', dist,
		      '(below threshold)' if is_below else '(over threshold)', file=rlog)

		# Check neighbors in every possible direction
		for d, v in zip('lrud', (-1, 1, -nx, nx)):
			m = n + v

			# Mimics the conditions in updateCellAstar and push_*
			if gdb.parse_and_eval(f'{d} > pot + {d}e') and 0 <= m < ns and gdb.parse_and_eval(f'costarr[{m}] < 254'):
				if gdb.parse_and_eval(f'pending[{m}]'):
					print('\tNot pushing', split_xy(m, nx), 'because of pending.', file=rlog)

				else:
					if gdb.parse_and_eval(f'{d} < 1e+10'):
						extra = 'whose potential is {d}'
					else:
						extra = ''

					print('\tPushing', split_xy(m, nx), extra, file=rlog)

		return False


class IterationBreak(gdb.Breakpoint):
	"""Breakpoint to log iterations and its queues in NavFn::propNavFnAstar"""

	def __init__(self):
		super().__init__('navfn.cpp:672')

	def stop(self):
		cycle = gdb.parse_and_eval('cycle')
		curT = gdb.parse_and_eval('curT')
		curPe = int(gdb.parse_and_eval('curPe'))
		overPe = int(gdb.parse_and_eval('overPe'))
		nx = int(gdb.parse_and_eval('nx'))

		print('\n--- Iteration', cycle, 'with threshold', curT, file=rlog)
		print('--- Current:', [split_xy(int(gdb.parse_and_eval(f'curP[{i}]')), nx) for i in range(curPe)], file=rlog)
		print('--- Over:', [split_xy(int(gdb.parse_and_eval(f'overP[{i}]')), nx) for i in range(overPe)], file=rlog)

		return False


class ConclusionBreak(gdb.Breakpoint):
	"""Breakpoint when the algorithm concludes"""

	def __init__(self):
		super().__init__('navfn.cpp:731')

	def stop(self):
		print('\n--- The destination has been found', file=rlog)
		rlog.flush()

		return False


class BadConclusionBreak(gdb.Breakpoint):
	"""Breakpoint when the algorithm concludes badly"""

	def __init__(self):
		super().__init__('navfn.cpp:733')

	def stop(self):
		cycle = gdb.parse_and_eval('cycle')
		cycles = gdb.parse_and_eval('cycles')
		print(f'\n--- Finished without finding a path in', cycle, 'cycles of', cycles, file=rlog)
		rlog.flush()

		return False


x = PositionBreak()
y = IterationBreak()
z = ConclusionBreak()
t = BadConclusionBreak()
