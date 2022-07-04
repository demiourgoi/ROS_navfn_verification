#
# White-box testing of ROS NavFn planner (using the GDB and Maude APIs)
#
# Currently, we check the A* algorithm in ROS and Maude execute the same
# number of iterations and that their stacks coincide in each of them.
#

import os
import sys
from itertools import islice

import gdb
import maude


# Stream where traces will be written
rlog = open('ros_log.txt', 'a')


def split_xy(n, nx):
	"""Split a linear index to X and Y coordinates"""

	return (n % nx, n // nx)


class RunTestBreak(gdb.Breakpoint):
	"""Breakpoint to capture test case data"""

	def __init__(self, engine):
		super().__init__('profile_cpp.cc:94')
		self.engine = engine

	def stop(self):
		cmap = gdb.parse_and_eval('map')
		tcase = [int(gdb.parse_and_eval(f'tcase[{i}]')) for i in range(4)]

		if self.engine.map is None:
			width = int(gdb.parse_and_eval('map.width'))
			height = int(gdb.parse_and_eval('map.height'))

			self.engine.begin_map(width, [int(gdb.parse_and_eval(f'map.data[{i}]')) for i in range(width * height)])

		self.engine.begin_case(tcase)


class CalcPathBreak(gdb.Breakpoint):
	"""Breakpoint to capture the internal positions of the path calculation"""

	def __init__(self):
		super().__init__('navfn.cpp:783')

	def stop(self):
		stc = int(gdb.parse_and_eval('stc'))	# Position
		dx = float(gdb.parse_and_eval('dx'))	# Fractional offset
		dy = float(gdb.parse_and_eval('dy'))

		print(f'\x1b[1;32m{stc:4} {dx:10} {dy:10}\x1b[0m')

		return False


class AStarBreak(gdb.Breakpoint):
	"""Breakpoint to follow the iterations of the A* algorithm"""

	def __init__(self, engine):
		super().__init__('navfn.cpp:672')
		self.engine = engine

	def stop(self):
		cycle = int(gdb.parse_and_eval('cycle'))	# Iteration number
		curT = gdb.parse_and_eval('curT')		# Threshold
		curPe = int(gdb.parse_and_eval('curPe'))	# Current stack size
		overPe = int(gdb.parse_and_eval('overPe'))	# Over stack size
		nextPe = int(gdb.parse_and_eval('nextPe'))	# Next stack size
		nx = int(gdb.parse_and_eval('nx'))		# Number of columns

		cur_st = [split_xy(int(gdb.parse_and_eval(f'curP[{i}]')), nx) for i in range(curPe)]
		over_st = [split_xy(int(gdb.parse_and_eval(f'overP[{i}]')), nx) for i in range(overPe)]
		next_st = [split_xy(int(gdb.parse_and_eval(f'nextP[{i}]')), nx) for i in range(nextPe)]

		self.engine.check_iteration(cycle, cur_st, next_st, over_st)

		return False


class TestEngine:
	"""White-box testing for ROS and Maude"""

	ASTAR_MAUDE_PATH = '../maude/astar_no_turnNavFnPlanner_rew.maude'

	def __init__(self, use_hook=True):
		# self.calcpath_b = CalcPathBreak()
		self.astar_b = AStarBreak(self)
		self.runtest_b = RunTestBreak(self)

		self.width = None
		self.height = None
		self.map = None
		self.term = None
		self.use_hook = use_hook

		self.filename = os.getenv('TEST_PATH')
		m = maude.getModule('ASTAR')
		self.module = m

		pose_kind = m.findSort('Pose').kind()
		cmap_kind = m.findSort('CostMap').kind()
		nat_kind = m.findSort('Nat').kind()
		path_kind = m.findSort('Path').kind()
		potential_kind = m.findSort('Potential').kind()

		# Symbol of the a* algorithm
		self.astar = m.findSymbol('a*i', (pose_kind, pose_kind, cmap_kind,
		                                  nat_kind, nat_kind, nat_kind), potential_kind)

		# Kinds for parsing in other methods
		self.nat_kind = nat_kind
		self.pose_kind = pose_kind

		# Symbols for constructing the map
		self.intlist = m.findSymbol('_,_', (nat_kind, nat_kind), nat_kind)
		self.mtIL = m.parseTerm('mtIL', nat_kind)
		self.cmap_symb = m.findSymbol('{_}', (nat_kind, ), cmap_kind)
		self.noPath = m.parseTerm('noPath', path_kind)
		self.path_union = m.findSymbol('__', (path_kind, path_kind), path_kind)

		# Strategy to advance an iteration
		self.iter_st = m.parseStrategy('curr ! ; next')

		self.already_missed_stack = False

		# Hook for "op get : CostMap Nat Nat Nat -> Float"
		class MapHook(maude.Hook):
			def __init__(self, parent):
				super().__init__()
				self.parent = parent
				self.cache = dict()  # Dictionary int in [0..255] -> Maude term representing the corresponding Float
				for i in range(0, 256):
					self.cache[i] = self.parent.module.parseTerm(str(float(i)))

			def run(self, term, data):
				try:
					_, x, y, ncols = [int(arg) for arg in term.arguments()]
					cell_value = self.parent.cmap[y * ncols + x]
					ret_term = self.cache[cell_value]
					# print(f'FAST {term} --> {ret_term}')
					return ret_term
				except Exception as e:
					print('hook:', e)

		if use_hook:
			self.mapHook = MapHook(self)
			maude.connectEqHook('get', self.mapHook)

	def begin_map(self, width, cmap):
		"""Communicate the map that NavFn has started with"""

		height = len(cmap) // width

		self.width = self.module.parseTerm(str(width), self.nat_kind)
		self.height = self.module.parseTerm(str(height), self.nat_kind)

		if self.use_hook:
			self.cmap = cmap
			self.map = self.cmap_symb(self.mtIL)
		else:
			self.map = self.cmap_symb(self.intlist(*map(self.module.parseTerm, map(str, cmap))))

		# Number of iterations
		cycles = str(max(len(cmap) // 20, width + height))
		self.iterations = self.module.parseTerm(cycles, self.nat_kind)

	def begin_case(self, tcase):
		"""Communicate the test case that NavFn has started with"""

		self.case = tcase

		origin = self.make_bpose(tcase[0], tcase[1])
		dest = self.make_bpose(tcase[2], tcase[3])

		# op a*i : Pose Pose CostMap Nat Nat Nat ~> Potential .
		self.term = self.astar(origin, dest, self.map,
		                       self.width, self.height,
		                       self.iterations)

		# print(self.term)
		self.term.reduce()

		self.already_missed_stack = False

	def check_iteration(self, cycle, *stacks):
		"""Notify an iteration executed by NavFn"""

		# print(f'{cycle:3} {" ".join(map(str, stacks))}', file=sys.stderr)

		if self.term is None:
			self.log_missing(cycle, stacks)
		else:
			stacks_maude = list(self.extract_stacks(self.term))

			for k, name in enumerate(('CURR', 'NEXT', 'OVER')):
				if stacks[k] != stacks_maude[k]:
					self.log_diff(name, cycle, stacks[k], stacks_maude[k],
					              set(stacks[k]) == set(stacks_maude[k]))

			self.term, _ = next(self.term.srewrite(self.iter_st), (None, None))

	def make_bpose(self, x, y):
		"""Construct a BasePose term"""

		return self.module.parseTerm(f'{{{x}, {y}}}', self.pose_kind)

	def extract_stacks(self, term):
		"""Extract the stacks from an a* term"""

		return map(self.extract_stack, islice(term.arguments(), 6, 9))

	def extract_stack(self, stack):
		"""Extract a Maude stack as a list of tuples"""

		if stack == self.noPath:
			stack = []
		elif stack.symbol() == self.path_union:
			stack = list(stack.arguments())
		else:
			stack = [stack]

		return [tuple(map(int, t.arguments())) for t in stack]

	def log_diff(self, name, cycle, left, right, order):
		diff_type = '\x1b[1;41mDifference' if not order else '\x1b[1;31mOrder difference'
		print(f'{diff_type} in {name} ({cycle}):\x1b[0m {left} vs. {right}', file=sys.stderr)
		print(f'{self.filename}:{",".join(map(str, self.case))}:{cycle}:{name}: {left} {"set(!=)" if order else "!="} {right}', file=rlog)
		rlog.flush()

	def log_missing(self, cycle, *stacks):
		if not self.already_missed_stack:
			print(f'\x1b[1;31mMissing stack when expected (in cycle {cycle}).\x1b[0m')
			print(f'{self.filename}:{",".join(map(str, self.case))}:{cycle}: missing stack', file=rlog)
			rlog.flush()

			self.already_missed_stack = True

maude.init()
maude.load(TestEngine.ASTAR_MAUDE_PATH)

x = TestEngine()

gdb.execute('tty /dev/null')
gdb.execute('run')
gdb.execute('quit')
