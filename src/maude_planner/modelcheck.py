#
# Model check the Maude implementation against the test suite
#


import time
import maude

from test_utils import load_test


def format_duration(duration, width=4):
	"""Pretty print a duration (given in ns)"""

	UNITS = (('ns', 1e3), ('μs', 1e3), ('ms', 1e3), ('s', 60), ('min', 60))

	for unit, div in UNITS:
		if duration < div:
			return f'{round(duration, width)} {unit}'

		duration /= div

	return f'{duration} h'


def simplify_test_name(name):
	"""Simplify test name if it is a standard one"""

	if name.startswith('tests/') and name.endswith('/test.txt'):
		return name[6:-9]

	return name


class MapHook(maude.Hook):
	"""Hook for op get : CostMap Nat Nat Nat -> Float"""

	def __init__(self):
		super().__init__()
		self.parent = {}
		self.cache = {}  # Dictionary int in [0..255] -> Maude term representing the corresponding Float


	def add_user(self, key, parent):
		self.parent[key] = parent
		self.cache[key] = {i: parent.module.parseTerm(str(float(i))) for i in range(256)}

	def run(self, term, data):
		try:
			key = str(term.symbol().getModule())
			_, x, y, ncols = [int(arg) for arg in term.arguments()]
			cell_value = self.parent[key].cmap[y * ncols + x]
			ret_term = self.cache[key][cell_value]
			# print(f'FAST {term} --> {ret_term}')
			return ret_term
		except Exception as e:
			print('hook:', e)


class NavFnAstarMC:
	"""Model checker for the NavFN-A* Maude implementation"""

	ASTAR_MAUDE_PATH = '../maude/astar_no_turnNavFnPlanner_strat.maude'

	def __init__(self, module_name, strat_str, formula_str, use_hook=True, use_strategy=True):
		self.width = None
		self.height = None
		self.map = None
		self.cmap = None
		self.term = None
		self.use_hook = use_hook
		self.use_strategy = use_strategy

		m = maude.getModule(module_name)
		self.module = m

		pose_kind = m.findSort('Pose').kind()
		cmap_kind = m.findSort('CostMap').kind()
		nat_kind = m.findSort('Nat').kind()
		path_kind = m.findSort('Path').kind()
		potential_kind = m.findSort('Potential').kind()

		# Symbol of the a* algorithm
		self.astar = self.get_astar_symbol(pose_kind, cmap_kind, nat_kind, potential_kind, path_kind)

		# Kinds for parsing in other methods
		self.nat_kind = nat_kind
		self.pose_kind = pose_kind

		# Symbols for constructing the map
		self.intlist = m.findSymbol('_,_', (nat_kind, nat_kind), nat_kind)
		self.mtIL = m.parseTerm('mtIL', nat_kind)
		self.cmap_symb = m.findSymbol('{_}', (nat_kind, ), cmap_kind)
		self.noPath = m.parseTerm('noPath', path_kind)
		self.path_union = m.findSymbol('__', (path_kind, path_kind), path_kind)

		# Strategy to control rewriting
		self.astar_st = m.parseStrategy(strat_str)

		# Temporal formulae
		self.formula = m.parseTerm(formula_str)

	def set_map(self, width, cmap):
		"""Set the map"""

		height = len(cmap) // width

		self.width = self.module.parseTerm(str(width), self.nat_kind)
		self.height = self.module.parseTerm(str(height), self.nat_kind)
		self.map = self.cmap_symb(self.intlist(*map(self.module.parseTerm, map(str, cmap))))

		if self.use_hook:
			self.cmap = cmap
			self.map = self.cmap_symb(self.mtIL)
		else:
			self.map = self.cmap_symb(self.intlist(*map(self.module.parseTerm, map(str, cmap))))

		# Number of iterations
		cycles = str(max(len(cmap) // 20, width + height))
		self.iterations = self.module.parseTerm(cycles, self.nat_kind)

	def check(self, origin, dest):

		origin = self.make_bpose(*origin)
		dest = self.make_bpose(*dest)

		self.term = self.get_initial_term(origin, dest)

		if self.use_strategy:
			graph = maude.StrategyRewriteGraph(self.term, self.astar_st)
		else:
			graph = maude.RewriteGraph(self.term)

		result = graph.modelCheck(self.formula)

		return result.holds

	def make_bpose(self, x, y):
		"""Construct a BasePose term"""

		return self.module.parseTerm(f'{{{x}, {y}}}', self.pose_kind)


class NoWallMC(NavFnAstarMC):
	"""Model checker for the NoWallMC property"""

	PROP_NAME = 'NoWall'

	def __init__(self, **kwargs):
		super().__init__('TEST-POT-STRAT',
		                 'astar',
		                 '[] (~ wallInCurrent /\ ~ wallInNext /\ ~ wallInExcess)',
		                 **kwargs)

	def get_astar_symbol(self, pose_kind, cmap_kind, nat_kind, potential_kind, path_kind):
		"""Symbol for op a*i : Pose Pose CostMap Nat Nat Nat ~> Potential"""

		return self.module.findSymbol('a*i', (pose_kind, pose_kind, cmap_kind,
                                                      nat_kind, nat_kind, nat_kind), potential_kind)


	def get_initial_term(self, origin, dest):
		"""Get the initial term for model checking"""

		return self.astar(origin, dest, self.map,
                                  self.width, self.height,
                                  self.iterations)


class EventuallyPathMC(NavFnAstarMC):
	"""Model checker for the EventuallyPath property"""

	PROP_NAME = 'EventuallyPath'

	def __init__(self, **kwargs):
		super().__init__('TEST-PATH-STRAT',
		                 'getPath',
		                 '[] (NonInfinitePotAtInit -> <> PathComputed)',
		                 **kwargs)

	def get_astar_symbol(self, pose_kind, cmap_kind, nat_kind, potential_kind, path_kind):
		"""Symbol for op a* : Pose Pose CostMap Nat Nat Nat Nat -> Path"""

		return self.module.findSymbol('a*', (pose_kind, pose_kind, cmap_kind) +
		                                    (nat_kind, ) * 4, path_kind)

	def get_initial_term(self, origin, dest):
		"""Get the initial term for model checking"""

		return self.astar(origin, dest, self.map,
                                  self.width, self.height,
                                  self.iterations, self.path_cycles)

	def set_map(self, width, cmap):
		super().set_map(width, cmap)

		# Number of iterations for the path calculation
		self.path_cycles = self.module.parseTerm(str(4 * width))


def process_tests(props, tests, args):
	"""Process the given test with the model checker"""

	for test_case in tests:
		w, h, map_data, test_cases, _ = load_test(test_case, ctype=int)

		# Ignore maps outside size bounds
		if not(args.min_size <= (w - 1) * (h - 1) <= args.max_size):
			continue

		print(f'\x1b[1m{simplify_test_name(test_case)}\x1b[0m', end='', flush=True)

		for prop in props:
			prop.set_map(w, map_data)

		start_time = time.perf_counter_ns()

		for (x0, y0, _), (x, y, _) in test_cases:
			for prop in props:
				if not prop.check((x0, y0), (x, y)):
					print(f'\n\x1b[1;31mFAILURE\x1b[0m for {prop.PROP_NAME} {x0},{y0} to {x},{y}', end='', flush=True)

		end_time = time.perf_counter_ns()

		print(f' \x1b[36m{format_duration(end_time - start_time)}\x1b[0m')


if __name__ == '__main__':
	import argparse

	parser = argparse.ArgumentParser(description='Model checking the NavFN-A* Maude implementation')
	parser.add_argument('test', help='Test case to be model checked', nargs='*')
	parser.add_argument('--no-strat', help='Do not control the A* algorithm with a strategy',
	                    dest='use_strategy', action='store_false')
	parser.add_argument('--no-hook', help='Do not use the hook for accessing the map',
	                    dest='use_hook', action='store_false')
	parser.add_argument('--max-size', '-M', help='Ignore maps of a greater size', type=int, default=int(1e6))
	parser.add_argument('--min-size', '-m', help='Ignore maps of a smaller size', type=int, default=0)

	args = parser.parse_args()

	maude.init()
	maude.load(NavFnAstarMC.ASTAR_MAUDE_PATH)

	# Objects handling the LTL properties to be checked
	props = [
		NoWallMC(use_hook=args.use_hook, use_strategy=args.use_strategy),
		EventuallyPathMC(use_hook=args.use_hook, use_strategy=args.use_strategy),
	]

	# Configure the hook that enables random access to the map
	if args.use_hook:
		mapHook = MapHook()
		maude.connectEqHook('get', mapHook)

		for prop in props:
			mapHook.add_user(str(prop.module), prop)

	process_tests(props, args.test, args)
