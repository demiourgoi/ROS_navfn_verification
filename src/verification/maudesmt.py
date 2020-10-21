#
# Extension of the Maude SMT support for other theories
#

import maude
import pysmt
import pysmt.shortcuts as smt

# Adaptation for versions before 0.6
if maude.Symbol.__hash__ is None:
	maude.Symbol.__hash__ = lambda self: hash(str(self))
	maude.Sort.__hash__ = lambda self: hash(str(self))

class SMTConverter:
	"""Convert Maude terms to SMT-LIB"""

	def __init__(self, module):
		self.module = module
		self.ops_table = {}
		self.sorts_table = {}

		self.boolean_sort = None
		self.integer_sort = None
		self.real_sort = None
		self.intlit_symb = None
		self.reallit_symb = None

		self.load()

	def load(self):
		"""Find the symbols for the SMT operators"""

		self.load_boolean()
		self.load_integer()
		self.load_real()
		self.load_real_integer()

		self.sorts_table[self.boolean_sort] = smt.BOOL
		self.sorts_table[self.integer_sort] = smt.INT
		self.sorts_table[self.real_sort] = smt.REAL

	def load_boolean(self):
		self.boolean_sort = self.module.findSort('Boolean')
		boolk = self.boolean_sort.kind()

		boolean_ops = [
			('true', 0, smt.TRUE),
			('false', 0, smt.FALSE),
			('not_', 1, smt.Not),
			('_and_', 2, smt.And),
			('_xor_', 2, smt.Xor),
			('_or_', 2, smt.Or),
			('_implies_', 2, smt.Implies),
			('_===_', 2, smt.EqualsOrIff),
			('_=/==_', 2, smt.NotEquals),
			('_?_:_', 3, smt.Ite)
		]

		for name, arity, target in boolean_ops:
			symb = self.module.findSymbol(name, [boolk] * arity, boolk)
			if symb is None:
				raise ValueError(f'cannot find {name} Boolean symbol')
			self.ops_table[symb] = target

	def load_integer(self):
		self.integer_sort = self.module.findSort('Integer')

		kinds = [self.boolean_sort.kind(), self.integer_sort.kind()]

		integer_ops = [
			('-_', [1], 1, lambda x: smt.Minus(smt.Nat(0), x)),
			('_+_', [1, 1], 1, smt.Plus),
			('_*_', [1, 1], 1, smt.Times),
			('_-_', [1, 1], 1, smt.Minus),
			# Not supported by pysmt
			('_div_', [1, 1], 1, None),
			('_mod_', [1, 1], 1, None),
			('_<_', [1, 1], 0, smt.LT),
			('_<=_', [1, 1], 0, smt.LE),
			('_>_', [1, 1], 0, smt.GT),
			('_>=_', [1, 1], 0, smt.GE),
			('_===_', [1, 1], 0, smt.Equals),
			('_=/==_', [1, 1], 0, smt.NotEquals),
			('_?_:_', [0, 1, 1], 1, smt.Ite)
		]

		for name, domain, rtype, target in integer_ops:
			symb = self.module.findSymbol(name, [kinds[k] for k in domain], kinds[rtype])
			if symb is None:
				raise ValueError(f'cannot find {name} integer symbol')
			self.ops_table[symb] = target

		self.intlit_symb = self.module.findSymbol('<Integers>', [], self.integer_sort.kind())

	def load_real(self):
		self.real_sort = self.module.findSort('Real')

		kinds = [self.boolean_sort.kind(), self.real_sort.kind()]

		real_ops = [
			('-_', [1], 1, lambda x: smt.Minus(smt.Real(0.0), x)),
			('_+_', [1, 1], 1, smt.Plus),
			('_*_', [1, 1], 1, smt.Times),
			('_-_', [1, 1], 1, smt.Minus),
			('_/_', [1, 1], 1, smt.Div),
			('_<_', [1, 1], 0, smt.LT),
			('_<=_', [1, 1], 0, smt.LE),
			('_>_', [1, 1], 0, smt.GT),
			('_>=_', [1, 1], 0, smt.GE),
			('_===_', [1, 1], 0, smt.Equals),
			('_=/==_', [1, 1], 0, smt.NotEquals),
			('_?_:_', [0, 1, 1], 1, smt.Ite)
		]

		for name, domain, rtype, target in real_ops:
			symb = self.module.findSymbol(name, [kinds[k] for k in domain], kinds[rtype])
			if symb is None:
				raise ValueError(f'cannot find {name} real symbol')
			self.ops_table[symb] = target

		self.reallit_symb = self.module.findSymbol('<Reals>', [], self.real_sort.kind())

	def load_real_integer(self):
		kinds = [self.boolean_sort.kind(), self.integer_sort.kind(), self.real_sort.kind()]

		real_integer_ops = [
			('toReal', [1], 2, smt.ToReal),
			# Not supported by pysmt
			('toInteger', [2], 1, lambda x: x),
			('isInteger', [2], 0, None)
		]

		for name, domain, rtype, target in real_integer_ops:
			symb = self.module.findSymbol(name, [kinds[k] for k in domain], kinds[rtype])
			if symb is None:
				raise ValueError(f'cannot find {name} real-integer symbol')
			self.ops_table[symb] = target

	def _parse_array_type(self, type_name):
		params = type_name[7:-2]

		# There may be parameterized views, so there may
		# be more than a comma in the typename
		comma_pos = 0
		level = 0

		for c in params:
			if c == ',' and level == 0:
				break
			if c == '{':
				level += 1
			elif c == '}':
				level -= 1

			comma_pos += 1

		key_type = self._smt_sort(self.module.findSort(params[:comma_pos-1]))
		value_type = self._smt_sort(self.module.findSort(params[comma_pos+1:]))

		return smt.ArrayType(key_type, value_type)

	def _smt_sort(self, sort):
		"""Correspondence between Maude and SMT sorts"""

		smt_sort = self.sorts_table.get(sort)

		if smt_sort is not None:
			return smt_sort

		# Array sort (if renamed or if the view names do not coincide
		# with the name of the sorts, it must be introduced manually
		# into sorts_table)
		if str(sort).startswith('Array`{'):
			smt_sort = self._parse_array_type(str(sort))

		# Custom sort
		else:
			smt_sort = smt.Type(str(sort))

		self.sorts_table[sort] = smt_sort
		return smt_sort

	def _make_fnsymb(self, symbol):
		rtype = self._smt_sort(symbol.getRangeSort())
		domain = [self._smt_sort(sort) for sort in symbol.getOpDeclarations()[0].getDomainAndRange()]

		return smt.Symbol(str(symbol), smt.FunctionType(rtype, domain[:-1]))

	def _make_function(self, symbol):
		symb = lambda *args: smt.Function(self._make_fnsymb(symbol), args)
		self.ops_table[symbol] = symb
		return symb

	def _make_polymorph(self, symbol):
		symbol_name = str(symbol)

		polymorph_ops = {
			'forall_._': lambda x, y: smt.ForAll([x], y),
			'exists_._': lambda x, y: smt.Exists([x], y),
			# Not really polymorph, but parameterized
			'_[_]': smt.Select,
			'_[_->_]': smt.Store,
			'_===_': smt.EqualsOrIff,
			'_=/==_': smt.NotEquals,
			'_?_:_': smt.Ite
		}

		symb = polymorph_ops.get(symbol_name)

		# Add it to the table for reuse
		if symb is not None:
			self.ops_table[symbol] = symb

		return symb

	def translate(self, term):
		"""Translate a Maude term into an SMT formula"""

		symbol = term.symbol()

		# Variable
		if str(symbol) == str(term.getSort()):
			return smt.Symbol(str(term), self._smt_sort(term.getSort()))
		# Integer constant
		elif symbol == self.intlit_symb:
			return smt.Int(int(term))
		# Real constant
		elif symbol == self.reallit_symb:
			return smt.Real(float(term))
		# Other symbols
		else:
			symb = self.ops_table.get(symbol)

			# If the symbol is not in the table, it may be a
			# polymorph for a custom type...
			if symb is None:
				symb = self._make_polymorph(symbol)

				# ...or an uninterpreted function
				if symb is None:
					symb = self._make_function(symbol)

			return symb(*[self.translate(arg) for arg in term.arguments()])


if __name__ == '__main__':

	maude.init()
	maude.load('smtex')

	smtmod = maude.getCurrentModule()

	sc = SMTConverter(smtmod)

	# Archimedian property
	t1 = sc.translate(smtmod.parseTerm('forall C:Real . (C:Real > 0/1 implies exists N:Integer . 1/1 / toReal(N:Integer) < C:Real)'))
	# Reals are not bounded
	t2 = sc.translate(smtmod.parseTerm('forall N:Integer . exists X:Real . X:Real > toReal(N:Integer)'))
	# Uinterpreted functions
	t3 = sc.translate(smtmod.parseTerm('log(R:Real) === 1/1'))
	t4 = sc.translate(smtmod.parseTerm('log(R:Real) === 1/1 and log(R:Real) === 2/1'))
	t5 = sc.translate(smtmod.parseTerm('forall B:Being . human(B:Being) implies mortal(B:Being) and human(socrate) and exists B:Being . mortal(B:Being)'))
	# Arrays
	t6 = sc.translate(smtmod.parseTerm('(A:Array{Integer, Integer}[N:Integer -> 1])[N:Integer] === 1'))
	t7 = sc.translate(smtmod.parseTerm('(A:Array{Integer, Integer}[N:Integer -> 1])[N:Integer] === 1 and length(A:Array{Integer,Integer}) > N:Integer'))

	# Check properties
	for t in [t1, t2, t3, t4, t5, t6, t7]:
		try:
			print(t, '--', 'sat' if smt.is_sat(t) else 'unsat')
		except pysmt.exceptions.SolverReturnedUnknownResultError:
			print(t, '--', 'undef')
