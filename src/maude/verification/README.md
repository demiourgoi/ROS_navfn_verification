# Formal verification of ROS Navigation 2

Two approaches have been followed to formally prove some [properties](#Properties) of the Maude specification of the ROS's `NavFn` planner:

1. Translating it into the [Dafny](https://github.com/dafny-lang/dafny), a programming language and verifier based on Hoare logic. These files are included in the [**dafny**](../../dafny) folder.

2. Manually expressing verification conditions in Maude and having them checked by SMT solvers. Several files are involved:

   * **vcs_navplanner.maude** includes the verification conditions for the `NavFn` planner implementation in Maude. The precondition and postcondition of a function *f* are Maude SMT terms assigned to the operators *f*`-precond` and *f*`-postcond` respectively. The following script can be used to check them.

   * **check_navfnplanner.py** triggers the verification of the properties expressed in the Maude in the previous file. The verification conditions are passed to the SMT solver and the result is printed on the screen.

   * **smtex.maude** and **smt_converter.py** extend the Maude support for SMT to the theories of uninterpreted functions, arrays and quantified formulae. The [`maude`](https://pypi.org/project/maude) and [Z3](https://github.com/Z3Prover/z3) Python bindings are used for the communication with Maude and the SMT solver respectively.

## Properties

The verified properties are:

1. *Position safety*: the positions with finite potential do not contain obstacles.
2. *Progress*: every position with finite potential (excluding the goal) has an adjacent position with lower potential.
