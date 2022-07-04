#!/usr/bin/env python3

#
# This script checks all verification conditions defined in vcs_navfnplanner.maude
#

import maude
import z3 as smt
import smt_converter as conv

# Adaptation for versions before 0.6
if maude.Symbol.__hash__ is None:
  maude.Symbol.__hash__ = lambda self: hash(str(self))
  maude.Sort.__hash__ = lambda self: hash(str(self))

  
def in_green(string):
  return "\033[92m" + string + "\033[0m"

def in_red(string):
  return "\033[91m" + string + "\033[0m"

def in_cyan(string):
  return "\033[96m" + string + "\033[0m"
  

class VerificationCondition:
  def __init__(self, maude_module, name, sorts_names, description = None):
    self.name = name
    self.sorts_names = sorts_names
    self.maude_module = maude_module
    if description is None:
      self.description = name
    else:
      self.description = description
    
  def _maude_term_as_string(self):
    args = []
    num = 0
    for sort in self.sorts_names:
      args.append(f"X{num}:{sort}")
      num = num + 1
    args_string = ", ".join(args)
    return f"{self.name}({args_string})"
  
  def check(self):
    sc = conv.SMTConverter(self.maude_module)
    term = self.maude_module.parseTerm("not " + self._maude_term_as_string())
    term.reduce()
    tr = sc.translate(term)
    
    solver = smt.Solver()
    solver.add(tr)
    print(f" * {self.description}... ", end = "")
    
    result = solver.check()
    
    if str(result) == "unsat":
      print(in_green("✓"))
    elif str(result) == "sat":
      print(in_red("𐄂"))
      print(solver)
    else:
      print(in_red("?"))
    

class VerificationModule:
  def __init__(self, module_name, sorts_names):
    self.module_name = module_name
    self.module = maude.getModule(module_name)
    self.sorts_names = sorts_names
    self.vcs = []
    
  def add_vc(self, name, description = None):
    self.vcs.append(VerificationCondition(self.module, name, self.sorts_names, description))
    
  def check(self):
    print(in_cyan(f"Checking module {self.module_name}:"))
    for vc in self.vcs:
      vc.check()
  
  
modules = [
    ("INIT-CURRENT-QUEUE-VERIF", 
      ["Pose", "PotentialMap", "CostMap", "PoseQueue"],
      [
        ("init-current-queue-vc1", "Postcondition holds")
      ]),
    ("UPDATE-POTENTIAL-VERIF", 
      ["PotentialMap", "Pose", "Pose", "CostMap", "RealInf", "RealInf", "PotentialMap"],
      [
        ("update-potential-vc1", "Call to minus meets precondition"),
        ("update-potential-vc2", "Call to greater than meets precondition"),
        ("update-potential-vc3", "Case diff > h"),
        ("update-potential-vc4", "Case diff <= h: diff is real"),
        ("update-potential-vc5", "Case diff <= h implies postcondition"),
      ]),
    ("TRAVERSE-NEIGHBOUR-VERIF", 
      ["Pose", "Pose", "Pose", "CostMap", "PotentialMap", "Real",
       "PoseQueue", "PoseQueue", "PoseQueue", "PoseQueue"],
      [
        ("traverse-neighbour-vc1", "Call to greater than meets precondition"),
        ("traverse-neighbour-vc2", "Case not GreaterThan"),
        ("traverse-neighbour-vc3", "Case GreaterThan, PM[P] + H < Threshold"),
        ("traverse-neighbour-vc4", "Case GreaterThan, PM[P] + H >= Threshold"),
      ]),
    ("TRAVERSE-NEIGHBOURS-VERIF", 
      ["Pose", "Pose", "CostMap", "PotentialMap", "Real",
       "PoseQueue", "PoseQueue", "PoseQueue", "PoseQueue"],
      [
        ("traverse-neighbours-vc1", "Call to traverse-neighbour than meets precondition (north)"),
        ("traverse-neighbours-vc2", "Call to traverse-neighbour than meets precondition (south)"),
        ("traverse-neighbours-vc3", "Call to traverse-neighbour than meets precondition (west)"),
        ("traverse-neighbours-vc4", "Call to traverse-neighbour than meets precondition (east)"),
        ("traverse-neighbours-vc5", "Postcondition holds"),
      ]),
    ("ASTAR-ITERATION-VERIF", 
      ["Pose", "Pose", "CostMap", "PotentialMap", "PoseQueue", "PoseQueue", "PoseQueue", "Real", "Integer", "PotentialMap"],
      [
        ("astar-iteration-vc1", "NumIterations reaches 0"),
        ("astar-iteration-vc2", "Start position's potential lower than infinity"),
        ("astar-iteration-vc3", "Queue nonempty, front position closed: precondition recursive call"),
        ("astar-iteration-vc4", "Queue nonempty, front position closed: postcondition follows"),
        ("astar-iteration-vc5", "Queue nonempty, front position open: precondition of MinVertical"),
        ("astar-iteration-vc6", "Queue nonempty, front position open: precondition of MinHorizontal"),
        ("astar-iteration-vc7", "Queue nonempty, front position open: at least one of minV and minH is not infinity"),
        ("astar-iteration-vc8", "Queue nonempty, front position open: precondition of UpdatePotential"),
        ("astar-iteration-vc9", "Queue nonempty, front position open: validity of queues"),
        ("astar-iteration-vc10", "Queue nonempty, front position open: precondition of TraverseNeighbours"),
        ("astar-iteration-vc11", "Queue nonempty, front position open: validity of updated queues"),
        ("astar-iteration-vc12", "Queue nonempty, front position open: precondition of recursive call"),
        ("astar-iteration-vc13", "Queue nonempty, front position open: postcondition follows"),
        ("astar-iteration-vc14", "Queue empty, and next as well: precondition of recursive call"),
        ("astar-iteration-vc15", "Queue empty, and next as well: postcondition"),
        ("astar-iteration-vc16", "Queue empty, but not next: precondition of recursive call"),
        ("astar-iteration-vc17", "Queue empty, but not next: postcondition"),
      ]),
    ("ASTAR-VERIF", 
      ["Pose", "Pose", "CostMap", "Integer", "PotentialMap", "Real", "Real", "PoseQueue", "PoseQueue", "PoseQueue"],
      [
        ("astar-vc1", "Call to BuildInitialPotentialMap meets precondition"),
        ("astar-vc2", "Call to EuclidDistance meets precondition"),
        ("astar-vc3", "Call to InitCurrentQueue meets precondition"),
        ("astar-vc4", "Call to AStarIteration meets precondition"),
      ])
  ]
  
if __name__ == '__main__':
  maude.init()
  maude.load('vcs_navfnplanner.maude')
  
  for (mod_name, sorts, vcs) in modules:
    vm = VerificationModule(mod_name, sorts)
    for (name, description) in vcs:
      vm.add_vc(name, description)
    vm.check()
  