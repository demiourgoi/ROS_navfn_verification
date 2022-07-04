This directoy contains the Maude implementation of the NavFn planner of ROS Navigation 2. The most relevant elements are:

* [**astar_navfnplanner.maude**](astar_navfnplanner.maude), with the Maude implementation of the NavFn planner, which have been differentially tested with the original one.
* [**common.maude**](common.maude), with general definitions for maps, paths, etc.
* [**astar_textbook.maude**](astar_textbook.maude), with the standard specification of the A* algorithm as a functional module. It includes constants for taking into account how expensive turns are.
* [**verification**](verification), with the deductive verification files for the Maude implementation. It includes a generalization of the connection from Maude to SMT solvers.
* [**integration**](integration), with the required files for integrating the Maude planner as a node in the ROS ecosystem and running simulations.
* [**variations**](variations), with some additional variations of the A* algorithm and some auxiliary files.
   * [**astar_no_turn.maude**](variations/astar_no_turn.maude), a variation of `astar_textbook.maude` that has been simplified to consider, as ROS does, that turns are "free".
   * [**astar_no_turn_rew.maude**](variations/astar_no_turn_rew.maude), the translation of `astar_no_turn.maude` to a rewrite system where model checking can be applied.
   * [**astar_navfnplanner_eqs.maude**](variations/astar_navfnplanner_eqs.maude), with an equational-only version of `astar_navfnplanner.maude`.
   * [**astar_navfnplanner_strat.maude**](variations/astar_navfnplanner_strat.maude), where some strategies are defined on top of `astar_navfnplanner.maude`.
   * [**astar-perf.maude**](variations/astar-perf.maude), for performance evaluation of **astar.maude**.