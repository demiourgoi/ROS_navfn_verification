This folder contains the following files:
* **common.maude**, with general definitions for maps, paths, etc.
* **astar.maude**, with the standard specification of the A* algorithm as a functional module. It includes constants for taking into account how expensive turns are.
* **astar-perf.maude**, for performance evaluation of **astar.maude**.
* **astar_no_turn.maude**, with the standard specification of the A* algorithm as a functional module. It has been simplified to consider, as ROS does, that turns are "free".
* **astar_no_turn_rew.maude**, with the translation of the standard A* from an equational (**astar_no_turn.maude**) to a rewrite system where model checking can be applied.
* **astar_no_turnNavFnPlanner.maude**, with the computation of the potential map and supporting two versions of the **computePath** function, in charge of computing the path from the potential map.
* **astar_no_turnNavFnPlanner_rew.maude**, where the computation of the potential map is computed by means of rewrite rules, allowing us to perform model checking.