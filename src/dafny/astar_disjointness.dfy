/*
 * Verification of the Maude implementation of A* algorithm.
 * 
 * This version includes an extra invariant, which specifies that there is no pair of nodes
 * with the same pose.
 */

datatype Point = Point(x: real, y: real, z: real)
datatype Pose = Pose(position: Point, angle: int)   // TODO: limit angle to 0..360
datatype OptionPose = Just(pose: Pose) | Nothing

datatype CostMap = CostMap(open: Point -> bool, num_rows: real, num_cols: real)
type Path = seq<Pose>

// We cannot declare path as ghost. If it were, Node would not be comparable by ==,
// which implies that Nodes cannot be stored within sets.
datatype Node = Node(pose: Pose, 
                     currentCost: real,
                     estimatedRemaining: real,
                     estimatedTotal: real,
                     previous: OptionPose,
                     path: seq<Pose>) // ghost field


function abs(x: real): real {
    if (x >= 0.0) then x else -x
}

/* We say that a closed node is valid if
    - its predecessor, if there is such, is in the set of closed nodes,
    - the ghost field 'path' matches the linked list of predecessors' poses,
    - the first element of the path is the starting point,
    - consecutive poses in the path contain adjacent positions
    - every pose in the path belongs to the open set
*/
predicate ValidClosedNode(n: Node, closed: set<Node>, start: Pose, costMap: CostMap) {
    n in closed &&
    match n.previous {
        case Nothing => n.path == [n.pose]
        case Just(p') =>
            exists n': Node :: n' in closed && n'.pose == p' && (n.path == n'.path + [n.pose]) 
    } &&
    |n.path| > 0 &&
    n.path[0] == start &&
    AdjacentPathRec(n.path) &&
    OpenPath(n.path, costMap)
}

/* A set of closed nodes is valid if every node in that set is valid, according
  to the definition above */
predicate ValidClosedSet(closed: set<Node>, start: Pose, costMap: CostMap) {
    forall n: Node | n in closed :: ValidClosedNode(n, closed, start, costMap)        
}


/* An open node is valid if, when we add it to a valid set of closed nodes,
   that set remains valid */
predicate ValidOpenNode(n: Node, closed: set<Node>, start: Pose, costMap: CostMap)
{
    ValidClosedNode(n, closed + {n}, start, costMap)
}

/* A set of open nodes is valid if every node in it is valid */
predicate ValidOpenSet(open: set<Node>, closed: set<Node>, start: Pose, costMap: CostMap)
{
    forall n: Node | n in open :: ValidOpenNode(n, closed, start, costMap)
}


predicate AdjacentPoses(pose1: Pose, pose2: Pose)
{
    var x1 := pose1.position.x;
    var x2 := pose2.position.x;
    var y1 := pose1.position.y;
    var y2 := pose2.position.y;
    abs(x1 - x2) <= 1.0 && abs(y1 - y2) <= 1.0
}

/* This predicate checks whether the poses in consecutive positions of a path
   are in adjacent positions.
*/
  
predicate AdjacentPathRec(path: seq<Pose>)
{
    if |path| <= 1 
        then true 
        else AdjacentPathRec(path[..|path| - 1]) && AdjacentPoses(path[|path| - 2], path[|path| - 1])
}


/* This predicate is equivalent to the previous one, but it is formulated by means of
   a quantifier over the path, instead of giving a recursive definition.

   This definition is more natural to humans, but Dafny handles better 'AdjacentPathRec'
*/
predicate AdjacentPath(path: seq<Pose>)
{
    |path| > 1 ==>
        forall i: int | 0 <= i < |path| - 1 :: AdjacentPoses(path[i], path[i + 1])
}

/* Here Dafny proves by itself that the two notions of valid path given previously
  are, in fact, equivalent */
lemma AdjacentNotionsEquivalent(path: seq<Pose>)
    ensures AdjacentPath(path) == AdjacentPathRec(path)
{
}


/* A path is open if every position occurring in it is open, according to
  the cost map. In other words, a path is open if does not hit any obstacles
  in the map */
predicate OpenPath(path: seq<Pose>, costMap: CostMap) {
    forall i: int | 0 <= i < |path| :: costMap.open(path[i].position)
}


predicate AllPosesDifferent(nodes: set<Node>) {
    forall n, m: Node | n in nodes && m in nodes :: n != m ==> n.pose != m.pose
}

predicate DisjointPoses(set1: set<Node>, set2: set<Node>) {
    forall x: Node, y: Node | x in set1 && y in set2 :: x.pose != y.pose
}


// The following functions and methods mimic their counterparts in the Maude implementation

/* The following function, given an angle (where 0 denotes 'facing upwards') in which the robot
 is facing, it returns the changes in the coordinates when the robot moves forward

 Maude: op getMove : Int ~> Pair 
 */
function method GetDeltaPos(angle: int): (real, real)
    ensures match GetDeltaPos(angle) { case (x, y) => abs(x) <= 1.0 && abs(y) <= 1.0 }
{
    if angle == 0 then (1.0, 0.0)
    else if angle == 45 then (1.0, -1.0)
    else if angle == 90 then (0.0, -1.0)
    else if angle == 135 then (-1.0, -1.0)
    else if angle == 180 then (-1.0, -0.0)
    else if angle == 225 then (-1.0, 1.0)
    else if angle == 270 then (0.0, 1.0)
    else if angle == 315 then (1.0, 1.0)
    else (0.0, 0.0)
}


/* A* algorithm: Given an initial pose, a destination and a map of obstacles, returns
               the shortest path between these two points that does not hit any obstacles

   Here we verify that the path starts and ends in the given positions, that it does not hit obstacles,
   and that its poses in consecutive positions are adjacent to each other.

   In Maude:
    op a* : Pose Pose CostMap  Float  Float -> Path .

   The last two Floats in the Maude specification denote the dimensions of the CostMap, which here
   are part of the CostMap data type
*/
method AStar(initialPose: Pose, goal: Point, costMap: CostMap) returns (path: Path)
    requires costMap.open(initialPose.position)
    ensures path != [] ==> path[0] == initialPose && path[|path| - 1].position == goal && AdjacentPath(path) && OpenPath(path, costMap);
    decreases *
{
    var initOpen := { Node(initialPose, 0.0, 0.0, 0.0, Nothing, [initialPose]) };
    path := AStarAux(goal, costMap, initOpen, {}, initialPose);
        assert AdjacentPathRec(path);
        AdjacentNotionsEquivalent(path);
        assert AdjacentPath(path);
}


/*
  In Maude:
    op a* : Pose CostMap  Float  Float NodeSet NodeSet -> Path .
*/
method AStarAux(goal: Point, costMap: CostMap, open: set<Node>, closed: set<Node>, ghost start: Pose) returns (path: Path)
    requires ValidClosedSet(closed, start, costMap)
    requires ValidOpenSet(open, closed, start, costMap)
    requires AllPosesDifferent(open + closed)    
    ensures path != [] ==> path[0] == start && path[|path| - 1].position == goal && AdjacentPathRec(path) && OpenPath(path, costMap);
    decreases *
{
    if open == {} {
        path := [];
    } else {
        var current, newOpen := Min(open);
        assert current in open;
        assert ValidOpenSet(newOpen, closed, start, costMap);
        if current.pose.position == goal {
            assert ValidOpenNode(current, closed, start, costMap);
            assert ValidClosedNode(current, closed + {current}, start, costMap);
            assert |current.path| > 0;
            assert current.path[|current.path| - 1] == current.pose;
            path := ComputePath(current, closed, [], start, costMap);
            assert path == current.path;
            assert path[|path| - 1].position == goal;
            assert path[0] == start;
        } else {
            var succs := OpenSuccessors(goal, current, costMap, newOpen, closed, start);
            assert ValidOpenSet(succs, closed + {current}, start, costMap);
            assert ValidClosedNode(current, closed + {current}, start, costMap);
            path := AStarAux(goal, costMap, succs, closed + { current }, start);
        }
    }
}

/*
    Given a set of nodes, it returns one with a minimal estimated cost, and
    the result of removing it from the input set.

    In Maude:
        op min : NodeSet ~> ResPair .
*/
method Min(nodes: set<Node>) returns (min: Node, rest: set<Node>)
    requires |nodes| > 0
    ensures min in nodes
    ensures rest == nodes - {min}
    {
    var remaining := nodes;
    min :| min in nodes;
    remaining := remaining - { min };
    while remaining != {}
        decreases remaining
        invariant min in nodes
    {
        var current :| current in remaining;
        if current.estimatedTotal < min.estimatedTotal {
            min := current;
        }
        remaining := remaining - { current };
    }
    rest := nodes - {min};
}


/*
  In Maude:
     op generate : Pose Node CostMap NodeSet NodeSet Float Float -> NodeSet .
*/
method OpenSuccessors(goal: Point, node: Node, costMap: CostMap, open: set<Node>, closed: set<Node>, ghost start: Pose) returns (newOpen: set<Node>)
    requires ValidOpenSet(open, closed, start, costMap)
    requires ValidClosedSet(closed, start, costMap)
    requires ValidOpenNode(node, closed, start, costMap)
    requires AllPosesDifferent(open + closed + {node});
    requires node !in open
    ensures ValidOpenSet(newOpen, closed + {node}, start, costMap)
    ensures AllPosesDifferent(newOpen + closed + {node})
{
    var successorsLeft := GenerateSuccessorsAux(goal, node, costMap, 45, 0, 0.0, 5, start);
        assert ValidOpenSet(successorsLeft, {node}, start, costMap);
        assert ValidOpenSet(successorsLeft, closed + {node}, start, costMap);

    var successorsRight := GenerateSuccessorsAux(goal, node, costMap, -45, -45, 0.0, 3, start);
        assert ValidOpenSet(successorsRight, {node}, start, costMap);
        assert ValidOpenSet(successorsRight, closed + {node}, start, costMap);
        assert ValidOpenSet(successorsLeft + successorsRight, closed + {node}, start, costMap);        
    
    assume DisjointPoses(successorsLeft, successorsRight);
    assume AllPosesDifferent(successorsLeft);
    assume AllPosesDifferent(successorsRight);
    assert DisjointPoses(successorsLeft, {node});
    assert DisjointPoses(successorsRight, {node});    

    assert AllPosesDifferent(successorsLeft + successorsRight + {node});
    var withoutClosed := RemoveClosed(successorsLeft + successorsRight, closed);
        assert ValidOpenSet(withoutClosed, closed + {node}, start, costMap);
        assert ValidOpenSet(withoutClosed, closed + {node}, start, costMap);
        assert withoutClosed <= successorsLeft + successorsRight;
        assert AllPosesDifferent(withoutClosed);
        assert DisjointPoses(successorsLeft + successorsRight, {node});
        assert DisjointPoses(withoutClosed, closed + {node});

    newOpen := UpdateOpen(withoutClosed, open, closed + {node}, start, costMap);
        assert ValidOpenSet(newOpen, closed + {node}, start, costMap);
}

/*
    In Maude:
     op generate : Pose Node CostMap Float Float Int Int Float Nat -> NodeSet .
*/
method GenerateSuccessorsAux(goal: Point, node: Node, costMap: CostMap,
                             deltaAngle: int, currentAngle: int, turnCost: real,
                             numSteps: nat, ghost start: Pose) returns (successors: set<Node>)
    requires |node.path| > 0 && node.path[0] == start && node.path[|node.path| - 1] == node.pose 
    requires AdjacentPathRec(node.path) && OpenPath(node.path, costMap)
    ensures ValidOpenSet(successors, {node}, start, costMap)
    ensures DisjointPoses(successors, {node})    
    decreases numSteps
{
    if numSteps == 0 {
        successors := {};
    } else {
        var newAngle := node.pose.angle + currentAngle;
        if {
            case newAngle < 0 => newAngle := newAngle + 360;
            case newAngle >= 360 => newAngle := newAngle - 360;
            case 0 <= newAngle < 360 => {}
        }
        var (deltaPosX, deltaPosY) := GetDeltaPos(newAngle);
        assume abs(deltaPosX) > 0.0;
        assume abs(deltaPosY) > 0.0;
        assert abs(deltaPosX) <= 1.0;
        assert abs(deltaPosY) <= 1.0;
        var newPosition := Point(node.pose.position.x + deltaPosX, node.pose.position.y + deltaPosY, node.pose.position.z);
        if (costMap.open(newPosition) && newPosition.x >= 0.0 && newPosition.y >= 0.0 &&
            newPosition.x < costMap.num_cols && newPosition.y < costMap.num_rows) {
            
            var costMovement := if (newAngle == 45 || newAngle == 135 || newAngle == 225 || newAngle == 315) then 1.4 else 1.0;
            var nextCost := node.currentCost + turnCost + costMovement;
            var nextEstimate := 0.0; // TODO: call heuristic
            var newNode := Node(
                Pose(newPosition, newAngle),
                nextCost,
                nextEstimate,
                nextCost + nextEstimate,
                Just(node.pose),
                node.path + [Pose(newPosition, newAngle)] 
            );
            assert costMap.open(newNode.pose.position);
            assert AdjacentPoses(Pose(newPosition, newAngle), node.pose);
            assert |node.path| > 0;
            assert node.path[|node.path| - 1] == node.pose;
            var remainingSucs := GenerateSuccessorsAux(goal, node, costMap, deltaAngle, currentAngle + deltaAngle, turnCost + 0.25, numSteps - 1, start);
            successors := remainingSucs + { newNode };
        } else {
            successors := GenerateSuccessorsAux(goal, node, costMap, deltaAngle, currentAngle + deltaAngle, turnCost + 0.25, numSteps - 1, start);
        }
    }
}                             

/*
    In Maude:
     op removeClosed : NodeSet NodeSet -> NodeSet .
*/
method RemoveClosed(input_set: set<Node>, closed_set: set<Node>) returns (result: set<Node>)    
    ensures result <= input_set
    ensures DisjointPoses(closed_set, result)
{
    result := {};
    var input := input_set;
    while input != {}
        decreases input
        invariant result <= input_set
        invariant input <= input_set
        invariant DisjointPoses(closed_set, result)
    {
        var current :| current in input;
        if other :| (other in closed_set && other.pose == current.pose) {
            
        } else {
            result := result + { current };
        }
        input := input - { current };
    }    
    return result;
}


/*
    In Maude:
     op updateOpen : NodeSet NodeSet -> NodeSet .
*/
method UpdateOpen(input_set: set<Node>, open_set: set<Node>, ghost closed: set<Node>, ghost start: Pose, ghost costMap: CostMap)  returns (result: set<Node>)
    requires ValidOpenSet(input_set, closed, start, costMap)
    requires ValidOpenSet(open_set, closed, start, costMap)
    requires AllPosesDifferent(open_set + closed)
    requires DisjointPoses(input_set, closed)
    requires AllPosesDifferent(input_set)

    ensures ValidOpenSet(result, closed, start, costMap)
    ensures AllPosesDifferent(result + closed)
{
    result := open_set;
    var input := input_set;
    while input != {}
        decreases input
        invariant input <= input_set
        invariant ValidOpenSet(result, closed, start, costMap)
        invariant AllPosesDifferent(result + closed)
        invariant AllPosesDifferent(input)
        invariant DisjointPoses(input, closed)
    {
        var current :| current in input;
        input := input - { current };
        if other :| (other in result && other.pose == current.pose) {
            if current.estimatedTotal < other.estimatedTotal {
                result := result - { other } + { current };
            }
        } else {
            result := result + { current };
        }
    }
}



/*
    It reconstructs the path from the goal node, by following the chain of previous
    poses.
    
    In Maude:
     op computePath : Node NodeSet Path -> Path .
*/
method ComputePath(n: Node, closed: set<Node>, accum: seq<Pose>, ghost start: Pose, ghost costMap: CostMap) returns (path: seq<Pose>)
    requires ValidClosedSet(closed, start, costMap)
    requires ValidOpenNode(n, closed, start, costMap)
    ensures path == n.path + accum
    decreases |n.path|
{
    match n.previous {
        case Nothing => {
            path := [n.pose] + accum;
        }
        case Just(p) =>  {            
            // TODO: This makes use of a ghost field. It should be changed
            //       by a call to a method that, given a pose and a set,
            //       returns the node in the set with that pose.
            //
            //       We would have to prove that all the poses in the set
            //       of closed nodes are different.
            var n' :| n' in closed && n'.pose == p && n.path == n'.path + [n.pose]; 
            path := ComputePath(n', closed, [n.pose] + accum, start, costMap);
        }
    }
}