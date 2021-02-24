datatype Point = Point(row: int, col: int)
datatype Pose = Pose(pos: Point)
datatype CostMap = CostMap(value: Point -> real, numRows: nat, numCols: nat)
datatype RealInf = Real(r: real) | Infinity


type Path = seq<Pose>
type PotentialMap = seq<seq<RealInf>>

const obstacleCost: real := 254.0
const mapCost: real := 50.0


predicate method open(costMap: CostMap, row: int, col: int)
{
  costMap.value(Point(row, col))  < obstacleCost
}

predicate PotentialMapHasDimensions(p: PotentialMap, numRows: nat, numCols: nat) {
  |p| == numRows
  && forall r: seq<RealInf> | r in p :: |r| == numCols
}

predicate ValidPotentialMap(pot: PotentialMap, costMap: CostMap)
{
  PotentialMapHasDimensions(pot, costMap.numRows, costMap.numCols)
  && forall i, j | 0 <= i < costMap.numRows && 0 <= j < costMap.numCols ::
    pot[i][j].Real? ==> open(costMap, i, j)
}

predicate ValidCostMap(costMap: CostMap)
{
  forall i, j | 0 <= i < costMap.numRows && 0 <= j < costMap.numCols :: costMap.value(Point(i, j)) > 0.0
}

predicate AdjacentHorizontal(p1: Pose, p2: Pose)
{
  (p1.pos.row == p2.pos.row && p1.pos.col == p2.pos.col + 1)
  || (p1.pos.row == p2.pos.row && p1.pos.col == p2.pos.col - 1)
}

predicate AdjacentVertical(p1: Pose, p2: Pose)
{
  (p1.pos.col == p2.pos.col && p1.pos.row == p2.pos.row - 1)
  || (p1.pos.col == p2.pos.col && p1.pos.row == p2.pos.row + 1)
}

predicate Adjacent(p1: Pose, p2: Pose, costMap: CostMap)
{
  AdjacentHorizontal(p1, p2) || AdjacentVertical(p1, p2)
}

predicate HasSomeAdjacentReal(p: Pose, pot: PotentialMap, numRows: nat, numCols: nat)
  requires PotentialMapHasDimensions(pot, numRows, numCols)
  requires 0 <= p.pos.row < numRows && 0 <= p.pos.col < numCols
{
  (p.pos.row > 0 && pot[p.pos.row - 1][p.pos.col].Real?)
      || (p.pos.row + 1 < numRows && pot[p.pos.row + 1][p.pos.col].Real?)
      || (p.pos.col > 0 && pot[p.pos.row][p.pos.col - 1].Real?)
      || (p.pos.col + 1 < numCols && pot[p.pos.row][p.pos.col + 1].Real?)
}


predicate ValidQueue(queue: seq<Pose>, pot: PotentialMap, costMap: CostMap)
  requires PotentialMapHasDimensions(pot, costMap.numRows, costMap.numCols)
{
  forall p: Pose | p in queue ::
    0 <= p.pos.row < costMap.numRows
    && 0 <= p.pos.col < costMap.numCols
    && HasSomeAdjacentReal(p, pot, costMap.numRows, costMap.numCols)
}

method AStar(start: Pose, goal: Pose, costMap: CostMap, numIterations: nat) returns (path: Path)
  requires 0 <= start.pos.row < costMap.numRows && 0 <= start.pos.col < costMap.numCols
  requires 0 <= goal.pos.row < costMap.numRows && 0 <= goal.pos.col < costMap.numCols
  requires open(costMap, goal.pos.row, goal.pos.col)
  requires ValidCostMap(costMap)
{
  var pot := BuildInitialPotentialMap(costMap.numRows, costMap.numCols, goal.pos.row, goal.pos.col);
  var initialThreshold := EuclidDistance(start.pos, goal.pos);
  initialThreshold := initialThreshold + obstacleCost;
  var current := InitCurrentQueue(goal, pot, costMap);

  var pot' := AStarIteration(start, goal, costMap, pot, current, [], [], initialThreshold, numIterations);
  path := ComputePath(start, goal, pot', costMap.numRows, costMap.numCols);
}

method AStarIteration(start: Pose, goal: Pose,
                      costMap: CostMap, pot: PotentialMap,
                      current: seq<Pose>, next: seq<Pose>, excess: seq<Pose>,
                      threshold: real, numIterations: nat) returns (pot': PotentialMap)
  requires ValidPotentialMap(pot, costMap)
  requires ValidQueue(current, pot, costMap) && ValidQueue(next, pot, costMap) && ValidQueue(excess, pot, costMap)
  requires ValidCostMap(costMap)
  requires 0 <= start.pos.row < costMap.numRows && 0 <= start.pos.col < costMap.numCols
  requires 0 <= goal.pos.row < costMap.numRows && 0 <= goal.pos.col < costMap.numCols
  ensures ValidPotentialMap(pot', costMap)
  decreases numIterations, |current|
{
  if (numIterations == 0) {
    return pot;
  }

  if (pot[start.pos.row][start.pos.col] != Infinity) {
    return pot;
  }

  if (|current| > 0) {
    if (!open(costMap, current[0].pos.row, current[0].pos.col)) {
      pot' := AStarIteration(start, goal, costMap, pot, current[1..], next, excess, threshold, numIterations);
      return;
    } else {
      var minV := MinVertical(current[0], pot, costMap.numRows);
      var minH := MinHorizontal(current[0], pot, costMap.numCols);
      assert minV != Infinity || minH != Infinity;
      var min := MinInfinity(minV, minH);
      var snd := MaxInfinity(minV, minH);
      var potAux := UpdatePotential(pot, current[0], goal, costMap, min, snd);
      assert ValidQueue(current, potAux, costMap);
      assert ValidQueue(next, potAux, costMap);
      assert ValidQueue(excess, potAux, costMap);
      var next', excess' := TraverseNeighbors(current[0], start, costMap, potAux, threshold, next, excess);
      assert ValidQueue(next', potAux, costMap);
      assert ValidQueue(excess', potAux, costMap);
      pot' := AStarIteration(start, goal, costMap, potAux, current[1..], next', excess', threshold, numIterations);
    }
  } else {
    if (next == []) {
      pot' := AStarIteration(start, goal, costMap, pot, excess, [], next, threshold + 2.0 * mapCost, numIterations - 1);
    } else {
      pot' := AStarIteration(start, goal, costMap, pot, next, [], excess, threshold, numIterations - 1);
    }
  }
}

method BuildInitialPotentialMap(numRows: nat, numCols: nat, initRow: nat,  initCol: nat) returns (p: PotentialMap)
  requires 0 <= initRow < numRows && 0 <= initCol < numCols
  ensures PotentialMapHasDimensions(p, numRows, numCols)
  ensures forall i, j | 0 <= i < numRows && 0 <= j < numCols && (i != initRow || j != initCol) :: p[i][j] == Infinity
  ensures p[initRow][initCol] == Real(0.0)


method InitCurrentQueue(p: Pose, ghost pot: PotentialMap, costMap: CostMap) returns (current: seq<Pose>)
  requires PotentialMapHasDimensions(pot, costMap.numRows, costMap.numCols)
  requires 0 <= p.pos.row < costMap.numRows && 0 <= p.pos.col < costMap.numCols
  requires pot[p.pos.row][p.pos.col].Real?
  ensures ValidQueue(current, pot, costMap)
{
  var p1l :=
    if (p.pos.row > 0 && open(costMap, p.pos.row - 1, p.pos.col)) then [Pose(Point(p.pos.row - 1, p.pos.col))] else [];
  var p2l :=
    if (p.pos.row + 1 < costMap.numRows && open(costMap, p.pos.row + 1, p.pos.col)) then [Pose(Point(p.pos.row + 1, p.pos.col))] else [];
  var p3l :=
    if (p.pos.col > 0 && open(costMap, p.pos.row, p.pos.col - 1)) then [Pose(Point(p.pos.row, p.pos.col - 1))] else [];
  var p4l :=
    if (p.pos.col + 1 < costMap.numCols && open(costMap, p.pos.row, p.pos.col + 1)) then [Pose(Point(p.pos.row, p.pos.col + 1))] else [];

  current := p1l + p2l + p3l + p4l;
}

method UpdatePotential(pot: PotentialMap, p: Pose, goal: Pose, costMap: CostMap, min: RealInf, snd: RealInf) returns (pot': PotentialMap)
  requires ValidPotentialMap(pot, costMap)
  requires ValidCostMap(costMap)
  requires 0 <= p.pos.row < costMap.numRows && 0 <= p.pos.col < costMap.numCols
  requires min.Real?
  requires open(costMap, p.pos.row, p.pos.col)
  ensures ValidPotentialMap(pot', costMap)
  ensures forall i, j | 0 <= i < costMap.numRows && 0 <= j < costMap.numCols ::
    pot[i][j].Real? ==> pot'[i][j].Real?
  ensures pot'[p.pos.row][p.pos.col].Real?
{
  var hf := costMap.value(Point(p.pos.row, p.pos.col));
  var diff := Minus(snd, min);
  if (GreaterThan(diff, Real(hf))) {
    var v' := hf + min.r;
    pot' := pot[p.pos.row := pot[p.pos.row][p.pos.col := Real(v')]];
  } else {
    assert diff.Real?;
    var d := diff.r / hf;
    var interpol := -0.2301 * d * d + 0.5307 * d + 0.7040;
    var v' := min.r + hf * interpol;
    pot' := pot[p.pos.row := pot[p.pos.row][p.pos.col := Real(v')]];
  }
}

method TraverseNeighbors(p: Pose, start: Pose, costMap: CostMap, pot: PotentialMap, threshold: real, next: seq<Pose>, excess: seq<Pose>)
    returns (next': seq<Pose>, excess': seq<Pose>)
  requires ValidPotentialMap(pot, costMap)
  requires ValidQueue(next, pot, costMap) && ValidQueue(excess, pot, costMap)
  requires 0 <= p.pos.row < costMap.numRows && 0 <= p.pos.col < costMap.numCols
  requires pot[p.pos.row][p.pos.col].Real?
  ensures ValidQueue(next', pot, costMap) && ValidQueue(excess', pot, costMap)
{
  var p1l, p1r: seq<Pose>;
  if (p.pos.row > 0 && open(costMap, p.pos.row - 1, p.pos.col)) {
    p1l, p1r := TraverseNeighbor(p, Pose(Point(p.pos.row - 1, p.pos.col)), start, costMap, pot, threshold);
  } else {
    p1l := []; p1r := [];
  }

  var p2l, p2r: seq<Pose>;
  if (p.pos.row > 0 && open(costMap, p.pos.row - 1, p.pos.col)) {
    p2l, p2r := TraverseNeighbor(p, Pose(Point(p.pos.row - 1, p.pos.col)), start, costMap, pot, threshold);
  } else {
    p2l := []; p2r := [];
  }

  var p3l, p3r: seq<Pose>;
  if (p.pos.row > 0 && open(costMap, p.pos.row - 1, p.pos.col)) {
    p3l, p3r := TraverseNeighbor(p, Pose(Point(p.pos.row - 1, p.pos.col)), start, costMap, pot, threshold);
  } else {
    p3l := []; p3r := [];
  }

  var p4l, p4r: seq<Pose>;
  if (p.pos.row > 0 && open(costMap, p.pos.row - 1, p.pos.col)) {
    p4l, p4r := TraverseNeighbor(p, Pose(Point(p.pos.row - 1, p.pos.col)), start, costMap, pot, threshold);
  } else {
    p4l := []; p4r := [];
  }

  next' := next + p1l + p2l + p3l + p4l;
  excess' := excess + p1r + p2r + p3r + p4r;
}

method TraverseNeighbor(p: Pose, newP: Pose, start: Pose, costMap: CostMap, pot: PotentialMap, threshold: real) returns (next': seq<Pose>, excess': seq<Pose>)
  requires ValidPotentialMap(pot, costMap)
  requires 0 <= p.pos.row < costMap.numRows && 0 <= p.pos.col < costMap.numCols
  requires 0 <= newP.pos.row < costMap.numRows && 0 <= newP.pos.col < costMap.numCols
  requires Adjacent(p, newP, costMap)
  requires pot[p.pos.row][p.pos.col].Real?
  ensures ValidQueue(next', pot, costMap) && ValidQueue(excess', pot, costMap)
{
  var h := EuclidDistance(p.pos, start.pos);
  if (!GreaterThan(pot[newP.pos.row][newP.pos.col], Real(pot[p.pos.row][p.pos.col].r + h + (1.0 / 1.41421) * costMap.value(Point(newP.pos.row, newP.pos.col))))) {
    next' := [];
    excess' := [];
  } else {
    if (pot[p.pos.row][p.pos.col].r + h < threshold) {
      next' := [newP];
      excess' := [];
    } else {
      next' := [];
      excess' := [newP];
    }
  }
}

method ComputePath(start: Pose, goal: Pose, pot: PotentialMap, numRows: nat, numCols: nat) returns (p: Path)

function method Abs(x: int): int {
  if (x >= 0) then x else -x
}


method EuclidDistance(p1: Point, p2: Point) returns (d: real) {
  var diffRows := Abs(p1.row - p2.row);
  var diffCols := Abs(p1.col - p2.col);
  d := (diffRows * diffRows + diffCols * diffCols) as real; // TODO: sqrt?
}

function method MinHorizontal(p: Pose, pot: PotentialMap, numCols: nat): RealInf
  requires exists numRows :: PotentialMapHasDimensions(pot, numRows, numCols) && 0 <= p.pos.row < numRows
  requires 0 <= p.pos.col < numCols
{
  var left := if p.pos.col > 0 then pot[p.pos.row][p.pos.col - 1] else Infinity;
  var right := if p.pos.col + 1 < numCols then pot[p.pos.row][p.pos.col + 1] else Infinity;
  MinInfinity(left, right)
}

function method MinVertical(p: Pose, pot: PotentialMap, numRows: nat): RealInf
  requires exists numCols :: PotentialMapHasDimensions(pot, numRows, numCols) && 0 <= p.pos.col < numCols
  requires 0 <= p.pos.row < numRows
{
  var up := if p.pos.row > 0 then pot[p.pos.row - 1][p.pos.col] else Infinity;
  var down := if p.pos.row + 1 < numRows then pot[p.pos.row + 1][p.pos.col] else Infinity;
  MinInfinity(up, down)
}

function method MinInfinity(x1: RealInf, x2: RealInf): RealInf {
  if (x1.Infinity?) then x2
  else if (x2.Infinity?) then x1
  else if (x1.r <= x2.r) then x1 else x2
}

function method MaxInfinity(x1: RealInf, x2: RealInf): RealInf {
  if (x1.Infinity?) then x1
  else if (x2.Infinity?) then x2
  else if (x1.r >= x2.r) then x1 else x2
}

predicate method GreaterThan(x1: RealInf, x2: RealInf)
  requires x1.Real? || x2.Real?
{
  x1.Infinity? || (x2.Real? && x1.r > x2.r)
}

function method Minus(x1: RealInf, x2: RealInf): RealInf
  requires x2.Real?
{
  if (x1.Infinity?) then Infinity else Real(x1.r - x2.r)
}
