# Algorithms

Written for an experienced C++ engineer who is new to motion planning and control. Each
section gives the intuition first, then the math, pseudocode, the exact source location,
complexity, and known limitations.

> Covers kinematics, configuration space, collision checking, planning, post-processing,
> dynamics and control.

**Contents**

1. [Kinematics](#kinematics): forward kinematics, Jacobian, manipulability, analytic IK, DLS IK
2. [Configuration space](#configuration-space): metric, interpolation, displacement bound
3. [Collision checking](#collision-checking): primitives, configuration validity
4. [Edge validity](#edge-validity): resolution choice and the continuous-safety proof
5. [Nearest-neighbour search](#nearest-neighbour-search): brute force and k-d tree with wraparound
6. [RRT](#rrt): probabilistic completeness
7. [RRT\*](#rrt-star): rewiring, the radius formula, asymptotic optimality
8. [A\* on a C-space grid](#a-star): admissibility, resolution completeness, curse of dimensionality
9. [Determinism](#determinism)
10. [Shortcut smoothing](#shortcut-smoothing)
11. [Time parameterization](#time-parameterization): synchronized trapezoidal profiles
12. [Joint dynamics](#joint-dynamics): model and integrator choice
13. [PID control](#pid-control): each term, derivative kick, filtering, anti-windup, feedforward

---

## Kinematics

### Forward kinematics

**Intuition.** Walk along the arm from the base. Each joint adds its angle to the running
orientation, and each link moves you forward by its length in that direction.

**Math.** With relative joint angles q and link lengths l, the absolute orientation of link i
is phi_i = q_0 + q_1 + ... + q_i, and

```
p_0     = base
p_{i+1} = p_i + l_i [cos phi_i, sin phi_i]
```

The end effector is p_N.

**Source.** `forwardKinematics`, [core/src/kinematics.cpp](../core/src/kinematics.cpp).
**Complexity.** O(N). **Verified by** REQ-KIN-002.

### Jacobian

**Intuition.** The Jacobian answers "if I nudge joint j, how does the end effector move?".
Turning joint j swings everything beyond it around p_j, so the tip moves perpendicular to
the vector from p_j to the tip, at a speed proportional to that vector's length.

**Math.** With r_j = p_ee - p_j, column j is the 90-degree rotation of r_j:

```
J(:, j) = [ -r_j.y ,  r_j.x ]^T          (2 x N matrix)
```

This is the planar case of the general rule J_j = z_j x (p_ee - p_j) for revolute joints.

**Source.** `jacobian`. **Complexity.** O(N). **Verified by** REQ-KIN-003 (against central
finite differences).

### Manipulability

w = sqrt(det(J J^T)) (Yoshikawa). It is the area of the ellipse of end-effector velocities
reachable with unit joint speeds. **w = 0 at a singularity**: when the arm is fully stretched
or folded, the tip cannot move along the arm's direction at all. Shown live in the GUI status
bar, and used by the DLS solver to decide how much damping to apply.

### Analytic inverse kinematics (2 links)

**Intuition.** The base, the elbow and the target form a triangle with known side lengths
l1, l2 and r = |target - base|. The law of cosines fixes the elbow angle. The triangle can
be mirrored across the base-to-target line, which gives the two solutions, elbow up and
elbow down.

**Math.**

```
cos q2 = (r^2 - l1^2 - l2^2) / (2 l1 l2)            unreachable if |cos q2| > 1
q2     = +/- acos(...)                               + = elbow down, - = elbow up
q1     = atan2(y, x) - atan2(l2 sin q2, l1 + l2 cos q2)
```

The `atan2` form is used instead of `acos` for q1 because it is numerically stable for every
quadrant.

**Source.** `solveIkAnalytic2Link`. **Complexity.** O(1).
**Limitations.** 2 links only. For a target exactly at full extension or full fold, the
two solutions coincide (a singularity).
**Verified by** REQ-KIN-004 and REQ-KIN-005.

### Damped least squares (DLS) inverse kinematics (N links)

**Intuition.** With 3 or more links there are infinitely many solutions, and no closed form.
Iterate instead: compute the position error e, ask the Jacobian which small joint change
dq best reduces it, apply that change, and repeat. The plain pseudo-inverse step
dq = J^+ e explodes near singularities, because J^+ has huge entries when J is nearly rank
deficient. Damping trades a little accuracy for bounded steps.

**Math.**

```
dq = J^T (J J^T + lambda^2 I)^-1 e
```

This is the minimiser of |J dq - e|^2 + lambda^2 |dq|^2: fit the error, but pay for large
joint motions. Only a 2 x 2 system is solved per iteration, whatever the DOF.

**Adaptive damping** (Nakamura and Hanafusa): lambda is zero far from singularities, which
gives the fast pseudo-inverse behaviour. It ramps up as manipulability drops below a
threshold w0:

```
lambda^2 = lambda_max^2 (1 - w / w0)^2   if w < w0,   else 0
```

**Pseudocode.**

```
q = clamp(seed)
repeat maxIterations:
    e = target - FK(q);  if |e| < tol: return Success
    J = jacobian(q);  lambda = adaptive(w(J))
    dq = J^T (J J^T + lambda^2 I)^-1 e
    dq = dq * min(1, maxStep / |dq|)        # trust-region clamp
    q  = clamp_to_limits(q + dq)
return NotConverged (best effort)
```

If the first attempt fails, optional **deterministic restarts** retry from configurations
drawn from a seeded `Rng`, so the result is still reproducible (REQ-KIN-007).

**Source.** `solveIkDls` and `dlsAttempt`. **Complexity.** O(N) per iteration.
**Limitations.** It is a local method. From a poor seed it can stall against a joint limit,
which is why restarts exist. For an unreachable target it converges slowly toward full
extension: full extension is itself a singularity, so the damping that keeps the steps
bounded also slows the final straightening (hence the 1 cm tolerance in REQ-KIN-006).

---

## Configuration space

**Why plan in C-space?** In the workspace the arm is a chain of fat line segments, which is
awkward to reason about. In configuration space (C-space) the whole arm is **a single
point** q = (q_1, ..., q_N). Obstacles become forbidden regions of that space (C-obstacles),
and planning reduces to finding a path for a point. The GUI's C-space view makes this
literal for 2 DOF: drag a circle in the workspace and watch its curved C-obstacle move.

**Metric and interpolation.** Joints with limits live on an interval. Continuous joints live
on a circle, where the shortest way from 170 deg to -170 deg is +20 deg. `configDifference`
takes the shortest signed angle on continuous joints and the plain difference elsewhere.
`configDistance` is its Euclidean norm, and `interpolate` moves along it. Everything else
(planners, edge checking) goes through these three functions, so wraparound is handled in
exactly one place.

**Source.** [core/src/cspace.cpp](../core/src/cspace.cpp).
**Verified by** REQ-CSP-001 to REQ-CSP-003.
**Limitation.** The metric weighs all joints equally, although moving the shoulder sweeps
much more workspace than moving the wrist. A weighted metric is a common refinement.

### Workspace displacement bound

`maxWorkspaceDisplacement(a, b)` bounds how far **any** point on the arm can travel while
the joints move along the straight C-space segment from a to b:

```
D(a, b) = sum_j |dq_j| * R_j,     R_j = l_j + l_{j+1} + ... + l_{N-1}
```

**Why it holds.** A point x on the arm beyond joint j moves, due to joint j alone, at speed
|dq_j/dt| * |x - p_j|, and |x - p_j| <= R_j, the length of the chain beyond joint j. Speeds
add (triangle inequality), and integrating along the segment gives the bound.
REQ-COL-007 includes a brute-force test that sums actual FK path lengths and checks them
against D.

---

## Collision checking

### Geometric model

* Each **link** is a capsule: its core segment [p_k, p_{k+1}] swept by a disc of radius r
  (link radius plus safety margin). Capsules make every test one distance query: *a capsule
  hits a shape iff dist(core segment, shape) <= r*.
* **Obstacles** are circles and convex polygons. Polygons are validated on construction
  (convex, simple, non-degenerate) and normalised to counter-clockwise order.
* An optional rectangular **boundary** must contain the whole arm.
* **Touching counts as collision.** Contact at exactly the allowed distance is reported as a
  collision, so an ambiguous case always errs toward safety.

### Primitives

| Query | Method |
|---|---|
| point to segment | Project onto the line, clamp the parameter to [0, 1], measure. |
| segment vs segment | Intersect iff each segment's endpoints lie on opposite sides of the other (orientation signs), plus collinear special cases. Otherwise the distance is the min of the four endpoint-to-segment distances (always true in 2D). |
| point in convex polygon | Inside iff it is not strictly to the right of any CCW edge. |
| segment vs polygon | 0 if either endpoint is inside (this catches a segment lying fully inside, which no edge test would), else the min distance to the edges. |
| capsule vs boundary | Both end discs strictly inside the rectangle. A capsule is the convex hull of its end discs, and the rectangle is convex. |

**Source.** [core/src/geometry.cpp](../core/src/geometry.cpp). Edge cases (tangent, touching
endpoint or vertex, crossing with both endpoints outside, fully inside, fully containing)
are tested under REQ-COL-002 and REQ-COL-003.

### Configuration validity

```
check(q):
    if any joint outside limits: return JointLimit(joint)
    pts = FK(q);  r = linkRadius + margin
    for each link k:
        for each obstacle o: if capsule_k hits o: return ObstacleContact(k, o)
        if bounds and capsule_k not inside: return Boundary(k)
    for each pair (i, j) with j >= i + 2:          # adjacent links share a joint
        if capsules i and j touch: return Self(i, j)
    return None
```

**Source.** `CollisionChecker::check`, [core/src/collision.cpp](../core/src/collision.cpp).
**Complexity.** O(N M + N^2) for N links and M obstacles (polygon tests are O(vertices)).
**Limitation.** There is no broad phase (bounding-volume hierarchy or spatial hash). With
only a handful of obstacles it is not needed. A 3D arm with meshes would use one (for
example, the BVH-based FCL library).

---

## Edge validity

**The problem.** A planner connects configurations a and b with a straight C-space segment
and needs to know whether the whole motion is safe. Only finitely many samples can be
checked. If the samples are too far apart, the arm can "tunnel" through a thin obstacle:
safe at every sample, yet passing through the wall between two of them. A fixed step in
joint space is not good enough, because the same 0.01 rad step moves the tip of a long arm
much farther than the tip of a short one.

**The solution: derive the resolution from workspace motion.** Choose a safety margin m and
check the arm **inflated** by m (capsule radius r + m). Then sample the edge so that no arm
point moves more than delta <= 2m between consecutive samples:

```
n = max(1, ceil( D(a, b) / delta ))       samples at t = k / n, k = 0..n
```

**Guarantee.** *If every sample of the inflated arm is collision-free, the real (uninflated)
arm is collision-free at every intermediate configuration, however thin the obstacle.*

**Proof.** Take any intermediate time t\*. It lies within half a step of some sample t_k, so
every point of the arm's core segments is within delta/2 <= m of where it was at t_k (by the
displacement bound). The real arm at t\* is the set of points within r of those cores.
Every such point is therefore within r + m of the cores at t_k, which is inside the inflated
capsules at t_k. Those are collision-free by assumption, so the real arm at t\* is too.
Self-collision works the same way: each of the two links moves at most delta/2, so their
separation shrinks by at most delta <= 2m, which is exactly the inflation added to the pair.
∎

**Why this design.** The guarantee holds for any obstacle thickness, including the 0.1 mm
wall in the test `EdgeValidity.NeverPassesThroughAThinObstacle`. The margin m is a physical
quantity (clearance) that a reviewer can reason about, rather than an opaque step count. The
default m = 5 mm gives delta = 1 cm. The `CollisionChecker` constructor refuses options that
would void the guarantee (delta > 2m).

**Cost.** Edge cost is proportional to the workspace distance travelled divided by delta.
Halving the margin doubles the edge-checking cost, which is the usual safety versus speed
trade-off. The property test `EdgeValidity.ValidEdgesAreCollisionFreeUnderDenseChecking`
checks the guarantee empirically. Every accepted edge is re-checked with the *uninflated*
arm at 50 times the resolution.

**Source.** `CollisionChecker::isEdgeValid` and `edgeSteps`.
**Possible improvement.** Checking samples in bisection order (midpoint first, then quarter
points, and so on) finds collisions earlier on average. That changes speed, not correctness.
**Verified by** REQ-COL-007.

---

## Nearest-neighbour search

**Intuition.** Every RRT iteration asks "which tree node is closest to this sample?". A
linear scan costs O(n) per query, so O(n^2) over a run. A k-d tree splits space with
axis-aligned planes so most of the tree can be skipped.

**Structure.** Insert-only. Node i stores point i and splits on axis (depth mod d) at that
point's coordinate: smaller values go left, others go right. No rebalancing is needed
because RRT inserts points in random order, which keeps the expected depth O(log n).

**Query.**

```
nearest(node, q):
    if node is null: return
    update best with node's point
    near, far = side containing q, other side
    nearest(near, q)
    if lowerBound(q, splitting plane) <= bestDistance: nearest(far, q)
```

**Wraparound.** On a continuous joint, the "far side" of a split at value s can also be
reached by going around the circle. The lower bound is therefore
`min(|q - s|, distance to the far side through the +/-pi seam)`. Without this, the tree
would miss neighbours across the seam. The test `FindsNeighbourAcrossTheWrapSeam` checks it.

**Ties.** Both implementations resolve equal distances to the lowest insertion index, and
radius queries return sorted ids. They therefore return *identical* answers, so swapping
strategies can never change a plan (`NearestNeighbourStrategyDoesNotChangeTheResult`).

**Source.** [core/src/nearest_neighbor.cpp](../core/src/nearest_neighbor.cpp).
**Complexity.** O(log n) expected per query in low dimensions. The advantage shrinks as d
grows (the benchmark shows this from 2 to 6 DOF), because in high dimensions almost every
cell is "close" to the query. That is another face of the curse of dimensionality.

---

## RRT

**Intuition.** Grow a tree from the start by repeatedly picking a random configuration and
taking one small step toward it from the nearest tree node. Nodes on the frontier of the
tree own large Voronoi regions (most random samples are closest to them), so the tree is
pulled into unexplored space quickly. This is the "Voronoi bias".

**Pseudocode.**

```
tree = {start}
repeat maxIterations:
    target = goal with probability goalBias, else a uniform sample
    near   = nearest(tree, target)
    new    = steer(near, target, stepSize)        # at most stepSize away
    if isEdgeValid(near, new):
        add new with parent near
        if dist(new, goal) <= goalTolerance and isEdgeValid(new, goal):
            return path(start ... new, goal)
return failure
```

**Goal bias** trades exploration for exploitation. 0 explores blindly; 1 is greedy and gets
stuck behind obstacles. A few percent is the usual choice.

**Probabilistic completeness.** If a path with positive clearance exists, the probability
that RRT has *not* found one decays to zero as the number of iterations grows. The argument:
cover the path with a chain of balls of radius below the clearance. Each ball has positive
volume, so uniform sampling eventually places a node in each, and consecutive nodes can be
connected. RRT gives **no** finite-time guarantee, and it cannot prove that no path exists:
"no path found" only means "not within this budget". The narrow-passage scenario shows the
practical consequence. The passage occupies a tiny fraction of C-space, so the expected
number of samples needed grows as the passage narrows.

**Path quality.** RRT returns the *first* path it finds, which is jagged and typically 30% to
100% longer than optimal (see the benchmark table). RRT is **not** asymptotically optimal:
with probability one it converges to a suboptimal path (Karaman and Frazzoli, 2011).

**Source.** [core/src/rrt.cpp](../core/src/rrt.cpp). **Complexity.** O(n log n) nearest
queries plus one edge check per iteration.

---

## RRT star

**Intuition.** RRT never revisits a decision: a node's parent is whatever happened to be
nearest when it was added. RRT\* fixes two things for each new node:

1. **Choose parent.** Among all nodes within radius r, connect to the one that gives the
   cheapest cost-to-come, not merely the nearest one.
2. **Rewire.** Offer the new node as a parent to every neighbour. If going through it is
   cheaper, reparent that neighbour, and its whole subtree becomes cheaper too.

Over time the tree converges toward a shortest-path tree, and the best start-to-goal path
converges to the optimum.

**The radius.** Too small and there is nothing to rewire. Too large and every iteration costs
O(n). Karaman and Frazzoli showed that

```
r(n) = gamma * (log n / n)^(1/d),    gamma > gamma* = 2 (1 + 1/d)^(1/d) (mu(C_free) / zeta_d)^(1/d)
```

(where mu is volume and zeta_d the volume of the unit d-ball) is exactly enough. The ball
around a new node is expected to contain about log n other nodes, which keeps the graph
connected "enough" for asymptotic optimality, while each iteration stays O(log n). The
(log n / n)^(1/d) term is the radius of a ball holding about log n of n uniformly scattered
points. We use the volume of the whole joint-limit box as an upper bound on mu(C_free),
which only makes the radius larger (safe), and cap it at `maxRewireRadius` so early
iterations, when n is tiny and the formula gives a huge radius, stay cheap.

**Asymptotic optimality.** With gamma above the bound, the cost of the best path converges
almost surely to the optimal cost as the number of samples goes to infinity. In practice
the RRT\* cost plot shows a fast initial drop and then slower refinement. A step plot that
stays flat for a long time is normal: improvements need samples in specific places.

**Implementation notes.** *Lazy choose-parent*: candidates are sorted by resulting cost and
edge-checked in that order, stopping at the first valid one, so usually only one or two
edges are checked. *Cost propagation*: each node keeps a child list, and rewiring subtracts
the saving from the whole subtree with an iterative DFS. The test
`CostIsNonIncreasingAndTreeCostsAreConsistent` checks the invariant
cost = parent cost + edge length for every node after rewiring.

**Source.** [core/src/rrt_star.cpp](../core/src/rrt_star.cpp).
**Complexity.** O(log n) neighbours per iteration in expectation, each possibly needing an
edge check, so RRT\* is several times slower per iteration than RRT (see the benchmark).

---

## A star

**Intuition.** Discretise C-space into a uniform grid and run the textbook best-first graph
search: always expand the open cell with the smallest f = g + h, where g is the cost so far
and h is a lower bound on the remaining cost.

**Grid.** `gridResolution` cells per joint over the joint limits (or the full circle for
continuous joints, with wraparound neighbours). Cells connect to all 3^d - 1 neighbours: 8 in
2D, 26 in 3D. Start and goal connect to the centres of their cells by straight edges, which
must themselves be valid.

**Heuristic.** h = configDistance(cell, goal cell), the straight-line C-space distance.

* *Admissible*: no sequence of grid moves can be shorter than the straight line, so A\*
  returns an optimal grid path.
* *Consistent*: h(a) <= cost(a, b) + h(b) by the triangle inequality, so each cell is expanded
  at most once and closed cells never need reopening.

`heuristicWeight` = 0 turns it into Dijkstra. The test `MatchesDijkstraCostWithObstacles...`
checks that both find the same cost while A\* expands fewer cells.

**Lazy checks.** Cell validity and edge validity are evaluated only when the search reaches
them, and an edge is checked only if it would improve the neighbour's g. Most of the grid is
never collision-checked.

**Optimal on the grid, not in C-space.** Grid paths can only turn in multiples of 45 degrees,
so even in free space an A\* path is longer than the straight line (open-space scenario:
A\* 3.38 rad versus RRT\* about 2.81 rad, with the straight line at 2.78).

**Resolution completeness.** A\* finds a path if one exists *on the grid*. If the free channel
is narrower than a cell, the grid has no path and A\* says so, even though a path exists in
the continuous space. Scenario `04_astar_coarse_grid_fails` demonstrates this, and the test
shows that 4 times the resolution succeeds.

**Curse of dimensionality.** The grid has res^d cells: 48^2 = 2,304 in 2D, 48^3 = 110,592 in
3D, but 48^6 = 1.2e10 for a 6-DOF arm. That is more memory than any computer has, before a
single collision check. Every grid-based method (A\*, Dijkstra, value iteration) hits this
wall, which is why the planner refuses d > 3 with an explanatory message, and why sampling
planners, whose cost depends on the difficulty of the problem rather than the size of the
space, dominate for real 6- and 7-DOF manipulators.

**Source.** [core/src/astar.cpp](../core/src/astar.cpp).
**Complexity.** O(N log N) for N = res^d cells in the worst case.

---

## Determinism

Same scenario and seed give a bit-identical plan (REQ-PLN-003). That makes failures
reproducible from a log line, lets the GUI's **Step** button re-run with a larger budget and
show the *same* tree growing, and makes tests exact rather than statistical. It needs four
things:

1. `Rng` converts raw `mt19937_64` bits to doubles itself, because standard distributions
   differ between standard libraries.
2. Nearest-neighbour ties break by insertion index, and radius queries return sorted ids.
3. A\*'s priority queue breaks f-ties by g, then by cell index.
4. No wall-clock time limit (it is opt-in, and documented as breaking determinism).

Scope: bit-identical results are guaranteed for the same build. Across different compilers
or standard libraries, `sin` and `cos` may differ in the last bit, and a long random search
can amplify that into a different tree. The random sequence itself is identical everywhere
(REQ-GEN-001).

---

## Shortcut smoothing

**Intuition.** RRT paths zig-zag because every edge points at a random sample. Most of those
detours are unnecessary: if two points on the path can see each other, the path between
them can be replaced by a straight edge.

**Pseudocode.**

```
if isEdgeValid(start, goal): return [start, goal]
repeat iterations:
    pick arc lengths s1 < s2 uniformly along the path
    a, b = configurations at s1, s2 (they may lie in the middle of segments)
    if a and b are on different segments and isEdgeValid(a, b):
        path = path[.. a] + [a, b] + path[b ..]
```

**Guarantees.** The endpoints never change, every edge is checked with the same
continuous-safety edge check as planning (so smoothing cannot introduce a collision), and the
cost never increases, because a straight segment is the shortest connection in the C-space
metric (triangle inequality). Picking points by arc length, not by waypoint index, lets
shortcuts cut corners in the middle of long segments.

**Limitations.** The result is shorter but still piecewise linear, with corners where the
arm must stop (see the next section). It is a local improvement, so it cannot move a path
into a different homotopy class (for example, from one side of an obstacle to the other).
Deterministic (seeded), like the planners.

**Source.** [core/src/smoothing.cpp](../core/src/smoothing.cpp). **Verified by** REQ-PST-001.

---

## Time parameterization

**Intuition.** A path says *where* to go, not *when*. Real joints have velocity and
acceleration limits. The time parameterization assigns a time to every point so that no
joint exceeds its limits, and the motion is as fast as those limits allow.

**Key design choice: synchronized, straight-line segments.** Each segment is executed as
q(t) = q_i + s(t) (q_{i+1} - q_i), with one scalar profile s(t) shared by all joints. Two
consequences:

1. The arm stays **exactly on the straight C-space segment that the collision checker
   validated**. An independent profile per joint (each joint as fast as it can) would bend
   the motion away from the checked segment, and the safety argument of
   [Edge validity](#edge-validity) would no longer apply.
2. All joints start and stop together, which is also what an operator expects to see.

**Math.** With joint displacements d_j, joint j has velocity d_j s' and acceleration d_j s''.
The joint limits therefore become limits on s:

```
s'  <= V = min_j  vmax_j / |d_j|
|s''| <= A = min_j  amax_j / |d_j|
```

The fastest rest-to-rest profile from s = 0 to 1 is a trapezoid: accelerate at A to V, cruise,
decelerate at A. The two ramps together cover V^2 / A.

```
if V^2 / A < 1:   t_a = V / A,          T = 2 t_a + (1 - V^2 / A) / V      (trapezoid)
else:             t_a = sqrt(1 / A),    T = 2 t_a                          (triangle: V never reached)
```

The joint that sets V or A runs at its limit. The others run proportionally slower and
arrive at the same time.

**Limitations.** Rest-to-rest at every waypoint is safe but slow. Blending through waypoints
(parabolic blends, or time-optimal path parameterization such as TOPP-RA) is faster, but it
leaves the straight segments, so it would need its own collision check of the blended curve.
Velocity is continuous; acceleration jumps at the ramp boundaries (infinite jerk). A real arm
would use S-curve (jerk-limited) profiles to reduce vibration. Both are natural
extensions.

**Source.** [core/src/trajectory.cpp](../core/src/trajectory.cpp). **Verified by**
REQ-PST-002 (limits respected, joints synchronized, motion on the segment, at rest at the
end).

---

## Joint dynamics

**Model.** Each joint is a rotational inertia with viscous friction:
`I_j q''_j + b_j q'_j = tau_j + tau_gravity,j(q)`. The default inertia of joint j is that of
everything beyond it with the arm stretched out (rods of 2 kg/m, plus 0.01 kg m^2 of rotor
inertia), the worst case. The default friction is 0.5 N m s/rad. Gravity is off by default:
the arm lies on a table seen from above. When it is on, the arm hangs in a vertical plane
and each joint carries the weight of the links beyond it.

**What is simplified, and why.** A real serial arm has a coupled, configuration-dependent
mass matrix M(q), Coriolis and centrifugal terms, and Coulomb friction. Decoupling the
joints keeps the control behaviour explainable joint by joint, which is the point of the
demo. The gravity option deliberately adds a configuration-dependent load, so the effect of
the integral term is visible.

**Integrator: semi-implicit (symplectic) Euler.**

```
q'_{k+1} = q'_k + dt * q''(q_k, q'_k, tau_k)
q_{k+1}  = q_k  + dt * q'_{k+1}         <- uses the NEW velocity
```

* *Explicit Euler* (old velocity in the position update) adds energy every step, so an
  undamped oscillator spirals outward. Semi-implicit Euler is symplectic: it preserves
  oscillation energy on average, at the same cost of one evaluation per step.
* *RK4* is far more accurate per step, but evaluates the dynamics four times, and its extra
  accuracy is wasted here: the torque is a zero-order-hold controller output that is constant
  over each 1 ms step, just as on a real digital controller, so the dominant "error" is the
  sampling, not the integrator.
* At dt = 1 ms the step is far below the fastest closed-loop time constant (tens of ms), so
  the integration error is negligible. The free-decay test compares against the analytic
  solution.

**Source.** [core/src/dynamics.cpp](../core/src/dynamics.cpp). **Verified by** REQ-CTL-001.

---

## PID control

**Structure** (one loop per joint, 1 kHz):

```
e   = r - y
P   = Kp e
I  += Ki e dt          (frozen while saturated and e pushes further into saturation)
D   = Kd (r' - y'_f)   (y'_f: low-pass filtered measured rate; r': trajectory velocity)
u   = clamp(P + I + D + feedforward, -tau_max, +tau_max)
```

**What each term does** (watch the tracking-error plot while editing gains live):

| Term | Role | Too low | Too high |
|---|---|---|---|
| Kp | Stiffness: torque per radian of error. | Large, slow errors that lag behind the reference. | Oscillation, overshoot after each segment, saturation. |
| Ki | Removes steady-state error caused by constant loads. | With gravity on, a permanent droop (the error curve settles away from 0). | Slow, low-frequency overshoot and ringing; windup under saturation. |
| Kd | Damping: resists velocity error. | Ringing at the end of every segment. | Noise amplification (sensor noise times Kd), sluggish response. |
| D filter | Low-pass cutoff on the measured rate. | (Cutoff too high or 0) noise passes straight into torque. | (Cutoff too low) phase lag, so damping arrives late and can destabilise. |
| Feedforward | Provides the torque the model says is needed, I r'' + b r'. | (Off) PID must *create* an error before it pushes, so the error spikes at every acceleration ramp. | Model errors are injected directly; it does not replace feedback. |

In the plot, without feedforward the error spikes at the start and end of every segment
(where the trajectory accelerates), because feedback only reacts to error. With feedforward
those spikes nearly disappear: the remaining error comes from discretisation and
integration. The test `FeedforwardReducesTrackingError` requires at least a 2 times
reduction in peak error.

**Derivative on measurement.** Differentiating the error e = r - y would differentiate the
setpoint: a step in r produces a huge spike Kd * Delta r / dt (the "derivative kick"; with
Kd = 50 and a 1 rad step at 1 kHz, 50,000 N m). Acting on the measurement avoids it. For
trajectory tracking the reference velocity r' is known analytically and is smooth, so it is
added explicitly. That is a velocity-error D term with no kick.

**Derivative filter.** The finite-difference rate (y_k - y_{k-1}) / dt amplifies measurement
noise by 1/dt. A first-order low-pass filter,
`y'_f += alpha (y'_raw - y'_f)` with `alpha = dt / (dt + 1 / (2 pi f_c))`, removes most of it.
The test injects 1 mrad noise and checks that the filtered D term is at least 4 times
smaller.

**Anti-windup (clamping / conditional integration).** When the torque saturates, the plant
no longer responds to more control effort, but a naive integrator keeps accumulating error.
When the error finally changes sign, the oversized integral has to unwind, which causes a
large overshoot. Conditional integration freezes the integrator while the output is
saturated *and* the error would drive it further into saturation. The test compares the same
saturated step with and without anti-windup: overshoot and peak integral are both at least
halved.

**Relation to impedance and admittance control** (surgical and collaborative robots): a PD
loop around a position reference already behaves like a virtual spring (Kp) and damper (Kd)
pulling the joint toward the reference. Impedance control makes that explicit and chooses
the stiffness and damping *on purpose*, often in Cartesian space through the Jacobian
(tau = J^T (K (x_d - x) + D (x_d' - x'))), so that contact with tissue or a person produces a
bounded, predictable force rather than the maximum available torque. Admittance control is
the dual: it measures force and turns it into a motion command.

**Source.** [core/src/pid.cpp](../core/src/pid.cpp),
[core/src/simulation.cpp](../core/src/simulation.cpp). **Verified by** REQ-CTL-002 to
REQ-CTL-007.

**Tracking error and safety.** The planner proved the *reference* collision-free with margin
m. The executed arm deviates from the reference by the tracking error. As long as every point
of the arm stays within m of where the reference puts it, the real arm is still safe. The
GUI checks every logged executed pose against the uninflated arm and warns if the controller
was tuned so badly that it collided. In a real system this would be a monitored runtime
limit on tracking error.
