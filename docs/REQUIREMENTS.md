# Software Requirements

> **Scope.** This is a demonstration project, not medical device software. The requirements
> are written in the "shall" style used in regulated software (IEC 62304) to show how a
> motion planner can be specified so that every behaviour is testable and traced to a test.

## Conventions

* Each requirement has a unique, never-reused ID: `REQ-<AREA>-<NNN>`.
* Areas: `GEN` general, `KIN` kinematics, `CSP` configuration space, `COL` collision,
  `NN` nearest-neighbour search, `PLN` planning, `PST` post-processing, `CTL` control and
  simulation, `SCN` scenarios.
* Every requirement is verified by at least one automated test. The mapping is generated
  into [TRACEABILITY.md](TRACEABILITY.md) by `tools/check_traceability.py`, and CI fails if
  any requirement is uncovered.
* Units: metres, radians, seconds, newton-metres unless stated otherwise.

---

## General

### REQ-GEN-001: Portable deterministic random numbers
The software shall generate pseudo-random numbers from an explicit 64-bit seed such that
the same seed produces a bit-identical sequence on every supported compiler and platform.

## Kinematics

### REQ-KIN-001: Arm model validation
The software shall represent a planar serial arm of 2 to 6 revolute joints with per-link
length and per-joint limits, and shall reject (by throwing `std::invalid_argument`) any
description with an unsupported joint count, a non-positive link length, mismatched limit
count, a lower limit not below its upper limit, or a negative link radius.

### REQ-KIN-002: Forward kinematics
The software shall compute the world position of every joint and of the end effector for a
given configuration, agreeing with hand-computed values to within 1e-12 m.

### REQ-KIN-003: Jacobian
The software shall compute the 2 x N end-effector position Jacobian analytically, agreeing
with a central finite-difference approximation to within 1e-8 for 2 to 6 joints, and shall
compute the manipulability measure sqrt(det(J J^T)), which is zero at singular configurations.

### REQ-KIN-004: Analytic two-link inverse kinematics
For a two-link arm and a reachable target, the software shall return the elbow-up or
elbow-down solution as requested, whose forward kinematics matches the target to within 1e-9 m.

### REQ-KIN-005: Inverse kinematics failure reporting
The inverse kinematics solvers shall report, rather than silently return, targets that are
unreachable, solutions that violate joint limits, and inputs of the wrong dimension.

### REQ-KIN-006: Iterative N-link inverse kinematics
The software shall solve end-effector position inverse kinematics for 2 to 6 joints with a
damped least squares method. With deterministic restarts enabled (10 restarts), it shall
return a configuration within joint limits whose end effector is within 1e-5 m of the target
for at least 99% of random reachable targets. For an unreachable target it shall return a
`NotConverged` status with a configuration within 1e-2 m of the closest reachable point.

### REQ-KIN-007: Inverse kinematics determinism
Given identical inputs, the iterative inverse kinematics solver shall return bit-identical
results.

### REQ-KIN-008: Angle wrapping
The software shall wrap angles to the interval (-pi, pi] and compute the shortest signed
angular difference between two angles.

## Configuration space

### REQ-CSP-001: Configuration distance metric
The software shall measure the distance between two configurations as the Euclidean norm
of the per-joint differences, using the shortest signed angular difference for continuous
joints and the plain difference for joints with limits. The distance shall be symmetric,
zero for identical configurations, and satisfy the triangle inequality.

### REQ-CSP-002: Configuration interpolation
The software shall interpolate between two configurations along the shortest path defined
by REQ-CSP-001, returning exactly the start configuration at fraction 0 and exactly the end
configuration at fraction 1.

### REQ-CSP-003: Uniform configuration sampling
The software shall sample configurations uniformly within the joint limits (the full circle
for continuous joints), deterministically from a seeded generator (REQ-GEN-001).

## Collision checking

### REQ-COL-001: Obstacle primitives
The software shall represent obstacles as circles and convex polygons. It shall accept
polygon vertices in either winding order and shall reject (by throwing
`std::invalid_argument`) polygons with fewer than three vertices, zero area, a concave
vertex, or self-intersection.

### REQ-COL-002: Capsule versus circle
The software shall detect contact between a link capsule and a circle, treating tangency
(distance equal to the sum of radii) as a collision, including contact by a rounded end cap
and full containment of either shape in the other.

### REQ-COL-003: Capsule versus convex polygon
The software shall detect contact between a link capsule and a convex polygon, treating
tangency to an edge or vertex as a collision, and shall detect a link that crosses the
polygon with both endpoints outside it or lies entirely inside it.

### REQ-COL-004: Workspace boundary
When a rectangular workspace boundary is defined, the software shall report a configuration
as invalid if any link capsule touches or leaves the boundary.

### REQ-COL-005: Self-collision
The software shall detect contact between non-adjacent link capsules and shall not report
contact between adjacent links, which share a joint. Self-collision checking shall be
switchable off.

### REQ-COL-006: Configuration validity
The software shall report a configuration as valid only if it is within joint limits and
free of obstacle contact, boundary violation and self-collision, with every link capsule
inflated by a configurable safety margin. Each invalid result shall state its cause, the
offending link, and the obstacle index or second link where applicable. The software shall
reject collision options that cannot provide the guarantee of REQ-COL-007.

### REQ-COL-007: Edge validity with continuous safety guarantee
The software shall check a straight-line motion between two configurations by sampling it
at a number of steps derived from an upper bound on the workspace displacement of any arm
point, such that no point moves more than twice the safety margin between consecutive
samples. An edge reported valid shall therefore be free of collision for the uninflated
arm at every intermediate configuration, including against obstacles thinner than the step.

### REQ-COL-008: Two-dimensional C-space occupancy
For a 2-DOF arm, the software shall compute the validity status of every cell centre of a
regular grid over [-pi, pi)^2, identical to the pointwise result of REQ-COL-006.

## Nearest-neighbour search

### REQ-NN-001: Nearest-neighbour equivalence
The software shall provide brute-force and k-d tree nearest-neighbour indexes over
configurations, using the distance of REQ-CSP-001 including wraparound. For every query both
shall return the same nearest point and the same set of points within a radius, resolving
ties by the lowest insertion index, so that the choice of index never changes a plan.

## Planning

### REQ-PLN-001: Common planner interface and result
Every planner shall implement `plan(start, goal, params)` and return a result that contains a
success flag, a human-readable message, the path, the path cost (C-space path length), the
number of iterations, the number of tree nodes, the planning time and the search tree.

### REQ-PLN-002: Path validity
A successful plan shall start exactly at the start configuration, end exactly at the goal
configuration, and consist only of edges that are valid according to REQ-COL-007. The
reported cost shall equal the C-space length of the returned path.

### REQ-PLN-003: Deterministic planning
Given the same collision checker, start, goal and parameters (including the seed), and
without a wall-clock time limit, every planner shall return a bit-identical result. The
choice of nearest-neighbour index shall not affect the result.

### REQ-PLN-004: RRT
The software shall provide an RRT planner with a configurable goal bias, step size,
iteration budget and goal tolerance. Every tree edge shall be no longer than the step size,
every node's cost shall equal its parent's cost plus the edge length, and the planner shall
report failure with a message when the budget is exhausted.

### REQ-PLN-005: RRT*
The software shall provide an RRT* planner with choose-parent and rewiring over a neighbour
radius r(n) = min(gamma (log n / n)^(1/d), r_max), with gamma no smaller than the theoretical
bound 2 (1 + 1/d)^(1/d) (mu(C) / zeta_d)^(1/d). It shall continue improving after the first
solution until its budget is spent, record the solution cost each time it improves (never
increasing), and keep every node's cost equal to its parent's cost plus the edge length.

### REQ-PLN-006: Grid A*
The software shall provide an A* planner on a uniform joint-space grid, 8-connected for
2 DOF and 26-connected for 3 DOF, with an admissible and consistent heuristic, returning a
minimum-cost path on the grid when one exists.

### REQ-PLN-007: A* dimensionality limit
The A* planner shall refuse arms with more than 3 joints, returning a failure message that
explains the exponential growth of the grid.

### REQ-PLN-008: Endpoint validation
Every planner shall check the start and goal configurations before planning and report an
invalid or wrongly sized start or goal as a failure without searching.

### REQ-PLN-009: Progress and cancellation
Planners shall report progress (and, optionally, snapshots of the growing search tree)
through callbacks and shall stop at the next iteration when an optional cancellation flag is
set, returning a result that says so.

## Scenarios

### REQ-SCN-001: Scenario file format
The software shall load planning scenarios from JSON files describing the arm (angles in
degrees), obstacles, workspace bounds, collision options, start configuration, goal (as joint
angles, or as an end-effector position resolved by collision-aware inverse kinematics) and
planner parameters. It shall reject malformed files with an error that names the offending
field.

### REQ-SCN-002: Shipped scenarios
The software shall ship at least five scenarios, including open space, a narrow passage, a
cluttered field, a case where grid A* fails at 3 DOF because of coarse resolution, and a
narrow-corridor reaching task. Each shall have a valid start and goal and be solvable by RRT
with its own parameters.

## Post-processing

### REQ-PST-001: Shortcut smoothing
The software shall shorten a valid path by deterministic random shortcuts. The result shall
keep the same start and end configurations, consist only of valid edges (REQ-COL-007), and
never be longer than the input.

### REQ-PST-002: Synchronized trapezoidal time parameterization
The software shall convert a path into a time-parameterized trajectory in which every
segment follows a trapezoidal (or, for short segments, triangular) velocity profile shared by
all joints. Each joint shall respect its own velocity and acceleration limits, all joints
shall start and stop each segment together, the arm shall remain on the straight C-space
segment between waypoints, the most constrained joint shall reach its limit, and the
reference shall be at rest at and after the end of the trajectory.

## Control and simulation

### REQ-CTL-001: Joint dynamics model
The software shall simulate each joint as a rotational inertia with viscous friction, with
optional gravity (off by default), integrated with semi-implicit Euler at a fixed time step
with zero-order-hold torque. Free motion shall match the analytic solution of the model.

### REQ-CTL-002: PID control
The software shall provide a per-joint PID controller with proportional, integral and
derivative terms whose step response on the default joint settles within 2.5 s to within 2%
with less than 15% overshoot.

### REQ-CTL-003: Integral anti-windup
The PID integrator shall stop accumulating while the output is saturated and the error drives
it further into saturation, so that saturation does not cause overshoot to grow, and the
integral term shall remove the steady-state error caused by a constant load such as gravity.

### REQ-CTL-004: Derivative on measurement with filtering
The derivative term shall act on the measured position (plus the reference's own velocity,
when supplied) rather than on the error, so that a setpoint step produces no derivative kick,
and shall pass the measured rate through a first-order low-pass filter with a configurable
cutoff.

### REQ-CTL-005: Torque saturation
The controller output shall be limited to a configurable torque magnitude and shall report
when it is saturated.

### REQ-CTL-006: Acceleration feedforward
The software shall optionally add a feedforward torque computed from the trajectory's
acceleration and velocity with the nominal joint model, and doing so shall reduce the peak
tracking error compared with feedback alone.

### REQ-CTL-007: Tracking log
The software shall log, at a configurable interval, the reference, actual position, tracking
error and commanded torque of every joint, and report per-joint maximum, RMS and final
tracking error.

### REQ-CTL-008: Executed motion stays within the safety margin
For every shipped scenario, executing the smoothed RRT plan with the default PID gains and
acceleration feedforward shall keep the tracking error below 5 mrad on every joint, and every
logged executed configuration of the uninflated arm shall be collision-free.
