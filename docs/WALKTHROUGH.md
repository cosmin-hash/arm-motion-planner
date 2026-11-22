# Code Walkthrough

A guided tour in reading order, designed to take an afternoon (about 4 hours). Each stop
lists the file's purpose, its key types and functions, and **what to look at first**. Read
the matching section of [ALGORITHMS.md](ALGORITHMS.md) alongside each algorithm file: the
code comments point to it.

| Stop | Topic | Files | Time |
|---|---|---|---|
| 0 | Orientation | README, ARCHITECTURE, CMake | 15 min |
| 1 | Foundations | `types`, `angles`, `rng`, `arm_model` | 15 min |
| 2 | Kinematics | `kinematics` | 30 min |
| 3 | Configuration space | `cspace` | 15 min |
| 4 | Collision checking | `geometry`, `collision`, `cspace_grid` | 40 min |
| 5 | Planning | `planner`, `nearest_neighbor`, `rrt`, `rrt_star`, `astar` | 60 min |
| 6 | Scenarios | `scenario`, `scenarios/*.json` | 10 min |
| 7 | Post-processing | `smoothing`, `trajectory` | 20 min |
| 8 | Control | `dynamics`, `pid`, `simulation` | 30 min |
| 9 | Verification | `tests/`, `tools/check_traceability.py`, `bench/` | 20 min |
| 10 | GUI | `app/` | 30 min |

A good habit while reading: run the GUI next to the code. Most core functions have a
visible effect (drag the end effector for IK, drag an obstacle for the C-space image, press
F5 and F7 for planning and execution).

---

## Stop 0: Orientation

* **[README.md](../README.md):** what the project does, how to build and run it.
* **[ARCHITECTURE.md](ARCHITECTURE.md):** read the module diagram and the "scenario to
  plan" data flow. Everything below follows that flow.
* **[CMakeLists.txt](../CMakeLists.txt)** and
  **[cmake/Dependencies.cmake](../cmake/Dependencies.cmake):** three targets matter:
  `amp_core` (the library, no Qt), `amp_tests` and `amp_app`. Note that the GUI is optional
  (`find_package(Qt6 ... QUIET)`), and that Eigen is fetched as headers only and included as
  `SYSTEM`, so `-Werror` applies only to our code.
* **[cmake/CompilerWarnings.cmake](../cmake/CompilerWarnings.cmake):** the warnings policy.

## Stop 1: Foundations

* **[types.hpp](../core/include/amp/types.hpp):** `Config` (a dynamic `Eigen::VectorXd` of
  joint angles), `Vec2`, `Jacobian`. *Look at:* why `Config` is dynamic (design decision D1).
* **[angles.hpp](../core/include/amp/angles.hpp):** `wrapToPi`, `angleDiff`. The
  half-open interval (-pi, pi] gives every angle a unique representative.
* **[rng.hpp](../core/include/amp/rng.hpp):** `Rng`. *Look at first:* the comment at the
  top explaining why `std::uniform_real_distribution` is not used (cross-platform
  determinism), then `uniform01()`.
* **[arm_model.hpp](../core/include/amp/arm_model.hpp) /
  [arm_model.cpp](../core/src/arm_model.cpp):** `ArmModel`, `JointLimits`. *Look at:* the
  constructor's validation (the rest of the code relies on it), and `normalize()` (clamp
  limited joints, wrap continuous ones).

## Stop 2: Kinematics

* **[kinematics.hpp](../core/include/amp/kinematics.hpp) /
  [kinematics.cpp](../core/src/kinematics.cpp)**
  * `forwardKinematics`: the 10-line loop that everything else builds on.
  * `jacobian`: column j is the end-effector offset from joint j, rotated 90 degrees.
  * `manipulability`: sqrt(det(J J^T)), zero at singular poses.
  * `solveIkAnalytic2Link`: law of cosines, elbow up and down.
  * `solveIkDls` and `dlsAttempt`: damped least squares with adaptive damping and
    deterministic restarts.
* *Look at first:* `dlsAttempt` in kinematics.cpp. The adaptive damping block
  (`w < w0 ? ...`) is the single most important piece of this stop.
* *Tests:* [test_kinematics.cpp](../tests/test_kinematics.cpp). See
  `Jacobian.MatchesCentralFiniteDifferences` and `IkDls.RoundTripForTwoToSixLinks`.

## Stop 3: Configuration space

* **[cspace.hpp](../core/include/amp/cspace.hpp) / [cspace.cpp](../core/src/cspace.cpp)**
  * `configDifference`, `configDistance`, `interpolate`: the single place where angle
    wraparound is handled.
  * `sampleUniform`: seeded sampling within joint limits.
  * `maxWorkspaceDisplacement`: sum_j |dq_j| R_j.
* *Look at first:* `maxWorkspaceDisplacement`. It is four lines, and it is the foundation of
  the edge-checking safety argument in the next stop.

## Stop 4: Collision checking

* **[geometry.hpp](../core/include/amp/geometry.hpp) /
  [geometry.cpp](../core/src/geometry.cpp):** `Circle`, `ConvexPolygon` (validated, CCW),
  `Bounds`. Exact primitives: `pointSegmentDistance`, `segmentsIntersect`,
  `segmentSegmentDistance`, `pointInConvexPolygon`, `segmentPolygonDistance`, and the
  capsule tests built on them. *Look at:* `segmentPolygonDistance`. The "endpoint inside"
  check is what catches a link lying entirely inside an obstacle.
* **[collision.hpp](../core/include/amp/collision.hpp) /
  [collision.cpp](../core/src/collision.cpp):** `CollisionChecker`, `CollisionOptions`,
  `CollisionReport`, `Obstacle` (a `std::variant`).
  * *Look at first:* the "Safety model" comment at the top of collision.hpp, then
    `isEdgeValid` and `edgeSteps`. Then read the proof in
    [ALGORITHMS.md#edge-validity](ALGORITHMS.md#edge-validity).
  * Also note how the constructor refuses options that would void the guarantee.
* **[cspace_grid.hpp](../core/include/amp/cspace_grid.hpp):** `computeCSpaceGrid2D`, which
  feeds the GUI's C-obstacle image.
* *Tests:* [test_geometry.cpp](../tests/test_geometry.cpp) covers every edge case.
  [test_collision.cpp](../tests/test_collision.cpp) contains
  `EdgeValidity.NeverPassesThroughAThinObstacle` and the dense re-check property test.

## Stop 5: Planning

Read in this order:

1. **[planner.hpp](../core/include/amp/planner.hpp):** `Planner::plan(start, goal, params,
   callbacks)`, `PlannerParams`, `PlanResult`, `PlannerCallbacks`. *Look at:* everything in
   `PlanResult`; the GUI and benchmark use all of it.
2. **[planner_internal.hpp](../core/src/planner_internal.hpp) and
   [planner.cpp](../core/src/planner.cpp):** `PlanRun` (time budget, cancellation, progress
   and tree callbacks), `checkEndpoints`, `extractPath`, `steer`, `isPathValid`.
3. **[nearest_neighbor.hpp](../core/include/amp/nearest_neighbor.hpp) /
   [.cpp](../core/src/nearest_neighbor.cpp):** `BruteForceNN` then `KdTreeNN`. *Look at:*
   `planeLowerBound` (wraparound) and the tie-break in `better()`.
4. **[rrt.cpp](../core/src/rrt.cpp):** the main loop has four commented steps (sample,
   nearest, steer, check). It is short; read all of it.
5. **[rrt_star.cpp](../core/src/rrt_star.cpp):** *look at first:* steps 2 to 4 in the loop
   (neighbourhood, choose parent, rewire) and `propagateCostDecrease`. Then
   `minimumGamma` / `neighborRadius` against the formula in ALGORITHMS.md.
6. **[astar.cpp](../core/src/astar.cpp):** the dimensionality refusal at the top, the grid
   lambdas (`coordsOf`, `indexOf`, `centerOf`), the deterministic `OpenOrder`, and the lazy
   validity cache.

*Tests:* [test_planners.cpp](../tests/test_planners.cpp), especially
`SameSeedGivesIdenticalResults`, `NearestNeighbourStrategyDoesNotChangeTheResult`,
`CostIsNonIncreasingAndTreeCostsAreConsistent` and `OptimalOnFreeGridWithKnownAnswer`.

## Stop 6: Scenarios

* **[scenario.hpp](../core/include/amp/scenario.hpp):** the JSON format is documented in the
  header comment. `loadScenario`, `parseScenario`, `solveIkCollisionFree`.
* **[scenario.cpp](../core/src/scenario.cpp):** *look at:* how every error names the field
  (`require(...)`), and how an end-effector goal is resolved through collision-aware IK.
* **[scenarios/](../scenarios/):** each file's `description` explains what it demonstrates.
  Run `amp_bench --check` to see them diagnosed, with ASCII C-space pictures for 2 DOF.

## Stop 7: Post-processing

* **[smoothing.cpp](../core/src/smoothing.cpp):** `shortcutSmooth`. *Look at:* the
  straight-line attempt first, then the arc-length shortcut loop.
* **[trajectory.hpp](../core/include/amp/trajectory.hpp) /
  [.cpp](../core/src/trajectory.cpp):** `MotionLimits`, `Trajectory`, `SegmentTiming`,
  `Trajectory::trapezoid`.
  * *Look at first:* the header comment, which explains why all joints share one profile
    s(t) (the motion stays on the collision-checked segment).
  * Then `trapezoid()` and `sample()`. Note the explicit end-state return in `sample()`: a
    test found a real bug there: sampling at or past the end used to return a non-zero
    acceleration.

## Stop 8: Control

* **[dynamics.hpp](../core/include/amp/dynamics.hpp) /
  [.cpp](../core/src/dynamics.cpp):** `JointDynamics::defaultsFor`, `gravityTorque`,
  `stepDynamics`. *Look at:* the one-line difference that makes `stepDynamics`
  semi-implicit.
* **[pid.hpp](../core/include/amp/pid.hpp) / [pid.cpp](../core/src/pid.cpp):**
  `PidController::update`. *Look at first:* the header comment, then three blocks in order:
  the derivative filter, the D term (on measurement), and the conditional integration
  (`deepening`).
* **[simulation.cpp](../core/src/simulation.cpp):** `simulateTracking`: reference, then
  PID with optional feedforward, then plant step, then log.
* *Tests:* [test_control.cpp](../tests/test_control.cpp). The anti-windup, derivative-kick and
  feedforward tests are the clearest demonstrations of each PID feature.

## Stop 9: Verification and tooling

* **Requirement tags:** every test has `// @verifies REQ-...` above it.
  [tools/check_traceability.py](../tools/check_traceability.py) turns those tags into
  [TRACEABILITY.md](TRACEABILITY.md) and fails (as the CTest test `traceability_check`) if
  a requirement is uncovered, a tag is unknown, or the matrix is stale.
* **[test_scenario.cpp](../tests/test_scenario.cpp):** contains the end-to-end test
  `ExecutedMotionWithFeedforwardStaysCollisionFree` (plan, smooth, time, execute,
  collision-check).
* **[bench/amp_bench.cpp](../bench/amp_bench.cpp):** the 50-seed comparison and the
  nearest-neighbour benchmark (Markdown output), plus `--check` with `printAsciiCSpace`.
* **[.github/workflows/ci.yml](../.github/workflows/ci.yml):** Windows/MSVC, Ubuntu/GCC (with
  the GUI), and a clang-format check.

## Stop 10: GUI

Read in this order:

1. **[main.cpp](../app/main.cpp):** theme and command-line options.
2. **[scene_model.hpp](../app/scene_model.hpp):** *the* file to understand. It is the single
   source of truth (arm, workspace, current pose, goal, plan, motion, tracking log, playback
   time). Note how any scene change invalidates the plan.
3. **[main_window.cpp](../app/main_window.cpp):** wiring. `startPlanning` →
   `onPlanningFinished` → `processPlan` → `execute` / `simulate` → `onPlaybackTick`.
   *Look at:* the scene-revision counter that discards stale results, and the
   executed-motion collision check in `simulate`.
4. **[planning_controller.cpp](../app/planning_controller.cpp):** the worker thread. Captures
   by value, queued invokes back to the GUI thread, atomic cancel, throttled tree snapshots.
5. **[workspace_view.cpp](../app/workspace_view.cpp),
   [cspace_view.cpp](../app/cspace_view.cpp), [plot_widget.cpp](../app/plot_widget.cpp):**
   pure QPainter. The views hold no state beyond visual traces.
6. **[controls_panel.cpp](../app/controls_panel.cpp):** widgets → `PlannerParams`,
   `TrackingOptions`, `MotionLimits`.

---

## Suggested exercises (to make it stick)

1. Set a breakpoint in `RrtStarPlanner::plan` at the rewire step, plan the open-space
   scenario, and watch a node get reparented.
2. In the GUI, load the narrow passage, select RRT, and press **Step** (F6) repeatedly. The
   tree grows identically every time: that is determinism.
3. Run A\* on scenario 04 with 14 cells per joint (it fails), then raise "A\* grid cells" to
   28, 42 and 56 and note the expansions and time in the summary. Extrapolate the cell count
   to a 6-DOF arm.
4. Turn feedforward off, execute the bone-cut corridor, and read the status bar. Then raise
   Kp and Kd until the warning disappears, and watch what happens to the torque saturation.
5. Add a requirement and a test for it: the traceability check fails until the tag and the
   regenerated matrix are in place (`python tools/check_traceability.py --write`).
