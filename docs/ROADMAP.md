# Roadmap: from a planar demo to a 3D cut-guide positioning robot

This document describes what it would take to turn this project into a simulation that
closely matches a **collaborative orthopaedic robot for knee arthroplasty**. In that class
of system, a 6-axis arm on a mobile cart positions a **cutting guide** on each planned bone
resection plane. An optical camera tracks markers fixed to the femur, tibia and robot. The
surgeon makes the cut with a standard saw through the guide, and arm motion is only enabled
while a foot pedal is held.

The aim is a faithful *engineering* simulation of the algorithms and safety logic. It is
**not** a medical device, it would not be validated for clinical use, and it does not
reproduce any vendor's proprietary design. The scope and numbers below are project targets,
not claims about any commercial system.

* [1. What carries over unchanged](#1-what-carries-over-unchanged)
* [2. Target system and workflow](#2-target-system-and-workflow)
* [3. Gap analysis](#3-gap-analysis)
* [4. Work packages](#4-work-packages)
* [5. Proposed requirements](#5-proposed-requirements)
* [6. Demo scenario](#6-demo-scenario)
* [7. Phasing and effort](#7-phasing-and-effort)
* [8. Non-goals](#8-non-goals)

## 1. What carries over unchanged

The core architecture was built to be dimension-agnostic where that mattered:

* **Joint-space planning.** RRT and RRT\* only see configurations, a distance metric, a
  sampler and a validity check. They work in 6D without changes.
* **The continuous-safety edge check** ([ALGORITHMS.md](ALGORITHMS.md)). The bound
  "no arm point moves more than Σ|Δθ_j|·R_j" holds in 3D, with R_j the largest distance
  from joint axis j to any point of the links beyond it. The inflate-by-m, step-by-2m
  proof is unchanged.
* **Shortcut smoothing and synchronized trapezoidal timing** (joint space).
* **Deterministic seeds, the k-d tree, the requirement and traceability process, CI and the
  warnings-as-errors build.**
* **The Qt application structure:** single source of truth (`SceneModel`), worker-thread
  planning, stale-result protection.

## 2. Target system and workflow

| Element | In the target system | In this simulation |
|---|---|---|
| Robot | 6-axis arm on a mobile base; end effector holds a cutting guide | URDF or DH model of a generic 6-axis arm with a spherical wrist; guide as a rigid tool |
| Tracking | Infrared camera; reflective marker arrays on femur, tibia, robot and pointer | Simulated tracker: marker geometry, noise, latency, frame rate, occlusion |
| Imaging | Optional pre-operative X-rays turned into a 3D bone model, or image-free landmarks | Bone meshes (synthetic or open data); landmark-only mode as an option |
| Plan | Implant size and pose, so each resection plane is known relative to the bone | Plan file: implant pose giving the distal femoral and proximal tibial planes, etc. |
| Registration | Surgeon digitizes landmarks and surfaces with a tracked pointer | Paired-point plus surface (ICP) registration with simulated noise |
| Execution | Pedal held: arm moves the guide to the plane; released: it stops. Surgeon saws through the guide | Mode state machine, pedal input, approach planning, hold with bone tracking |
| Collaboration | Surgeon can hand-guide the arm | Admittance control from a simulated force/torque sensor |

Workflow to simulate, in order:

1. Load the patient scene: bone meshes, leg and soft-tissue envelope, table, robot base
   pose, camera pose.
2. Calibrate: pointer tip (pivot calibration), and the robot flange to its marker array
   (hand-eye calibration).
3. Register: digitize landmarks and surface points, then compute T_camera←bone and
   report the registration error.
4. Plan: choose the implant pose, which defines each resection plane in bone coordinates.
5. Position: for each plane, compute a reachable guide pose, plan a collision-free
   approach, move with the pedal held, then do a final alignment under tracking.
6. Hold: keep the guide on the plane while the leg moves slightly. Compensate small
   motion; stop and alert on large motion or lost tracking.
7. Verify: report achieved vs planned plane error (angles in degrees, offset in mm).

## 3. Gap analysis

| Area | Today (planar) | Needed (3D) | Size |
|---|---|---|---|
| Math types | `Vec2`, angles | `Eigen::Isometry3d`, quaternions, SE(3) log/exp maps | S |
| Kinematics | Summed angles; 2×N Jacobian | DH/URDF chain; 6×N geometric Jacobian; tool frame | M |
| Inverse kinematics | 2-link closed form; DLS on position | Closed form for spherical wrists (up to 8 branches); DLS on a 6D pose error; singularity handling | M |
| Goal definition | Single configuration or point | **Plane goal**: a cut plane fixes 3 DOF, leaving 3 free. Choose the best guide pose within that set | M |
| Collision geometry | 2D capsules, circles, polygons | 3D capsules (segment-segment distance), convex meshes (GJK/EPA), bounding-volume tree broad phase | L |
| Workspace constraints | Obstacles only | Patient and leg envelope, table, staff zone, **camera line of sight** to every tracked array | M |
| Planners | RRT, RRT\*, A\* | RRT-Connect (bidirectional) as default in 6D; goal-region sampling; A\* removed for this robot | M |
| Final approach | Joint-space segment | Straight Cartesian move along the plane normal, with IK along the line and a check of every segment | M |
| Constrained motion | None | Planning that keeps the guide in a plane or on an axis (projection-based constrained RRT) | L |
| Trajectories | Rest-to-rest trapezoid | Add Cartesian linear moves, a tool-speed limit near the patient, and jerk-limited (S-curve) profiles | M |
| Dynamics and control | Independent joints, PID plus feedforward | Coupled dynamics (RNEA), inverse-dynamics feedforward, **admittance** hand-guiding, Cartesian servoing to a moving target | L |
| Tracking | None | Marker arrays, noise, latency, dropouts; pose estimation from markers | M |
| Registration | None | Pivot calibration, paired-point (SVD/Kabsch), ICP, hand-eye (AX = XB); error metrics | L |
| Safety logic | Executed-motion collision check | Mode state machine, pedal enable, watchdogs, deviation thresholds, fault states | M |
| Visualization | 2D QPainter | 3D OpenGL view: bones, arm, guide, planes, camera frustum; clinical readouts | L |

S = days, M = 1 to 2 weeks, L = 2 to 4 weeks, for one developer.

## 4. Work packages

### WP1. 3D kinematics (`amp::kin3`)

* Robot description from a minimal URDF (joints, limits, link frames) or DH table. Collision
  capsules per link.
* Forward kinematics to every link frame and to the tool centre point (TCP) of the guide.
* Geometric Jacobian (6×N). Manipulability, and distance to the wrist, elbow and shoulder
  singularities.
* IK:
  * closed form for a spherical-wrist 6-axis arm, returning all valid branches;
  * DLS on the 6D error `log(T_target · T_current⁻¹)`, with the existing adaptive damping;
  * branch selection by joint-limit margin, manipulability and distance from the current
    pose.
* Tests:
  * numeric vs analytic Jacobian;
  * FK–IK round trip on every branch;
  * behaviour at singular poses.

### WP2. Plane goals

A resection plane fixes the guide's slot normal (2 rotations) and offset (1 translation).
Translation within the plane (2 DOF) and rotation about the normal (1 DOF) are free within
limits. They are chosen by optimizing:

* reach and joint-limit margin;
* manipulability;
* clearance from the patient and table;
* camera visibility of the guide and bone markers;
* closeness to the current pose (short motion).

Implementation: sample the free 3 DOF, solve IK, score, then refine the best (or use DLS
with the free directions in the null space). This is the robot's main task-specific
difference from a generic point-to-point planner.

### WP3. 3D collision checking (`amp::col3`)

* Capsule–capsule and capsule–mesh distance. Convex decomposition of the guide and leg
  envelope. Bounding-volume tree for bone meshes.
* Inflate by the safety margin m and keep the continuous-safety step rule (R_j now measured
  from the 3D joint axes).
* **Line-of-sight constraint.** The segment from the camera to each required marker array
  must not intersect the arm or guide. This is checked like a collision, against a thin
  cylinder or cone. A pose that blinds the camera is invalid.
* Allowed contact: the guide is *meant* to touch bone at the end of the approach, so the
  final segment uses a separate rule ("guide may touch bone only on its contact face").

### WP4. Planning in 6D

* Add **RRT-Connect** as the default (fast in 6D); keep RRT\* for path-quality comparisons.
* Goal regions: sample goals from the WP2 set, not a single configuration.
* Two-stage motion:
  1. collision-free joint-space transit to a **pre-approach pose**, a fixed distance off the
     plane along its normal;
  2. a **Cartesian straight line** to the plane, with IK at each step and a check of every
     segment.
* **Constrained planning** (projection onto the constraint manifold) for moves that must
  keep the guide in its plane, e.g. sliding along the plane to avoid an obstacle.
* Re-planning when the target moves beyond what servoing can absorb.
* Planning-time budget with a guaranteed answer within it ("found", "not found in time",
  or "goal unreachable").
* Benchmarks on 6D scenes: success rate, time, path length, minimum clearance.

### WP5. Trajectories

* Keep synchronized joint-space trapezoids for transit.
* Cartesian linear moves with TCP speed and acceleration limits; lower speed inside a
  zone around the patient.
* Jerk-limited (S-curve) profiles; optional blending through waypoints, with the blended
  curve re-checked for collisions.

### WP6. Dynamics and control

* Coupled rigid-body dynamics M(q)q̈ + C(q,q̇)q̇ + g(q) via the recursive Newton–Euler
  algorithm, with joint friction.
* Joint control:
  * keep PID, but feedforward becomes inverse dynamics (computed torque);
  * re-run the existing finding (feedback-only tracking error vs safety margin) in 6D.
* **Admittance (hand-guiding) mode:**
  * a simulated force/torque sensor at the flange; the virtual mass-damper model is
    M_d·ẍ + D_d·ẋ = F_ext;
  * velocity and workspace limits;
  * optional virtual boundaries, e.g. motion only along the plane normal.
* **Target tracking (hold mode):**
  * the guide's target pose is expressed in the bone frame;
  * each tracker frame updates the target, and a Cartesian correction is sent through the
    Jacobian;
  * filtering and latency compensation;
  * the robot stops if the correction exceeds a threshold (bone moved too far) or tracking
    drops out.

### WP7. Tracking and registration

* Simulated optical tracker:
  * marker array geometries;
  * pose estimation from marker positions;
  * Gaussian noise (sub-millimetre RMS) and a configurable frame rate and latency;
  * occlusion computed from the scene (shared with the WP3 line-of-sight check).
* Pointer pivot calibration (least squares).
* **Paired-point registration** (Kabsch/SVD) on anatomical landmarks, then **surface
  ICP** on digitized bone points. Report fiducial registration error and target
  registration error at the planned planes.
* **Hand-eye calibration** (AX = XB) between the robot flange and its marker array.
* Tests: recover known transforms under noise; check ICP convergence from realistic
  initial errors; show error grows with noise as expected.

### WP8. Safety logic

* Explicit mode state machine:
  * `Idle → Calibration → Registration → Planning → HandGuided ⇄ AutoPositioning →
    Hold → Verify`;
  * plus `Fault` from any state.
* Arm motion only while the pedal is held; releasing it stops the arm within a bounded
  distance (computed from the dynamics and checked in tests).
* Watchdogs:
  * stale tracker data;
  * a required marker array occluded;
  * pose deviation beyond a threshold;
  * joint limit or singularity proximity;
  * excess external force;
  * planning timeout.
* Every transition and fault is logged, and the log can be exported (as the tracking CSV
  is today).
* A hazard list with each mitigation traced to a requirement and a test. This is the start
  of an ISO 14971-style risk file, which the current project lacks.

### WP9. 3D visualization and UI

* OpenGL view (Qt `QOpenGLWidget`):
  * bone meshes, arm and guide, planned and achieved planes, camera frustum;
  * marker arrays drawn with their visibility state.
* Planning screen with the standard knee readouts:
  * varus/valgus and flexion/extension angles of each cut;
  * resection depth in mm;
  * implant size.
* Status: current mode, pedal state, tracking health, registration error, live plane
  error.
* Keep the 2D views as debugging tools (joint plots, tracking error, planner tree
  statistics).

### WP10. Data and scenes

* Bone geometry: simple parametric femur/tibia shapes first, then open-licence knee models
  (for example from the Open Knee(s) project), with the licence recorded.
* Scene files extended to 3D: robot base pose, camera pose, marker arrays, bone meshes and
  their poses, implant plan, leg-motion script for the hold test.

## 5. Proposed requirements

New areas follow the existing scheme (`docs/REQUIREMENTS.md`). Each requirement gets
automated tests and appears in the traceability matrix.

| ID | Requirement (shall) |
|---|---|
| REQ-K3D-001 | FK shall return every link frame and the TCP pose for a URDF/DH chain of up to 7 joints. |
| REQ-K3D-002 | The analytic Jacobian shall match a central-difference Jacobian to 1e-6. |
| REQ-K3D-003 | Closed-form IK shall return all valid branches; each shall reproduce the target pose to 1e-6 m and 1e-6 rad. |
| REQ-K3D-004 | DLS IK shall converge on reachable 6D targets and report failure (not a wrong pose) otherwise. |
| REQ-GOL-001 | Given a resection plane, the system shall return a guide pose on that plane that satisfies joint limits, collision and line-of-sight constraints, or report that none exists. |
| REQ-C3D-001 | 3D distance queries (capsule–capsule, capsule–convex) shall match reference values to 1e-9 m. |
| REQ-C3D-002 | Edge validity shall guarantee clearance of the uninflated arm along the whole segment (3D displacement bound). |
| REQ-C3D-003 | A configuration that blocks the camera's view of a required marker array shall be invalid. |
| REQ-P6D-001 | RRT-Connect shall return a valid path or "not found within budget" within the configured time budget. |
| REQ-P6D-002 | The final approach shall be a straight Cartesian line along the plane normal, checked for collisions over its whole length. |
| REQ-TRJ-001 | TCP speed shall not exceed the configured limit inside the patient zone. |
| REQ-CTL-101 | With inverse-dynamics feedforward, maximum TCP tracking error shall stay below the safety margin on all demo scenes. |
| REQ-CTL-102 | In admittance mode, TCP velocity shall stay within the configured limit for any applied force. |
| REQ-TRK-001 | In hold mode, a bone motion within the compensation threshold shall be corrected to the plane tolerance within the configured settling time. |
| REQ-TRK-002 | Bone motion beyond the threshold, or tracker data older than the configured age, shall stop the arm and enter `Fault`. |
| REQ-REG-001 | Paired-point registration shall recover a known transform to within the error predicted by the simulated noise. |
| REQ-REG-002 | ICP shall converge from initial errors up to the configured bound and report its residual. |
| REQ-SAF-001 | The arm shall move only while the enable pedal is held; on release it shall stop within the computed stopping distance. |
| REQ-SAF-002 | Every mode transition and fault shall be logged with a timestamp and reason. |

## 6. Demo scenario

"Robot-positioned distal femoral cut":

1. Load the knee scene. The 3D view shows bones, the robot, the camera and marker arrays.
2. Run calibration and registration with simulated noise. The UI reports registration
   error (target: below 1 mm at the cut planes, in simulation).
3. Load the plan. The distal femoral plane appears with its varus/valgus, flexion and
   depth readouts.
4. Hold the pedal (a key or on-screen button). The planner computes the plane goal and the
   transit and approach paths. The arm moves; the live tree and clearance are shown.
5. In hold mode, a scripted 3 mm leg motion is compensated, and the plane error stays
   within tolerance. A scripted 15 mm motion stops the arm and shows the fault.
6. Occlude the femur marker array (drag the arm into the camera's view in hand-guided
   mode): the system faults and refuses to move.
7. Report achieved vs planned plane error. Export the session log.

Project targets in simulation: plane orientation error ≤ 1°, offset ≤ 1 mm, with the
default noise model. These are engineering targets for the simulation only.

## 7. Phasing and effort

| Phase | Content | Depends on | Estimate |
|---|---|---|---|
| A | WP1 kinematics, WP3 capsule collision, basic WP9 3D view | — | 4–6 weeks |
| B | WP2 plane goals, WP4 RRT-Connect, two-stage approach, WP5 Cartesian moves | A | 3–4 weeks |
| C | WP7 tracker simulation and registration | A | 3–4 weeks |
| D | WP6 dynamics, admittance and target tracking; WP8 state machine | B, C | 4–6 weeks |
| E | Constrained planning, S-curves, demo polish, documentation and hazard list | D | 3–4 weeks |

That is about 4 to 6 months for one developer. Phase A alone already gives a convincing
3D planning demo; phases A to C cover most of the algorithmic substance.

Library choice: the current project implements its algorithms from scratch. For the 3D
version the same rule can hold for planning, IK, registration and control. Collision
checking against meshes (FCL or Coal) and rigid-body dynamics (Pinocchio) are reasonable
places to use proven libraries. Each would be recorded as a design decision with its
licence and the reason it was chosen.

## 8. Non-goals

* Clinical use, regulatory submission, or validation against real patient data.
* Reproducing any vendor's proprietary algorithms, user interface, branding or
  specifications.
* Real hardware drivers, real-time operating systems, or certified safety controllers.
  Timing is simulated.
* Soft-tissue (ligament) balancing models beyond a simple placeholder.
