# Roadmap: toward a 3D orthopaedic surgical robot simulation

This document describes how this planar demo could grow into a 3D simulation of the
algorithms behind orthopaedic surgical robots: arms that help a surgeon place implants
accurately in joint replacement and similar bone procedures.

It is an engineering plan for a simulation. The result would **not** be a medical device,
it would not be validated for clinical use, and it is not modelled on any particular
commercial product. All numbers are project targets for the simulation.

* [1. Capability levels](#1-capability-levels)
* [2. What carries over unchanged](#2-what-carries-over-unchanged)
* [3. Milestone 1: guided instrument positioning in 3D](#3-milestone-1-guided-instrument-positioning-in-3d)
* [4. Milestone 2: boundary-constrained tool motion](#4-milestone-2-boundary-constrained-tool-motion)
* [5. Milestone 3: supervised autonomy with learned components](#5-milestone-3-supervised-autonomy-with-learned-components)
* [6. Requirements](#6-requirements)
* [7. Demo scenarios](#7-demo-scenarios)
* [8. Phasing and effort](#8-phasing-and-effort)
* [9. Standards context and non-goals](#9-standards-context-and-non-goals)

## 1. Capability levels

Robotic assistance in orthopaedics spans a range of autonomy. This roadmap climbs it one
level at a time, and each level reuses everything below it.

| Level | Robot's role | Surgeon's role | Milestone |
|---|---|---|---|
| 0. Navigation | Tracks bones and instruments against a plan; no arm | Performs every motion | Built into M1 |
| 1. Guided positioning | Moves an instrument guide to a planned plane or axis and holds it there, following bone motion; moves only while the operator enables it | Performs the cut or drilling through the guide | **M1** |
| 2. Boundary-constrained | Lets the surgeon move the tool freely but resists or stops motion that would leave the planned volume | Moves the tool | M2 |
| 3. Supervised autonomy | Plans and executes the bone preparation itself, perceiving the anatomy and adapting in real time | Approves the plan, supervises, can stop at any time | M3 |

**Milestone 1 is the minimum target.** It is specified completely, with requirements in
section 6. Milestones 2 and 3 build on the same kinematics, collision, planning, tracking
and safety code.

## 2. What carries over unchanged

The planar project already contains the parts that do not depend on dimension:

* **Joint-space planning.** RRT and RRT\* see only configurations, a distance metric, a
  sampler and a validity check. They work in 6D or 7D without changes.
* **The continuous-safety edge check** ([ALGORITHMS.md](ALGORITHMS.md)). The bound
  "no arm point moves more than Σ|Δθ_j|·R_j" holds in 3D, with R_j the largest distance
  from joint axis j to any point of the links beyond it. The inflate-by-m, step-by-2m
  proof is unchanged.
* **Shortcut smoothing and synchronized trapezoidal timing** in joint space.
* **The key control finding:** executed motion must be checked, not only planned motion,
  because tracking error can exceed the safety margin.
* **Deterministic seeds, the k-d tree, numbered requirements with traceability, CI, and
  the warnings-as-errors build.**
* **The Qt application structure:** a single source of truth (`SceneModel`), worker-thread
  planning, and stale-result protection.

## 3. Milestone 1: guided instrument positioning in 3D

### 3.1 Scope

**Robot**
* A generic 6- or 7-axis arm with a fixed base pose relative to the operating table.
* The end effector carries an instrument guide: a cutting slot (a plane) or a drill sleeve
  (an axis).

**Tracking**
* A simulated optical tracker follows marker arrays on the bones, the robot and a pointer.

**Planning**
* An implant plan relative to the bone defines every target plane and axis.
* The bone model comes from imaging (a mesh) or from digitized landmarks alone (image-free).

**Workflow**
1. Calibrate the pointer and the robot.
2. Register the bones.
3. Plan the implant.
4. For each target: position the guide, hold it while the cut is made, and verify the cut.
5. Optionally, assess joint laxity (gap measurement through range of motion).

**Safety**
* Motion only while an operator enable device is held (a dead-man switch).
* An explicit mode state machine.
* Watchdogs on tracking, deviation, force and limits.

### 3.2 Gap analysis

| Area | Today (planar) | Needed (3D) | Size |
|---|---|---|---|
| Math types | `Vec2`, angles | `Eigen::Isometry3d`, quaternions, SE(3) log/exp maps | S |
| Kinematics | Summed angles; 2×N Jacobian | URDF/DH chain; 6×N geometric Jacobian; tool frames | M |
| Inverse kinematics | 2-link closed form; DLS on position | Closed form for spherical wrists (up to 8 branches); DLS on a 6D pose error; null space for 7 axes; singularity handling | M |
| Goal definition | One configuration or point | **Plane and axis goals**: a plane fixes 3 of 6 DOF, an axis fixes 4. Choose the best pose within the remaining freedom | M |
| Collision geometry | 2D capsules, circles, polygons | 3D capsules, convex meshes (GJK/EPA), bounding-volume tree broad phase | L |
| Workspace constraints | Obstacles only | Patient and limb envelope, table, staff zone, **tracker line of sight** | M |
| Planners | RRT, RRT\*, A\* | RRT-Connect as the 6D default; goal-region sampling; A\* not used above 3 DOF | M |
| Final approach | Joint-space segment | Straight Cartesian line along the plane normal or axis, checked over its whole length | M |
| Trajectories | Rest-to-rest trapezoid | Cartesian linear moves, a tool-speed limit near the patient, jerk-limited (S-curve) profiles | M |
| Dynamics and control | Independent joints; PID plus feedforward | Coupled dynamics (RNEA), inverse-dynamics feedforward, admittance hand-guiding, Cartesian servoing to a moving target | L |
| Tracking | None | Marker arrays, noise, latency, dropouts, occlusion; pose estimation | M |
| Registration and calibration | None | Pivot calibration, paired-point (Kabsch/SVD), ICP, hand-eye (AX = XB); error metrics | L |
| Safety logic | Executed-motion collision check | Mode state machine, enable device, watchdogs, deviation thresholds, fault handling | M |
| Visualization | 2D QPainter | 3D OpenGL scene; clinical readouts | L |

S = days, M = 1 to 2 weeks, L = 2 to 4 weeks, for one developer.

### 3.3 Work packages

**WP1. 3D kinematics**
* Minimal URDF or DH robot description: joints, limits, link frames, collision capsules.
* Forward kinematics to every link frame and to the tool centre point (TCP).
* Geometric Jacobian (6×N), manipulability, and distance to wrist, elbow and shoulder
  singularities.
* IK:
  * closed form for spherical-wrist 6-axis arms, returning all valid branches;
  * DLS on the 6D error `log(T_target · T_current⁻¹)` with the existing adaptive damping;
  * null-space secondary goals for 7-axis arms (joint-limit margin, manipulability);
  * branch selection by limit margin, manipulability and distance from the current pose.

**WP2. Plane and axis goals**
* A planar guide fixes the plane's normal (2 rotations) and offset (1 translation). The
  in-plane translation (2 DOF) and the rotation about the normal (1 DOF) are free within
  limits. A drill sleeve fixes 4 DOF and leaves 2 free.
* The free DOF are chosen by optimizing:
  * reach and joint-limit margin;
  * manipulability;
  * clearance from patient and table;
  * tracker visibility;
  * a short motion from the current pose.
* Method: sample the free DOF, solve IK, score, refine the best. Alternatively, run DLS
  with the free directions in the null space.

**WP3. 3D collision checking**
* Capsule–capsule and capsule–convex distance; convex decomposition of tools and limb
  envelope; a bounding-volume tree for bone meshes.
* Inflate by the safety margin m and keep the continuous-safety step rule, with R_j
  measured from the 3D joint axes.
* **Line of sight:** the segment from the tracker camera to each required marker array
  must stay clear of the arm and tools. It is checked like a collision.
* **Intended contact:** at the end of the approach the guide is allowed to touch bone, but
  only on its contact face, under a separate rule.

**WP4. Planning in 6D**
* RRT-Connect as the default; RRT\* kept for path-quality comparisons.
* Goal regions: sample goals from the WP2 set, not a single configuration.
* Two-stage motion:
  1. collision-free joint-space transit to a pre-approach pose, offset along the plane
     normal or axis;
  2. a straight Cartesian approach, with IK at every step and a check of every segment.
* Re-planning when the target moves beyond what servoing can absorb.
* A planning-time budget with a definite answer: found, not found within budget, or
  unreachable.

**WP5. Trajectories**
* Synchronized joint-space trapezoids for transit (existing).
* Cartesian linear moves with TCP speed and acceleration limits, and a slower zone around
  the patient.
* Jerk-limited (S-curve) profiles. Optional waypoint blending, with the blended curve
  re-checked for collisions.

**WP6. Dynamics and control**
* Coupled rigid-body dynamics M(q)q̈ + C(q,q̇)q̇ + g(q) (recursive Newton–Euler) with
  joint friction.
* Joint control: PID with inverse-dynamics (computed-torque) feedforward. Repeat the
  executed-motion safety check in 6D.
* **Admittance hand-guiding:**
  * a simulated force/torque sensor at the flange;
  * a virtual mass-damper model, M_d·ẍ + D_d·ẋ = F_ext;
  * velocity and workspace limits.
* **Hold mode:**
  * the guide target is expressed in the bone frame, and each tracker frame updates it;
  * a Cartesian correction is sent through the Jacobian, with filtering and latency
    compensation;
  * the robot stops if the correction exceeds a threshold or tracking data goes stale.

**WP7. Tracking, calibration and registration**
* **Simulated optical tracker:**
  * marker array geometry and pose estimation;
  * Gaussian noise (sub-millimetre RMS), a configurable frame rate and latency;
  * occlusion computed from the scene (shared with the WP3 line-of-sight check).
* **Calibration:**
  * pointer pivot calibration (least squares);
  * hand-eye calibration (AX = XB) between the robot flange and its marker array.
* **Registration:**
  * paired-point registration (Kabsch/SVD) on landmarks;
  * surface ICP on digitized points;
  * fiducial and target registration error reported at the planned planes.
* **Image-free mode:** build a bone coordinate frame from digitized landmarks alone, for
  example the hip centre found by pivoting the limb, plus knee and ankle landmarks.

**WP8. Clinical workflow functions**
* **Implant planning:**
  * implant pose and size give every target plane and axis;
  * readouts: varus/valgus, flexion/extension, rotation, resection depth.
* **Cut verification:** a tracked flat probe placed on the resected surface measures the
  achieved plane against the plan.
* **Joint laxity assessment:** limb poses tracked through range of motion; gap distances
  between bone surfaces computed in extension and flexion and shown against the plan.

**WP9. Safety logic**
* Mode state machine:
  * `Idle → Calibration → Registration → Planning → HandGuided ⇄ AutoPositioning →
    Hold → Verify`;
  * plus `Fault` from any state.
* Motion only while the enable device is held. On release the arm stops within a bounded
  distance, computed from the dynamics and checked in tests.
* Watchdogs:
  * stale tracker data;
  * an occluded required array;
  * deviation beyond a threshold;
  * joint-limit or singularity proximity;
  * excess external force;
  * planning timeout.
* Every transition and fault is logged and exportable.
* A hazard list, each mitigation traced to a requirement and a test.

**WP10. 3D visualization and data**
* OpenGL view (`QOpenGLWidget`):
  * bones, arm, guide, planned and achieved planes, tracker frustum;
  * marker visibility state.
* Workflow screens for calibration, registration, planning, positioning and verification.
* Bone geometry: parametric shapes first, then open-licence models with the licence
  recorded.
* 3D scene files covering:
  * robot base pose and tracker pose;
  * arrays, meshes and the plan;
  * scripted limb motion.

## 4. Milestone 2: boundary-constrained tool motion

The surgeon holds the tool; the robot keeps it inside the planned volume.

* **Virtual boundaries:**
  * the allowed region is a plane slab for a saw, a cylinder for a drill, or a mesh for
    a burr;
  * each is defined in the bone frame and moves with the bone.
* **Constraint control:**
  * admittance control with stiffness that rises near the boundary;
  * a hard stop at the boundary;
  * passivity-based damping so that interaction stays stable.
* **Tool enable:** the cutting tool runs only while its tip is inside the allowed region
  (simulated tool on/off).
* **Constrained motion planning:** projection-based constrained RRT, for repositioning that
  keeps the tool in its plane or on its axis.
* **Material removal model:** a voxel model of the bone, updated as the tool moves. It
  shows removed volume, remaining volume, and any violation of the boundary.

## 5. Milestone 3: supervised autonomy with learned components

At this level the robot performs the bone preparation under supervision. The classical
algorithms above stay at the core. The new parts handle perception, decisions inside a
verified envelope, and monitoring.

**Architecture principle: learned components propose; verified components decide.**
* Every learned output (a registration, a segmentation, a tool path, a predicted force)
  passes through deterministic, testable checks before it can move the robot:
  * collision and continuous-safety checking;
  * boundary checking;
  * kinematic and dynamic limits;
  * consistency with the tracker.
* A rejected proposal falls back to a safe state, never to an unchecked action.

**Additions:**
* **Perception:**
  * bone surface segmentation from imaging and from intra-operative depth or ultrasound
    sensing;
  * markerless registration (learned initial alignment, then ICP refinement);
  * learned confidence estimates gate whether the result is used.
* **Autonomous tool paths:**
  * coverage paths for burring, and sweep paths for sawing, inside the M2 boundary
    volume;
  * optimized for time, tool engagement and heat;
  * every path checked against the boundary and the collision model before execution.
* **Force-aware execution:**
  * hybrid force/position control during cutting;
  * cutting-force models (physics-based, optionally learned corrections);
  * feed rate adapted to measured force;
  * a stop on force anomalies (unexpected bone density, tool binding).
* **Real-time adaptation:** continuous bone motion compensation, and replanning of the
  remaining path when anatomy shifts or registration is updated.
* **Monitoring:**
  * anomaly detection on force, tracking and motor signals;
  * an out-of-distribution check on perception inputs;
  * uncertainty thresholds that pause execution and ask for surgeon confirmation.
* **Supervision interface:**
  * plan preview with predicted outcome;
  * step-by-step approval or continuous supervision;
  * immediate stop at any time;
  * clear display of what the system is doing and why it paused.
* **ML lifecycle:**
  * versioned datasets and models;
  * training and test separation by patient;
  * documented performance on held-out data;
  * a reproducible build of every model used;
  * a defined process for updating models.

## 6. Requirements

These are proposed requirements in the style of `docs/REQUIREMENTS.md`. When implemented,
each gets automated tests and appears in the traceability matrix.

### Milestone 1

| ID | Requirement (shall) |
|---|---|
| REQ-K3D-001 | FK shall return every link frame and the TCP pose for a URDF/DH chain of up to 7 joints. |
| REQ-K3D-002 | The analytic Jacobian shall match a central-difference Jacobian to 1e-6. |
| REQ-K3D-003 | Closed-form IK shall return all valid branches, each reproducing the target pose to 1e-6 m and 1e-6 rad. |
| REQ-K3D-004 | DLS IK shall converge on reachable 6D targets and report failure, not a wrong pose, otherwise. |
| REQ-K3D-005 | For 7-axis arms, null-space motion shall not change the TCP pose by more than 1e-6 m / 1e-6 rad. |
| REQ-GOL-001 | Given a target plane, the system shall return a guide pose on it that satisfies joint limits, collision and line-of-sight constraints, or report that none exists. |
| REQ-GOL-002 | Given a target axis, the system shall return a sleeve pose on it under the same constraints, or report that none exists. |
| REQ-C3D-001 | 3D distance queries (capsule–capsule, capsule–convex) shall match reference values to 1e-9 m. |
| REQ-C3D-002 | Edge validity shall guarantee clearance of the uninflated arm along the whole segment (3D displacement bound). |
| REQ-C3D-003 | A configuration that blocks the tracker's view of a required marker array shall be invalid. |
| REQ-C3D-004 | Contact between guide and bone shall be allowed only on the guide's contact face and only during the final approach. |
| REQ-P6D-001 | RRT-Connect shall return a valid path, "not found within budget", or "unreachable" within the configured time budget. |
| REQ-P6D-002 | The final approach shall be a straight Cartesian line along the plane normal or axis, checked over its whole length. |
| REQ-TRJ-001 | TCP speed shall not exceed the configured limit inside the patient zone. |
| REQ-TRJ-002 | Jerk-limited profiles shall respect the configured velocity, acceleration and jerk limits on every joint. |
| REQ-CTL-101 | With inverse-dynamics feedforward, maximum TCP tracking error shall stay below the safety margin on all demo scenes. |
| REQ-CTL-102 | In admittance mode, TCP velocity shall stay within the configured limit for any applied force. |
| REQ-TRK-001 | In hold mode, bone motion within the compensation threshold shall be corrected to the plane tolerance within the configured settling time. |
| REQ-TRK-002 | Bone motion beyond the threshold, or tracker data older than the configured age, shall stop the arm and enter `Fault`. |
| REQ-TRK-003 | Loss of line of sight to a required array shall stop the arm within one tracker frame plus the stopping time. |
| REQ-CAL-001 | Pivot calibration shall recover a known tip offset to within the error predicted by the simulated noise. |
| REQ-CAL-002 | Hand-eye calibration shall recover a known flange-to-array transform to within the predicted error. |
| REQ-REG-001 | Paired-point registration shall recover a known transform to within the predicted error and report FRE. |
| REQ-REG-002 | ICP shall converge from initial errors up to the configured bound and report its residual and the TRE at each target. |
| REQ-REG-003 | Image-free mode shall build a bone frame from digitized landmarks and a pivot-derived joint centre. |
| REQ-WFL-001 | Planning readouts (varus/valgus, flexion/extension, rotation, resection depth) shall match independently computed values. |
| REQ-WFL-002 | Cut verification shall report achieved-vs-planned plane angle and offset from a tracked probe. |
| REQ-WFL-003 | Laxity assessment shall report gap distances in extension and flexion from tracked limb poses. |
| REQ-SAF-001 | The arm shall move only while the enable device is held; on release it shall stop within the computed stopping distance. |
| REQ-SAF-002 | Every mode transition and fault shall be logged with a timestamp and reason. |
| REQ-SAF-003 | Each hazard in the hazard list shall trace to at least one mitigation requirement with a test. |

### Milestone 2

| ID | Requirement (shall) |
|---|---|
| REQ-BND-001 | The tool tip shall not cross a virtual boundary by more than the configured tolerance under any applied force within the specified range. |
| REQ-BND-002 | Boundaries shall move with the tracked bone, within the tracker latency budget. |
| REQ-BND-003 | The simulated cutting tool shall be enabled only while its tip is inside the allowed region. |
| REQ-BND-004 | Constrained planning shall keep the tool on its plane or axis to within the configured tolerance along the whole path. |
| REQ-BND-005 | The material removal model shall report removed volume, remaining planned volume and any boundary violation. |

### Milestone 3

| ID | Requirement (shall) |
|---|---|
| REQ-AUT-001 | No learned output shall command motion without passing the collision, boundary, limit and tracker-consistency checks. |
| REQ-AUT-002 | A rejected or low-confidence proposal shall put the system in a safe paused state and request surgeon input. |
| REQ-AUT-003 | Autonomous tool paths shall stay inside the planned volume and be verified before execution. |
| REQ-AUT-004 | During cutting, force above the configured limit or an anomaly flag shall stop tool motion within the configured time. |
| REQ-AUT-005 | Remaining tool paths shall be re-planned when registration changes beyond a threshold. |
| REQ-AUT-006 | The surgeon shall be able to stop the system at any time from any state. |
| REQ-AUT-007 | Perception inputs outside the training distribution shall be flagged and shall block autonomous execution. |
| REQ-AUT-008 | Every model used shall be versioned with its training data and documented held-out performance. |

## 7. Demo scenarios

**M1: robot-positioned cut**
1. Load a knee scene: bones, robot, tracker and arrays appear in the 3D view.
2. Calibrate and register with simulated noise. Registration error is shown (target:
   below 1 mm at the target planes).
3. Load the plan. The first target plane appears with its readouts.
4. Hold the enable control. The planner chooses the guide pose, plans transit and approach,
   and the arm moves while showing clearance.
5. In hold mode:
   * a scripted 3 mm limb motion is compensated;
   * a 15 mm motion stops the arm with a fault;
   * blocking a marker array with the arm (in hand-guided mode) also faults.
6. Verify the cut with the tracked probe. Plane angle and offset error are shown. Export
   the session log.

Simulation targets: plane orientation error ≤ 1° and offset ≤ 1 mm with the default noise
model.

**M2: boundary-constrained cut.** The user drags the tool with the mouse (simulated force).
The tool stops at the boundary, the voxel bone shows removed material, and the tool turns
off outside the allowed region.

**M3: supervised autonomous preparation**
* The system proposes a tool path; the user approves it.
* The robot executes it with force feedback.
* A simulated hard spot triggers an anomaly stop.
* A simulated registration shift triggers replanning of the remaining path.

## 8. Phasing and effort

| Phase | Content | Depends on | Estimate |
|---|---|---|---|
| A | WP1 kinematics, WP3 collision, basic 3D view | — | 4–6 weeks |
| B | WP2 goals, WP4 planning, WP5 trajectories | A | 3–4 weeks |
| C | WP7 tracking, calibration and registration | A | 3–4 weeks |
| D | WP6 control, WP9 safety logic | B, C | 4–6 weeks |
| E | WP8 workflow functions, workflow screens, demo, hazard list | D | 3–4 weeks |
| F | Milestone 2 | E | 6–8 weeks |
| G | Milestone 3 | F | 3–6 months |

Milestone 1 (phases A to E) is about 4 to 6 months for one developer. Phase A alone already
gives a convincing 3D planning demo.

**Libraries**
* The current project implements its algorithms from scratch. The same rule can hold for
  planning, IK, registration and control.
* Mesh collision (FCL or Coal), rigid-body dynamics (Pinocchio) and, in M3, an ML inference
  runtime are reasonable places to use proven libraries.
* Each would be recorded as a design decision with its licence and the reason.

## 9. Standards context and non-goals

A real product in this area would be developed under:
* a software life-cycle process (IEC 62304);
* risk management (ISO 14971);
* the particular standard for robotically assisted surgical equipment (IEC 80601-2-77);
* usability engineering (IEC 62366-1);
* for learned components, regulators' guidance on machine-learning-enabled devices.

This project borrows the practices that can be shown in code (requirements, traceability,
hazard-to-test links, deterministic and reproducible builds) and claims nothing further.

Non-goals:
* clinical use, regulatory submission, or validation on real patient data;
* reproducing any vendor's algorithms, user interface, branding or specifications;
* hardware drivers, real-time operating systems, or certified safety controllers (timing
  is simulated);
* detailed soft-tissue (ligament) mechanics beyond the gap measurement in WP8.
