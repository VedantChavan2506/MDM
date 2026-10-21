# GROUP 7 — THREE-WHEEL OMNIDIRECTIONAL ROBOT
## Complete Blender Model, Kinematics, Sensor Simulation, and Obstacle Avoidance

**College Robotics Engineering Capstone Project**  
*Robot performs holonomic movement in indoor environments.*

---

## 1. Executive Summary & Root-Cause Diagnosis

### 1.1 Original Defect Diagnosis
In earlier iterations of this project, the omnidirectional robot suffered from a critical failure: **the robot traveled directly into obstacles and tunneled through walls instead of detecting and avoiding them.** Furthermore, the robot was difficult to distinguish in renders, and sensor outputs were not geometrically verified against ground truth.

Through systematic code and geometric inspection, two core root causes were identified and eliminated:
1. **Sensor Self-Intersection & Occlusion**:
   - The ultrasonic raycast origin was placed too close to or within internal sensor geometry / beam meshes. In Blender's raycasting pipeline, rays collided with non-obstacle meshes (such as the sensor PCB, transducer barrels, or the beam visualization cylinders themselves) and were prematurely discarded, preventing rays from penetrating outward to reach actual obstacle meshes.
   - *Fix*: Migrated to direct 3D mesh raycasting (`obj.ray_cast`) specifically evaluated against arena obstacle and wall objects (`Obstacle_Pillar_Front`, `Obstacle_Crate_Lateral`, `Obstacle_Partition_Wall`, `Obstacle_Corner_Pillar`, `Wall_North/South/East/West`).
2. **Absence of Geometric Swept-Path Collision Enforcement**:
   - The simulation controller previously updated the robot's world position purely based on commanded kinematics ($x_{k+1} = x_k + v_x \Delta t$) without verifying clearance against physical boundaries. If the avoidance logic reacted late or experienced overshoot, the robot penetrated through obstacle geometry.
   - *Fix*: Implemented a **Dual-Layer Safety Architecture**:
     - *Layer 1 (Proactive Sensory Avoidance)*: An ultrasonic sensor state machine (`FOLLOW_ROUTE` $\rightarrow$ `STOP` $\rightarrow$ `ASSESS_CLEARANCE` $\rightarrow$ `AVOID_OBSTACLE` $\rightarrow$ `REJOIN_ROUTE` $\rightarrow$ `BLOCKED`).
     - *Layer 2 (Swept-Path Geometric Collision Detector)*: A mathematical barrier enforcing a conservative robot footprint radius ($R_{\text{col}} = 0.42\text{ m}$) across fine sub-steps between consecutive frames, clamping motion at the obstacle boundary with **zero penetration** and stopping wheel rotation if stalled.

---

## 2. Mechanical Design & Scene Architecture

### 2.1 Mechanical Structure
- **Triangular Chassis**: Symmetrical equilateral dual-deck plate architecture ($R = 0.30\text{ m}$, $300\text{ mm}$ center-to-wheel radius) with chamfered structural corners and brass standoffs.
- **Three Omni Wheels**: Mounted exactly $120^\circ$ apart at radius $R = 0.30\text{ m}$. Each wheel features dual aluminum rim plates ($r = 0.05\text{ m}$, $50\text{ mm}$ radius) and 10 high-contrast passive barrel rollers. Rollers are aligned tangentially in the rim's plane of rotation, allowing passive radial slip while driving traction tangentially.
- **Separate Motors & Encoders**: Three high-torque DC gearmotors mounted to black aluminum L-brackets, each coupled to an optical rotary encoder disc and reader module (`Encoder_Disc_1/2/3`, `Encoder_Sensor_1/2/3`).
- **Labelled IMU**: A central 6-DOF IMU module with bold extruded 3D typography (`IMU 6-DOF`) positioned at $(0, 0, 0.16\text{ m})$.
- **Three HC-SR04 Ultrasonic Sensors**: Front ($0^\circ$), Left ($+90^\circ$), and Right ($-90^\circ$) rangefinders featuring dual acoustic transducers and dynamic visual detection beams that scale in real time and illuminate in warning red upon proximity detection.
- **Indoor Test Arena**: $10\text{ m} \times 10\text{ m}$ arena with high-contrast floor grid, perimeter containment walls with caution stripes, Start Pad, Destination Goal Pad, coordinate reference axes, and varied obstacle types (cylindrical pillar, lateral crate, and partition wall).

### 2.2 Strict Blender Collections Hierarchy
All scene elements are organized into 9 isolated, cleanly named collections:
1. `Robot_Chassis`: Base deck, top deck, standoffs, battery pack, microcontroller board, power switch.
2. `Omni_Wheels`: Dual rim discs, axles, and all 30 passive barrel rollers.
3. `Motors`: Drive motors, gearbox housings, and motor mounting brackets.
4. `Encoders`: Optical encoder discs and optocoupler reader assemblies.
5. `IMU`: Central sensor breakout board, chip package, and 3D text label (`IMU 6-DOF`).
6. `Ultrasonic_Sensors`: Front, Left, and Right HC-SR04 modules, transducer barrels, and dynamic beam meshes.
7. `Environment`: Floor slab, $10\text{ m} \times 10\text{ m}$ grid, perimeter walls, caution stripes, Start Pad, Goal Pad, coordinate axes.
8. `Obstacles`: Front cylindrical pillar ($r = 0.38\text{ m}$), lateral shipping crate, interior partition wall, corner pillar.
9. `Cameras`: Presentation view, Close-up mechanism view, Top-down orthographic view, Follow tracking camera.

---

## 3. Kinematics Mathematical Formulation

### 3.1 Robot Parameters
- Chassis Center-to-Wheel Distance: $R = 0.30\text{ m}$
- Wheel Radius: $r = 0.05\text{ m}$
- Wheel Mounting Angles:
  - Wheel 1 (Front): $\phi_1 = 0^\circ$ ($0\text{ rad}$)
  - Wheel 2 (Rear-Left): $\phi_2 = 120^\circ$ ($2\pi/3\text{ rad} \approx 2.0944\text{ rad}$)
  - Wheel 3 (Rear-Right): $\phi_3 = 240^\circ$ ($4\pi/3\text{ rad} \approx 4.1888\text{ rad}$)

### 3.2 Inverse Kinematics Formulation
Projecting body velocities $[v_x, v_y, \omega_z]^T$ onto the active traction directions:
$$v_i = -v_x\sin\phi_i + v_y\cos\phi_i + R\omega_z$$

In matrix form:
$$\begin{bmatrix} v_1 \\ v_2 \\ v_3 \end{bmatrix} = \begin{bmatrix} 0 & 1 & R \\ -\frac{\sqrt{3}}{2} & -\frac{1}{2} & R \\ \frac{\sqrt{3}}{2} & -\frac{1}{2} & R \end{bmatrix} \begin{bmatrix} v_x \\ v_y \\ \omega_z \end{bmatrix}$$

Individual wheel rotational speeds and RPMs:
$$\omega_{wi} = \frac{v_i}{r}, \quad \text{RPM}_i = \frac{\omega_{wi} \times 60}{2\pi}$$

### 3.3 Forward Kinematics Formulation
Inverting the Jacobian yields the forward kinematics for state reconstruction from wheel speeds:
$$\begin{bmatrix} v_x \\ v_y \\ \omega_z \end{bmatrix} = \begin{bmatrix} 0 & -\frac{1}{\sqrt{3}} & \frac{1}{\sqrt{3}} \\ \frac{2}{3} & -\frac{1}{3} & -\frac{1}{3} \\ \frac{1}{3R} & \frac{1}{3R} & \frac{1}{3R} \end{bmatrix} \begin{bmatrix} v_1 \\ v_2 \\ v_3 \end{bmatrix}$$

$$v_x = \frac{1}{\sqrt{3}} (v_3 - v_2), \quad v_y = \frac{2}{3} v_1 - \frac{1}{3} v_2 - \frac{1}{3} v_3, \quad \omega_z = \frac{1}{3R} (v_1 + v_2 + v_3)$$

### 3.4 Kinematic Motion Modes Verification
- **Pure Forward** ($v_x > 0, v_y = 0, \omega = 0$): $v_1 = 0$, $v_2 = -v_3$. Front wheel rolls freely on passive rollers; rear wheels drive symmetrically in opposite directions.
- **Pure Lateral Left** ($v_x = 0, v_y > 0, \omega = 0$): $v_1 = v_y$, $v_2 = v_3 = -0.5 v_y$. Front wheel drives at full speed; rear wheels counter-rotate at half speed.
- **Pure In-Place Spin** ($v_x = 0, v_y = 0, \omega \ne 0$): $v_1 = v_2 = v_3 = R\omega$. All wheels rotate at identical tangential velocities.

---

## 4. Sensor Simulation & Dual-Layer Safety

### 4.1 Sensor Pipeline
1. **Optical Encoders**: Resolution $N = 360\text{ PPR}$. Angular displacement is integrated per timestep ($\Delta \theta = \omega \Delta t$) with discrete tick accumulation $\text{Ticks} = \text{round}((\theta / 2\pi) \times 360)$. Reversing motion decrements ticks with zero drift.
2. **6-DOF IMU**: Reports ground-truth and noisy yaw angle, gyroscope rate with calibrated sensor bias ($0.003\text{ rad/s}$) and Gaussian noise, and non-inertial centripetal/Coriolis accelerations ($a_x = \dot{v}_x - v_y \omega$, $a_y = \dot{v}_y + v_x \omega$).
3. **HC-SR04 Ultrasonic Rangefinders**: Direct 3D mesh raycasting evaluates line-of-sight distance against obstacle geometry. Triggers emergency braking when front distance $\le 0.65\text{ m}$.

### 4.2 Autonomous Obstacle Avoidance State Machine
The navigation controller operates across a documented state machine:
- `FOLLOW_ROUTE`: Track active trajectory waypoint.
- `STOP`: Proactive stop when distance falls below safety threshold ($0.65\text{ m}$).
- `ASSESS_CLEARANCE`: Sample Left and Right ultrasonic sensors; determine safe evasion corridor.
- `AVOID_OBSTACLE`: Execute holonomic lateral translation ($+Y$) without rotating chassis.
- `REJOIN_ROUTE`: Resume forward advance past obstacle boundary.
- `NAVIGATE_TO_GOAL`: Holonomic diagonal approach to Destination Goal Pad.
- `GOAL_REACHED`: Park at destination.
- `BLOCKED`: If all clearances are obstructed, robot remains safely halted with zero movement.

---

## 5. 16-Stage Master Demonstration Sequence

The animation spans **857 frames (28.6 seconds at 30 FPS)**:

| Stage | Duration | $v_x\text{ (m/s)}$ | $v_y\text{ (m/s)}$ | $\omega\text{ (rad/s)}$ | State | Description |
| :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| **1** | 1.0 s | 0.00 | 0.00 | 0.00 | `INITIAL_REST` | Robot stationary at Start Pad $(0, 0)$ |
| **2** | 1.5 s | +0.40 | 0.00 | 0.00 | `FOLLOW_ROUTE` | Forward translation along $+X$ |
| **3** | 1.5 s | -0.40 | 0.00 | 0.00 | `FOLLOW_ROUTE` | Backward translation returning to Start $X$ |
| **4** | 1.5 s | 0.00 | +0.40 | 0.00 | `FOLLOW_ROUTE` | Left lateral translation along $+Y$ (pure crab walk) |
| **5** | 1.5 s | 0.00 | -0.40 | 0.00 | `FOLLOW_ROUTE` | Right lateral translation returning to Start $Y$ |
| **6** | 1.2 s | +0.25 | -0.25 | 0.00 | `FOLLOW_ROUTE` | Diagonal movement $(+X, -Y)$ |
| **7** | 1.2 s | -0.25 | +0.25 | 0.00 | `FOLLOW_ROUTE` | Diagonal return $(-X, +Y)$ to origin |
| **8** | 1.5 s | 0.00 | 0.00 | -2.09 | `FOLLOW_ROUTE` | Clockwise in-place rotation |
| **9** | 1.5 s | 0.00 | 0.00 | +2.09 | `FOLLOW_ROUTE` | Anticlockwise in-place rotation |
| **10** | 3.5 s | +0.40 | 0.00 | 0.00 | `FOLLOW_ROUTE` | Approach Front Obstacle (Pillar at $X=2.5\text{ m}$) |
| **11** | 1.0 s | 0.00 | 0.00 | 0.00 | `STOP` | Proactive stop triggered at proximity alert ($d \le 0.65\text{ m}$) |
| **12** | 1.0 s | 0.00 | 0.00 | 0.00 | `ASSESS_CLEARANCE` | Evaluate Left & Right clearance |
| **13** | 2.5 s | 0.00 | +0.40 | 0.00 | `AVOID_OBSTACLE` | Safe lateral evasion manoeuvre ($+Y$ corridor) |
| **14** | 3.2 s | +0.45 | 0.00 | 0.00 | `REJOIN_ROUTE` | Rejoin forward route past obstacle |
| **15** | 3.5 s | +0.05 | +0.50 | 0.00 | `NAVIGATE_TO_GOAL` | Navigate to Destination Goal Pad |
| **16** | 1.5 s | 0.00 | 0.00 | 0.00 | `GOAL_REACHED` | Safely parked at Goal Pad; **0 collisions** |

---

## 6. Automated Validation Test Suite

A rigorous 9-point automated test suite (`verify_test_suite.py`) executes directly inside Blender's Python environment. **All 9 master tests pass with zero failures**:

```
======================================================================
GROUP 7: EXECUTING MASTER AUTOMATED VALIDATION TEST SUITE (TESTS A - I)
======================================================================

[PASS] TEST A: KINEMATICS (Forward, Backward, Lateral, Diagonal, Rotations)
       Details: 9/9 motion modes verified. Maximum round-trip numerical error: 3.33e-16 m/s
[PASS] TEST B: COLLISION PREVENTION (Direct Frontal Approach Against Pillar)
       Details: Robot attempted X=2.20m; constrained at X=1.675m outside boundary (1.70m). 0 penetration.
[PASS] TEST C: SIDE OBSTACLES (Lateral Obstacle Boundaries Constrained)
       Details: Lateral crate stopped at Y=1.150m; Partition wall stopped at Y=-1.250m. Zero penetration.
[PASS] TEST D: BLOCKED ROBOT (Enclosed Robot Stays At Rest With Zero Tunneling)
       Details: Commanded outward movement in blocked state was rejected; robot remained completely stopped at boundary.
[PASS] TEST E: SENSOR DISTANCES (3D Raycast Geometric Accuracy & Threshold Trigger)
       Details: Distance at origin: 1.783m (expected ~1.78m). Distance at close approach: 0.534m (Hazard alert triggered).
[PASS] TEST F: ENCODERS (RPM, Cumulative Angle, 360 PPR Tick Integration & Reversal)
       Details: Forward 1.0s: 688 ticks. Reverse 1.0s: returned to 0 ticks. Exact tracking.
[PASS] TEST G: IMU (Heading, Gyro Rate With Bias/Noise, Centripetal/Coriolis Acceleration)
       Details: Measured gyro rate: 1.5023 rad/s (expected ~1.503). Measured ay: 0.585 m/s^2 (expected 0.60).
[PASS] TEST H: RESET AND REPLAY (State Re-initialization & Clean Baseline)
       Details: State, encoders, IMU, and coordinate accumulators reset to exact initial conditions.
[PASS] TEST I: FILE INTEGRITY (All Collections, Named Objects, 4 Cameras, Pads, Obstacles)
       Details: All 9 Master collections, 3 wheels, 3 motors, 3 encoders, labelled IMU, 4 obstacles, Start/Goal pads verified.

======================================================================
MASTER TEST SUITE SUMMARY: 9/9 PASSED, 0 FAILED
======================================================================
```

---

## 7. Deliverables & Artifacts

All final files are located in `d:\Downloads\MDM\Group7_OmniRobot\`:

```
Group7_OmniRobot/
├── Group7_OmniRobot.blend           # Master 3D scene (857 baked keyframes, materials, 4 cameras)
├── build_robot.py                   # Parametric geometry & arena generator
├── simulate_robot.py                # Kinematics engine, swept-path collision barrier, sensor baker
├── verify_test_suite.py             # Automated 9-test validation suite
├── render_deliverables.py           # Headless Cycles/Workbench rendering script
├── README.md                        # Master technical documentation
├── backup/                          # Pre-modification project backups
│
├── telemetry_dashboard.html         # Interactive real-time HTML5/Canvas visualization suite
├── telemetry_data.json              # 857-frame telemetry dataset (positions, speeds, sensors, states)
├── telemetry_data.js                # Standalone JavaScript dataset for CORS-free browser loading
├── telemetry_data.csv               # 857-row CSV export for MATLAB / Excel analysis
│
├── simulation_demo.mp4              # Full 28.6s 1080p demonstration video
├── screenshot_robot_closeup.png     # Macro presentation of chassis, labelled IMU, motors, encoders
├── screenshot_arena_presentation.png# 3D perspective of arena and obstacle avoidance
├── screenshot_topdown_kinematics.png# Orthographic top-down trajectory and clearance analysis
└── screenshot_obstacle_detection.png# Close approach & reactive warning beam visualization
```

---

## 8. Exact Execution Commands

### 8.1 Open the Final Blender File
```powershell
& "C:\Users\vmcha\AppData\Local\Programs\blender-4.2.0-windows-x64\blender.exe" "d:\Downloads\MDM\Group7_OmniRobot\Group7_OmniRobot.blend"
```
*Press **Spacebar** in the 3D Viewport to play the full 857-frame obstacle avoidance simulation.*

### 8.2 Rerun the Complete Pipeline
```powershell
# Step 1: Rebuild 3D Model, Arena, and Collections
& "C:\Users\vmcha\AppData\Local\Programs\blender-4.2.0-windows-x64\blender.exe" -b -P "d:\Downloads\MDM\Group7_OmniRobot\build_robot.py"

# Step 2: Rerun Simulation, Kinematics, Collision Barrier, and Telemetry Export
& "C:\Users\vmcha\AppData\Local\Programs\blender-4.2.0-windows-x64\blender.exe" -b "d:\Downloads\MDM\Group7_OmniRobot\Group7_OmniRobot.blend" -P "d:\Downloads\MDM\Group7_OmniRobot\simulate_robot.py"

# Step 3: Run Automated Test Suite (Tests A through I)
& "C:\Users\vmcha\AppData\Local\Programs\blender-4.2.0-windows-x64\blender.exe" -b "d:\Downloads\MDM\Group7_OmniRobot\Group7_OmniRobot.blend" -P "d:\Downloads\MDM\Group7_OmniRobot\verify_test_suite.py"

# Step 4: Re-render Screenshots and MP4 Video
& "C:\Users\vmcha\AppData\Local\Programs\blender-4.2.0-windows-x64\blender.exe" -b "d:\Downloads\MDM\Group7_OmniRobot\Group7_OmniRobot.blend" -P "d:\Downloads\MDM\Group7_OmniRobot\render_deliverables.py"
```

### 8.3 View the Interactive Telemetry Dashboard
Double-click `telemetry_dashboard.html` or run:
```powershell
Start-Process "d:\Downloads\MDM\Group7_OmniRobot\telemetry_dashboard.html"
```
The dashboard automatically loads `telemetry_data.js` without requiring an HTTP server or triggering browser CORS restrictions. Use the playback slider to inspect 2D position, heading, wheel RPMs, encoder ticks, IMU acceleration, and ultrasonic ranges across all 857 frames.

---

## 9. Hardware Limitations & Real-World Considerations

1. **Wheel Slip & Dead Reckoning Drift**: In real-world physical platforms, passive omni rollers experience finite micro-gaps and varying contact friction, leading to odometry drift over extended trajectories. In hardware, an Extended Kalman Filter (EKF) combining wheel encoders with IMU sensor fusion and LiDAR SLAM is recommended.
2. **Surface Texture**: Omnidirectional mobility depends on smooth, level ground surfaces (epoxy floor, polished concrete, vinyl). Carpeting or outdoor debris severely increases lateral rolling resistance.
3. **Acoustic Sensor Cones**: HC-SR04 ultrasonic rangefinders possess an approximate $15^\circ$ beam spread. At steep angles of incidence ($>45^\circ$) against flat walls, acoustic energy can reflect away, requiring multi-sensor fusion.

---
*Group 7 Robotics Engineering Project — Completed and Validated.*
