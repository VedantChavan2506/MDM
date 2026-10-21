# GROUP 7 — THREE-WHEEL OMNIDIRECTIONAL ROBOT
## Blender 3D Modelling, Kinematics, Sensor Simulation, and Indoor Demonstration

**College Robotics Engineering Capstone Project**  
*Robot performs holonomic movement in indoor environments.*

---

## 1. Project Overview

This project presents a comprehensive, physically verified 3D simulation and kinematic control system for a **Three-Wheel Omnidirectional Mobile Robot** operating in a structured indoor testing arena.

Unlike non-holonomic mobile robots (such as differential-drive or Ackermann-steering vehicles) that are constrained by non-integrable velocity constraints ($\dot{y} = \dot{x}\tan\theta$), this omnidirectional platform possesses **three degrees of freedom in the plane** $(x, y, \theta)$. It can independently and simultaneously translate in arbitrary directions ($v_x, v_y$) while independently rotating at angular velocity $\omega_z$, without requiring chassis reorientation.

### Key Capabilities
- **Holonomic Mobility**: Omnidirectional translation (forward/backward, pure sideways lateral crab walk, 45° diagonal) with concurrent or decoupled yaw rotation.
- **Parametric 3D Model**: Triangular dual-deck aluminum/acrylic chassis, 3 high-torque DC gearmotors with L-brackets, 3 optical rotary encoders, central 6-DOF IMU, 3 HC-SR04 ultrasonic sensors, battery, and controller board.
- **Symmetric 120° Omni Wheels**: Three dual-disc omni wheels equipped with 10 passive barrel rollers each, oriented tangential to the drive perimeter to allow low-friction lateral slip while transferring tangential traction.
- **Sensor Pipeline**: Wheel encoders (RPM, ticks, angular velocity), IMU (yaw, gyro rate, Coriolis-compensated accelerations), and raycast-based ultrasonic distance measurement.
- **Obstacle Detection & Avoidance**: Proactive range query against 3D arena obstacles, automatic braking at safety thresholds ($0.55\text{ m}$), and holonomic evasive maneuvering.
- **Interactive Telemetry Dashboard**: Standalone HTML5/Canvas visualization suite featuring real-time 2D arena tracking, speed dials, encoder counters, and synchronized multi-channel signal plots.

---

## 2. Coordinate System & Kinematic Formulation

### 2.1 Coordinate Conventions
1. **World Inertial Frame $\{W\}$**:
   - Origin $(0, 0, 0)$ located at the center of the indoor arena launch pad.
   - $+X_W$: Arena Forward (East, indicated by Red axis).
   - $+Y_W$: Arena Lateral (North, indicated by Green axis).
   - $+Z_W$: Vertical Up (indicated by Blue axis).
2. **Robot Body-Fixed Frame $\{B\}$**:
   - Origin $(0, 0, 0)_B$ at the geometric center of the chassis lower deck.
   - $+X_B$: Robot Forward Heading ($\theta = 0^\circ$).
   - $+Y_B$: Robot Left Lateral axis ($+90^\circ$).
   - $+Z_B$: Upward rotation axis (Yaw rotation $\dot{\theta} = \omega_z$).

### 2.2 Robot Geometric Parameters
- Chassis Radius $R = 0.25\text{ m}$ ($250\text{ mm}$ center-to-wheel contact radius).
- Wheel Radius $r = 0.04\text{ m}$ ($40\text{ mm}$ radius, $80\text{ mm}$ diameter).
- Wheel Mounting Angles $\phi_i$:
  - **Wheel 1 (Front)**: $\phi_1 = 0^\circ$ ($0\text{ rad}$)
  - **Wheel 2 (Rear-Left)**: $\phi_2 = 120^\circ$ ($2\pi/3\text{ rad} \approx 2.0944\text{ rad}$)
  - **Wheel 3 (Rear-Right)**: $\phi_3 = 240^\circ$ ($4\pi/3\text{ rad} \approx 4.1888\text{ rad}$)

Each wheel's traction axis points along the circle tangent vector $\vec{t}_i = [-\sin\phi_i, \cos\phi_i]^T$, while its passive barrel rollers roll freely along the radial axle vector $\vec{n}_i = [\cos\phi_i, \sin\phi_i]^T$.

---

### 2.3 Inverse Kinematics Formulation

The velocity of the contact point of wheel $i$ in body coordinates is given by the rigid body kinematic relationship:
$$\mathbf{v}_{wi} = \mathbf{v}_B + \boldsymbol{\omega}_z \times \mathbf{r}_i$$
where $\mathbf{r}_i = [R\cos\phi_i, R\sin\phi_i, 0]^T$ and $\boldsymbol{\omega}_z = [0, 0, \omega_z]^T$.

Projecting along the active drive direction $\vec{t}_i$:
$$v_i = \mathbf{v}_{wi} \cdot \vec{t}_i = [v_x - R\omega_z\sin\phi_i, v_y + R\omega_z\cos\phi_i] \cdot [-\sin\phi_i, \cos\phi_i]^T$$
$$v_i = -v_x\sin\phi_i + v_y\cos\phi_i + R\omega_z$$

Expressed in matrix form:
$$\begin{bmatrix} v_1 \\ v_2 \\ v_3 \end{bmatrix} = \begin{bmatrix} -\sin\phi_1 & \cos\phi_1 & R \\ -\sin\phi_2 & \cos\phi_2 & R \\ -\sin\phi_3 & \cos\phi_3 & R \end{bmatrix} \begin{bmatrix} v_x \\ v_y \\ \omega_z \end{bmatrix}$$

Substituting $\phi_1 = 0^\circ$, $\phi_2 = 120^\circ$, $\phi_3 = 240^\circ$:
$$\begin{bmatrix} v_1 \\ v_2 \\ v_3 \end{bmatrix} = \begin{bmatrix} 0 & 1 & R \\ -\frac{\sqrt{3}}{2} & -\frac{1}{2} & R \\ \frac{\sqrt{3}}{2} & -\frac{1}{2} & R \end{bmatrix} \begin{bmatrix} v_x \\ v_y \\ \omega_z \end{bmatrix}$$

Wheel angular velocities and RPMs:
$$\omega_{wi} = \frac{v_i}{r}, \quad \text{RPM}_i = \frac{\omega_{wi} \times 60}{2\pi}$$

---

### 2.4 Forward Kinematics Formulation

Inverting the $3 \times 3$ Jacobian matrix yields the analytical Forward Kinematics equations:
$$\begin{bmatrix} v_x \\ v_y \\ \omega_z \end{bmatrix} = \begin{bmatrix} 0 & -\frac{1}{\sqrt{3}} & \frac{1}{\sqrt{3}} \\ \frac{2}{3} & -\frac{1}{3} & -\frac{1}{3} \\ \frac{1}{3R} & \frac{1}{3R} & \frac{1}{3R} \end{bmatrix} \begin{bmatrix} v_1 \\ v_2 \\ v_3 \end{bmatrix}$$

This produces three decoupled equations:
$$v_x = \frac{1}{\sqrt{3}} (v_3 - v_2)$$
$$v_y = \frac{2}{3} v_1 - \frac{1}{3} v_2 - \frac{1}{3} v_3$$
$$\omega_z = \frac{1}{3R} (v_1 + v_2 + v_3)$$

### 2.5 Theoretical Verification of Motion Modes

| Motion Mode | Commanded Body Velocity | Wheel 1 Speed ($v_1$) | Wheel 2 Speed ($v_2$) | Wheel 3 Speed ($v_3$) | Physical Mechanism |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Pure Forward** | $v_x = 0.5\text{ m/s}, v_y = 0, \omega = 0$ | $0.000\text{ m/s}$ | $-0.433\text{ m/s}$ | $+0.433\text{ m/s}$ | Front wheel rollers roll passively; rear wheels rotate with equal & opposite speeds |
| **Pure Lateral (Left)** | $v_x = 0, v_y = 0.5\text{ m/s}, \omega = 0$ | $+0.500\text{ m/s}$ | $-0.250\text{ m/s}$ | $-0.250\text{ m/s}$ | Front wheel drives at full speed; rear wheels brake/drive in unison at half speed |
| **Diagonal ($+X, -Y$)** | $v_x = 0.35, v_y = -0.35, \omega = 0$ | $-0.350\text{ m/s}$ | $-0.128\text{ m/s}$ | $+0.478\text{ m/s}$ | Asymmetric vector superposition producing smooth linear translation at $-45^\circ$ |
| **Pure In-Place Spin** | $v_x = 0, v_y = 0, \omega = 1.2\text{ rad/s}$ | $+0.300\text{ m/s}$ | $+0.300\text{ m/s}$ | $+0.300\text{ m/s}$ | All 3 wheels drive forward at identical speed $R\omega$; pure zero-radius rotation |

---

## 3. Sensor Simulation Models

### 3.1 Optical Rotary Encoders
- **Resolution**: $N = 360\text{ PPR}$ (Pulses Per Revolution).
- Cumulative angle integration at step $k$:
  $$\theta_{wi}[k] = \theta_{wi}[k-1] + \omega_{wi}[k]\Delta t$$
- Discrete encoder tick count:
  $$\text{Ticks}_i[k] = \text{round}\left( \frac{\theta_{wi}[k]}{2\pi} \times N \right)$$

### 3.2 6-DOF Inertial Measurement Unit (IMU)
- Mounted at chassis center $(0, 0, z_{\text{deck}})_B$.
- **Gyroscope**: $\omega_{\text{meas}} = \omega_z + \text{bias} + \mathcal{N}(0, \sigma_{\text{gyro}}^2)$ with bias $= 0.003\text{ rad/s}$, $\sigma_{\text{gyro}} = 0.008\text{ rad/s}$.
- **Accelerometer**: Accounts for linear body frame accelerations and non-inertial rotating frame Coriolis/centripetal effects:
  $$a_{x} = \frac{dv_x}{dt} - v_y \omega_z + \mathcal{N}(0, \sigma_{\text{accel}}^2)$$
  $$a_{y} = \frac{dv_y}{dt} + v_x \omega_z + \mathcal{N}(0, \sigma_{\text{accel}}^2)$$

### 3.3 HC-SR04 Ultrasonic Rangefinders
- 3 sensors mounted at radius $0.245\text{ m}$ facing:
  - **Front**: $0^\circ$ relative to heading
  - **Left**: $+90^\circ$
  - **Right**: $-90^\circ$
- **Raycasting Engine**: Uses Blender's native BVH tree collision query `scene.ray_cast()` against obstacle geometry.
- **Range**: Minimum $0.02\text{ m}$, Maximum $2.50\text{ m}$.
- **Dynamic 3D Visualization**: Visual detection cylinders scale dynamically in the Blender viewport to contact obstacle surfaces. When distance drops below the safety threshold ($0.55\text{ m}$), the beam switches from glowing safe green (`#00e676`) to emergency warning red (`#ff1744`).

---

## 4. Programmed Indoor Demonstration Route

The 16.0-second ($480\text{ frames}$ at $30\text{ FPS}$) programmed demonstration trajectory demonstrates the full holonomic operational envelope:

```
[Phase 1: Pure Forward]  (0.0s - 2.5s)   -> vx = +0.40 m/s, vy = 0.00, w = 0.00 rad/s
[Phase 2: Pure Lateral]  (2.5s - 5.0s)   -> vx = 0.00, vy = +0.40 m/s (crab walk, fixed yaw)
[Phase 3: Diagonal]      (5.0s - 7.5s)   -> vx = +0.30 m/s, vy = -0.30 m/s
[Phase 4: In-Place Spin] (7.5s - 9.5s)   -> vx = 0.00, vy = 0.00, w = +1.20 rad/s (360° spin)
[Phase 5: Approach Obs]  (9.5s - 12.0s)  -> vx = +0.35 m/s, Front US sensor tracks obstacle distance
[Phase 6: Evade & Stop]  (12.0s - 16.0s) -> Reactive brake on d < 0.55m; holonomic lateral escape
```

---

## 5. Deliverables Directory Structure

The project workspace is organized in `Group7_OmniRobot/`:

```
Group7_OmniRobot/
│
├── Group7_OmniRobot.blend           # Complete master Blender scene with baked animations
├── build_robot.py                   # Script that generates robot model, arena, and cameras
├── simulate_robot.py                # Kinematics engine, sensor pipeline, and keyframe baker
├── render_deliverables.py           # Headless rendering script for screenshots and MP4 video
├── README.md                        # Complete technical and engineering documentation
│
├── telemetry_dashboard.html         # Interactive companion HTML5/Canvas telemetry suite
├── telemetry_data.js                # Standalone JavaScript telemetry dataset
├── telemetry_data.json              # Full 480-frame JSON telemetry log
├── telemetry_data.csv               # 480-row CSV telemetry log for MATLAB / Excel
│
├── simulation_demo.mp4              # Full 16-second 1080p 30fps animation video
├── screenshot_robot_closeup.png     # Macro presentation shot of chassis, motors, and wheels
├── screenshot_arena_presentation.png# 3D perspective presentation of robot in arena
├── screenshot_topdown_kinematics.png# Orthographic top-down analysis of trajectory
└── screenshot_obstacle_detection.png# Front ultrasonic sensor proximity detection event
```

---

## 6. How to Run and Test

### 6.1 Opening the 3D Scene in Blender
1. Launch Blender (version 4.2 LTS or later).
2. Open `Group7_OmniRobot.blend`.
3. Press **Spacebar** to play the baked simulation in the 3D viewport.
4. Switch between camera views using `Numpad 0` or the Camera selector:
   - `Camera_Closeup`: High-detail macro view of the robot mechanism.
   - `Camera_Presentation`: Angled perspective view of the entire arena.
   - `Camera_TopDown`: Orthographic plan view for trajectory evaluation.
   - `Camera_Follow`: Smooth tracking camera locked to the robot.

### 6.2 Regenerating Model and Simulation via Terminal
Run the complete pipeline headlessly using Blender's Python runtime:

```powershell
# 1. Build robot geometry, materials, and arena
blender -b -P build_robot.py

# 2. Run kinematics simulation, raycast sensors, and bake keyframes
blender -b Group7_OmniRobot.blend -P simulate_robot.py

# 3. Render all screenshots and demonstration MP4 video
blender -b Group7_OmniRobot.blend -P render_deliverables.py
```

### 6.3 Opening the Live Telemetry Dashboard
Double-click `telemetry_dashboard.html` to open it in any web browser (Google Chrome, Microsoft Edge, Firefox, Safari).
- Scrub through time using the timeline slider.
- Inspect real-time Wheel RPMs, Encoder ticks, IMU acceleration, and Ultrasonic sensor cones on the 2D arena canvas.

---

## 7. Real-World Hardware Considerations & Limitations

1. **Wheel Slippage & Non-Ideal Roller Contact**: In real hardware, passive rollers have finite gaps between consecutive rollers, creating periodic micro-vibrations and anisotropic friction. Odometry from encoders drifts over time, requiring sensor fusion (Extended Kalman Filter with IMU and LiDAR/Vision).
2. **Surface Smoothness Requirements**: Omni wheels perform well on smooth epoxy, tile, or polished concrete. Rough carpet or outdoor terrain impedes passive roller rotation, significantly degrading lateral efficiency.
3. **Acoustic Sensor Reflections**: HC-SR04 ultrasonic sensors have a conical beam angle ($\approx 15^\circ$). Specular reflection can cause false echoes or missed detections at shallow angles ($>45^\circ$) against smooth planar walls.

---
*Created by Group 7 Robotics Engineering Team.*
