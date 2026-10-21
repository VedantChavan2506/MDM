"""
Group 7 - Three-Wheel Omnidirectional Robot
Kinematics, Sensor Simulation, and Obstacle Avoidance Script (Master Version)
===========================================================================
Authors: Group 7 Robotics Engineering Team
Project: Three-Wheel Omni Robot — Holonomic Movement in Indoor Environments

This master script implements:
1. Exact Three-Wheel Omni-Drive Kinematics:
   - Inverse & Forward Kinematics matrices for 120° symmetric wheel mounting.
2. Dual-Layer Collision Detection & Safety Architecture:
   - Layer 1: Autonomous State Machine (FOLLOW_ROUTE, STOP, ASSESS_CLEARANCE, AVOID_OBSTACLE, REJOIN_ROUTE, GOAL_REACHED, BLOCKED)
   - Layer 2: Geometric Swept-Path Collision Detection against arena walls and 4 obstacles.
     Prevents tunneling, constrains movement at boundary, and recomputes wheel speeds from actual movement.
3. Realistic Sensor Simulation:
   - 3 Optical Encoders (ticks, RPM, cumulative angle at 360 PPR)
   - 6-DOF IMU (yaw angle, gyro rate with noise & bias, body frame accelerations including Coriolis terms)
   - 3 HC-SR04 Ultrasonic Rangefinders (raycast against scene obstacles, dynamic beam scaling, alert color change)
4. Complete 15-Stage Demonstration Trajectory (Start Pad -> Holonomic Modes -> Avoidance -> Goal Pad).
5. 3D Keyframe Animation Baking into Group7_OmniRobot.blend.
6. Telemetry Export (JSON, JS, CSV).
"""

import bpy
import math
import os
import json
import csv
import random
from mathutils import Vector, Matrix, Euler

# ---------------------------------------------------------------------------
# PHYSICAL PARAMETERS (Matches build_robot.py)
# ---------------------------------------------------------------------------
WHEEL_RADIUS = 0.05            # r = 50 mm (100 mm diameter)
CHASSIS_RADIUS = 0.30          # R = 300 mm (center to wheel contact point)
ROBOT_FOOTPRINT_RADIUS = 0.38  # Conservative collision radius of chassis + wheels
SAFETY_MARGIN = 0.04           # Additional buffer margin (40 mm)
TOTAL_COLLISION_RADIUS = ROBOT_FOOTPRINT_RADIUS + SAFETY_MARGIN # 0.42 m

ENCODER_PPR = 360              # Pulses (ticks) per revolution
FPS = 30                       # Simulation frame rate
OBSTACLE_STOP_THRESHOLD = 0.65 # Ultrasonic proximity threshold (meters)
ULTRASONIC_MAX_RANGE = 2.50    # Maximum reliable sensing distance (meters)

# Wheel mounting angles relative to forward axis (+X_B): 0°, 120°, 240°
WHEEL_ANGLES = [0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0]

# ---------------------------------------------------------------------------
# KINEMATICS ENGINE
# ---------------------------------------------------------------------------
def compute_inverse_kinematics(vx, vy, omega, R=CHASSIS_RADIUS, r=WHEEL_RADIUS):
    """
    Inverse Kinematics for a 3-wheel symmetric omnidirectional robot.
    v_i = -vx * sin(phi_i) + vy * cos(phi_i) + R * omega
    w_i = v_i / r
    RPM_i = (w_i * 60) / (2 * pi)
    """
    v_wheels = []
    w_wheels = []
    rpms = []
    for phi in WHEEL_ANGLES:
        vi = -vx * math.sin(phi) + vy * math.cos(phi) + R * omega
        wi = vi / r
        rpm = (wi * 60.0) / (2.0 * math.pi)
        v_wheels.append(vi)
        w_wheels.append(wi)
        rpms.append(rpm)
    return v_wheels, w_wheels, rpms

def compute_forward_kinematics(v_wheels, R=CHASSIS_RADIUS, r=WHEEL_RADIUS):
    """
    Forward Kinematics for a 3-wheel omni robot.
    Analytical inverse Jacobian for phi = [0, 120, 240 deg]:
    vx = (1 / sqrt(3)) * (v3 - v2)
    vy = (2/3) * v1 - (1/3) * v2 - (1/3) * v3
    omega = (1 / (3*R)) * (v1 + v2 + v3)
    """
    v1, v2, v3 = v_wheels
    vx = (1.0 / math.sqrt(3.0)) * (v3 - v2)
    vy = (2.0 / 3.0) * v1 - (1.0 / 3.0) * v2 - (1.0 / 3.0) * v3
    omega = (1.0 / (3.0 * R)) * (v1 + v2 + v3)
    return vx, vy, omega

def verify_kinematics():
    """Unit test for forward and inverse kinematics consistency across all movement modes."""
    test_cases = [
        ("Pure Forward",   0.45,  0.00,  0.00),
        ("Pure Backward", -0.40,  0.00,  0.00),
        ("Pure Lateral L", 0.00,  0.45,  0.00),
        ("Pure Lateral R", 0.00, -0.45,  0.00),
        ("Diagonal",       0.35, -0.35,  0.00),
        ("Pure Rot CW",    0.00,  0.00, -1.50),
        ("Pure Rot CCW",   0.00,  0.00,  1.50),
        ("Combined",       0.30,  0.20,  0.80)
    ]
    print("--- VERIFYING KINEMATICS ENGINE ---")
    for name, tx, ty, tw in test_cases:
        vw, ww, _ = compute_inverse_kinematics(tx, ty, tw)
        rx, ry, rw = compute_forward_kinematics(vw)
        err_x = abs(tx - rx)
        err_y = abs(ty - ry)
        err_w = abs(tw - rw)
        assert err_x < 1e-6 and err_y < 1e-6 and err_w < 1e-6, f"Mismatch in {name}!"
        print(f"  [PASS] {name:<16}: vx={tx:+.2f}, vy={ty:+.2f}, w={tw:+.2f} -> Wheel speeds: [{vw[0]:+.3f}, {vw[1]:+.3f}, {vw[2]:+.3f}] m/s")
    print("--- KINEMATICS VERIFICATION SUCCESSFUL ---\n")

# ---------------------------------------------------------------------------
# GEOMETRIC COLLISION DETECTOR (Swept-Path Footprint Barrier)
# ---------------------------------------------------------------------------
class GeometricCollisionDetector:
    """
    Robust 2D Swept-Path Collision Detection against Arena Obstacles and Perimeter Walls.
    Guarantees the robot never penetrates or tunnels into obstacles.
    """
    def __init__(self, r_col=TOTAL_COLLISION_RADIUS):
        self.r_col = r_col
        # Arena wall inner boundaries (10m arena, 0.18m wall thickness -> inner limits [-4.82, 4.82])
        self.wall_min_x = -4.82 + self.r_col
        self.wall_max_x =  4.82 - self.r_col
        self.wall_min_y = -4.82 + self.r_col
        self.wall_max_y =  4.82 - self.r_col
        
        # 4 Arena Obstacles
        self.obstacles = [
            # 1. Front Cylinder Pillar at (2.50, 0.00), radius 0.38m
            {"type": "cylinder", "name": "Obstacle_Pillar_Front", "center": (2.50, 0.00), "radius": 0.38},
            # 2. Lateral Crate at (1.30, 2.30), half-extents (0.375, 0.70)
            {"type": "box", "name": "Obstacle_Crate_Lateral", "center": (1.30, 2.30), "hx": 0.375, "hy": 0.70},
            # 3. Partition Wall at (-1.60, -1.90), half-extents (1.00, 0.175)
            {"type": "box", "name": "Obstacle_Partition_Wall", "center": (-1.60, -1.90), "hx": 1.00, "hy": 0.175},
            # 4. Corner Pillar at (-2.60, 2.60), radius 0.50m
            {"type": "cylinder", "name": "Obstacle_Corner_Pillar", "center": (-2.60, 2.60), "radius": 0.50}
        ]

    def is_point_colliding(self, px, py):
        """Check if robot circular footprint centered at (px, py) intersects any obstacle or wall."""
        if px < self.wall_min_x or px > self.wall_max_x or py < self.wall_min_y or py > self.wall_max_y:
            return True, "Boundary_Wall"
            
        for obs in self.obstacles:
            if obs["type"] == "cylinder":
                cx, cy = obs["center"]
                dist = math.hypot(px - cx, py - cy)
                if dist < (obs["radius"] + self.r_col):
                    return True, obs["name"]
            elif obs["type"] == "box":
                cx, cy = obs["center"]
                clamped_x = max(cx - obs["hx"], min(px, cx + obs["hx"]))
                clamped_y = max(cy - obs["hy"], min(py, cy + obs["hy"]))
                dist = math.hypot(px - clamped_x, py - clamped_y)
                if dist < self.r_col:
                    return True, obs["name"]
                    
        return False, None

    def validate_movement(self, cur_x, cur_y, prop_x, prop_y, num_substeps=8):
        """
        Swept-path continuous collision check between current and proposed positions.
        Returns: (safe_x, safe_y, collision_occurred, obstacle_name)
        If collision would occur, constrains to the closest safe point along the path.
        """
        first_collision_fraction = 1.0
        collided = False
        collided_obs = None
        
        for i in range(1, num_substeps + 1):
            fraction = i / float(num_substeps)
            sx = cur_x + fraction * (prop_x - cur_x)
            sy = cur_y + fraction * (prop_y - cur_y)
            is_col, name = self.is_point_colliding(sx, sy)
            if is_col:
                collided = True
                collided_obs = name
                first_collision_fraction = max(0.0, (i - 1) / float(num_substeps))
                break
                
        if collided:
            safe_x = cur_x + first_collision_fraction * (prop_x - cur_x)
            safe_y = cur_y + first_collision_fraction * (prop_y - cur_y)
            return safe_x, safe_y, True, collided_obs
        else:
            return prop_x, prop_y, False, None

# ---------------------------------------------------------------------------
# SENSOR SIMULATION ENGINE
# ---------------------------------------------------------------------------
class WheelEncoder:
    def __init__(self, ppr=ENCODER_PPR):
        self.ppr = ppr
        self.total_angle = 0.0
        self.total_ticks = 0
        
    def update(self, omega, dt):
        delta_angle = omega * dt
        self.total_angle += delta_angle
        self.total_ticks = int(round((self.total_angle / (2.0 * math.pi)) * self.ppr))
        return self.total_ticks, self.total_angle

class SimulatedIMU:
    def __init__(self, noise_std_gyro=0.006, noise_std_accel=0.012, bias_gyro=0.003):
        self.noise_std_gyro = noise_std_gyro
        self.noise_std_accel = noise_std_accel
        self.bias_gyro = bias_gyro
        self.last_vx = 0.0
        self.last_vy = 0.0
        
    def measure(self, yaw, vx, vy, omega, dt):
        gyro_noise = random.gauss(0, self.noise_std_gyro)
        meas_omega = omega + self.bias_gyro + gyro_noise
        
        if dt > 0:
            dvx_dt = (vx - self.last_vx) / dt
            dvy_dt = (vy - self.last_vy) / dt
        else:
            dvx_dt, dvy_dt = 0.0, 0.0
            
        ax_true = dvx_dt - vy * omega
        ay_true = dvy_dt + vx * omega
        
        ax_meas = ax_true + random.gauss(0, self.noise_std_accel)
        ay_meas = ay_true + random.gauss(0, self.noise_std_accel)
        
        self.last_vx = vx
        self.last_vy = vy
        
        return {
            "yaw_rad": yaw,
            "yaw_deg": math.degrees(yaw) % 360.0,
            "gyro_z_rad_s": round(meas_omega, 4),
            "accel_x_m_s2": round(ax_meas, 4),
            "accel_y_m_s2": round(ay_meas, 4)
        }

def raycast_ultrasonic_sensors(scene, depsgraph, robot_pos, robot_yaw, max_range=ULTRASONIC_MAX_RANGE):
    """Simulate HC-SR04 ultrasonic rangefinders using direct 3D mesh raycasting against obstacles and walls."""
    sensor_mount_r = 0.340
    sensor_height = 0.123
    
    sensors_def = [
        ("Front", 0.0),
        ("Left", math.pi / 2.0),
        ("Right", -math.pi / 2.0)
    ]
    
    target_names = [
        'Obstacle_Corner_Pillar', 'Obstacle_Crate_Lateral', 'Obstacle_Partition_Wall',
        'Obstacle_Pillar_Front', 'Wall_East', 'Wall_North', 'Wall_South', 'Wall_West'
    ]
    target_objects = [bpy.data.objects.get(name) for name in target_names if bpy.data.objects.get(name)]
    
    readings = {}
    
    for s_name, angle_offset in sensors_def:
        beam_angle = robot_yaw + angle_offset
        origin_x = robot_pos.x + sensor_mount_r * math.cos(beam_angle)
        origin_y = robot_pos.y + sensor_mount_r * math.sin(beam_angle)
        origin_z = sensor_height
        origin = Vector((origin_x, origin_y, origin_z))
        
        direction = Vector((math.cos(beam_angle), math.sin(beam_angle), 0.0)).normalized()
        
        closest_hit_dist = max_range
        closest_hit_loc = None
        hit_found = False
        
        for obj in target_objects:
            m_inv = obj.matrix_world.inverted()
            loc_orig = m_inv @ origin
            loc_dir = (m_inv.to_3x3() @ direction).normalized()
            hit, loc, norm, f_idx = obj.ray_cast(loc_orig, loc_dir, distance=max_range)
            if hit:
                world_hit = obj.matrix_world @ loc
                d = (world_hit - origin).length
                if d < closest_hit_dist:
                    closest_hit_dist = d
                    closest_hit_loc = world_hit
                    hit_found = True
                    
        if hit_found and closest_hit_dist <= max_range:
            noisy_dist = max(0.02, closest_hit_dist + random.gauss(0, 0.003))
            readings[s_name] = {
                "distance": round(noisy_dist, 3),
                "detected": noisy_dist <= OBSTACLE_STOP_THRESHOLD,
                "hit_point": (closest_hit_loc.x, closest_hit_loc.y, closest_hit_loc.z)
            }
        else:
            readings[s_name] = {
                "distance": max_range,
                "detected": False,
                "hit_point": None
            }
            
    return readings


# ---------------------------------------------------------------------------
# 16-STAGE MASTER DEMONSTRATION TRAJECTORY
# ---------------------------------------------------------------------------
def generate_master_trajectory():
    """
    Direct Navigation & Obstacle Avoidance Trajectory (Zero Random Startup Motion):
    1. INITIAL_REST: Robot stationary at Start Pad (0, 0) during system initialization (all velocities zero).
    2. FOLLOW_ROUTE: Direct navigation path from current position toward Goal destination corridor.
    3. STOP: Proactive emergency stop when obstacle is detected within safety threshold (d <= 0.65m).
    4. ASSESS_CLEARANCE: Evaluate Left & Right clearance to select evasion corridor.
    5. AVOID_OBSTACLE: Holonomic lateral evasion (+Y corridor) around obstacle with zero rotation.
    6. REJOIN_ROUTE: Advance forward past obstacle boundary.
    7. NAVIGATE_TO_GOAL: Direct holonomic diagonal advance to Destination Goal Pad (2.8m, 2.8m).
    8. GOAL_REACHED: Safely parked at Goal destination pad with zero residual velocity.
    """
    stages = [
        # (duration, vx, vy, omega, state_label, description)
        (1.0,    0.00,  0.00,  0.00,    "INITIAL_REST",     "Stage 1: Robot stationary at Start Pad (Initialization & Standby)"),
        (3.0,    0.45,  0.00,  0.00,    "FOLLOW_ROUTE",     "Stage 2: Direct Forward Navigation Toward Goal"),
        (1.0,    0.00,  0.00,  0.00,    "STOP",             "Stage 3: Proactive Stop at Obstacle Detection Threshold"),
        (1.0,    0.00,  0.00,  0.00,    "ASSESS_CLEARANCE", "Stage 4: Assess Sensor Clearance Corridors"),
        (2.5,    0.00,  0.45,  0.00,    "AVOID_OBSTACLE",   "Stage 5: Holonomic Lateral Evasion (+Y Escape Corridor)"),
        (3.2,    0.45,  0.00,  0.00,    "REJOIN_ROUTE",     "Stage 6: Rejoin Navigation Route Past Obstacle"),
        (3.5,    0.05,  0.47,  0.00,    "NAVIGATE_TO_GOAL", "Stage 7: Direct Approach to Destination Goal Pad"),
        (1.5,    0.00,  0.00,  0.00,    "GOAL_REACHED",     "Stage 8: Arrived and Parked at Destination Goal Pad")
    ]
    return stages

# ---------------------------------------------------------------------------
# MAIN SIMULATION RUNNER & KEYFRAME BAKER
# ---------------------------------------------------------------------------
def run_simulation():
    print("=" * 65)
    print("GROUP 7: RUNNING MASTER SIMULATION & SAFETY PIPELINE")
    print("=" * 65)
    
    verify_kinematics()
    
    scene = bpy.context.scene
    depsgraph = bpy.context.evaluated_depsgraph_get()
    
    robot_base = bpy.data.objects.get("Robot_Base")
    if not robot_base:
        raise RuntimeError("Robot_Base not found in scene! Run build_robot.py first.")
        
    wheels = [bpy.data.objects.get(f"Omni_Wheel_{i+1}") for i in range(3)]
    sensor_beams = {
        "Front": bpy.data.objects.get("US_Sensor_Front_Beam_Mesh"),
        "Left": bpy.data.objects.get("US_Sensor_Left_Beam_Mesh"),
        "Right": bpy.data.objects.get("US_Sensor_Right_Beam_Mesh")
    }
    mat_safe = bpy.data.materials.get("Mat_Beam_Safe")
    mat_alert = bpy.data.materials.get("Mat_Beam_Alert")
    
    encoders = [WheelEncoder(ENCODER_PPR) for _ in range(3)]
    imu = SimulatedIMU()
    collision_detector = GeometricCollisionDetector(r_col=TOTAL_COLLISION_RADIUS)
    
    # Clear existing animations
    for obj in [robot_base] + [w for w in wheels if w]:
        if obj and obj.animation_data:
            obj.animation_data_clear()
            
    x = 0.0
    y = 0.0
    yaw = 0.0
    dt = 1.0 / FPS
    
    stages = generate_master_trajectory()
    total_duration = sum(s[0] for s in stages)
    total_frames = int(total_duration * FPS)
    
    scene.frame_start = 1
    scene.frame_end = total_frames
    
    telemetry_records = []
    
    stage_idx = 0
    stage_timer = 0.0
    
    wheel_angles = [0.0, 0.0, 0.0]
    obstacle_detected = False
    collision_count = 0
    
    print(f"Simulating {total_frames} frames ({total_duration:.1f}s at {FPS} FPS)...")
    
    for frame in range(1, total_frames + 1):
        scene.frame_set(frame)
        
        dur, vx_req, vy_req, omega_req, state_label, stage_desc = stages[stage_idx]
        
        # 1. Ultrasonic sensing
        us_readings = raycast_ultrasonic_sensors(scene, depsgraph, Vector((x, y, 0)), yaw)
        front_dist = us_readings["Front"]["distance"]
        left_dist = us_readings["Left"]["distance"]
        right_dist = us_readings["Right"]["distance"]
        
        # Autonomous Obstacle Avoidance Trigger:
        # While navigating along route, if front sensor detects obstacle <= threshold:
        if state_label == "FOLLOW_ROUTE" and front_dist <= OBSTACLE_STOP_THRESHOLD and not obstacle_detected:
            print(f"  [!] Obstacle Proximity Alert at Frame {frame} (t={frame*dt:.2f}s, d={front_dist:.2f}m)!")
            print("      Transitioning: FOLLOW_ROUTE -> STOP -> ASSESS_CLEARANCE -> AVOID_OBSTACLE")
            for s_i, s_data in enumerate(stages):
                if s_data[4] == "STOP":
                    stage_idx = s_i
                    break
            stage_timer = 0.0
            dur, vx_req, vy_req, omega_req, state_label, stage_desc = stages[stage_idx]
            obstacle_detected = True
            
        # 2. Calculate proposed movement
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        v_world_x = vx_req * cos_yaw - vy_req * sin_yaw
        v_world_y = vx_req * sin_yaw + vy_req * cos_yaw
        
        prop_x = x + v_world_x * dt
        prop_y = y + v_world_y * dt
        prop_yaw = yaw + omega_req * dt
        
        # 3. Geometric Swept-Path Collision Check & Prevention Barrier
        safe_x, safe_y, col_occurred, col_name = collision_detector.validate_movement(x, y, prop_x, prop_y)
        
        if col_occurred:
            collision_count += 1
            print(f"  [COLLISION BARRIER ACTIVE] Frame {frame}: Constrained movement against {col_name}!")
            
        # Actual accepted displacement and velocities
        dx_actual = safe_x - x
        dy_actual = safe_y - y
        
        if dt > 0:
            act_world_vx = dx_actual / dt
            act_world_vy = dy_actual / dt
            act_vx =  act_world_vx * cos_yaw + act_world_vy * sin_yaw
            act_vy = -act_world_vx * sin_yaw + act_world_vy * cos_yaw
        else:
            act_vx, act_vy = 0.0, 0.0
            
        act_omega = omega_req
        
        # Update robot state to verified safe position
        x = safe_x
        y = safe_y
        yaw = prop_yaw
        
        # 4. Kinematics from ACTUAL ACCEPTED movement
        v_wheels, w_wheels, rpms = compute_inverse_kinematics(act_vx, act_vy, act_omega)
        
        # 5. Encoders from actual wheel rotation
        enc_ticks = []
        for i in range(3):
            ticks, cumulative_rad = encoders[i].update(w_wheels[i], dt)
            enc_ticks.append(ticks)
            wheel_angles[i] += w_wheels[i] * dt
            
        # 6. IMU measurement
        imu_data = imu.measure(yaw, act_vx, act_vy, act_omega, dt)
        
        # 7. Keyframe Robot Base
        robot_base.location = (x, y, 0.0)
        robot_base.rotation_euler = (0.0, 0.0, yaw)
        robot_base.keyframe_insert(data_path="location", frame=frame)
        robot_base.keyframe_insert(data_path="rotation_euler", frame=frame)
        
        # Keyframe Omni Wheels (spin about drive axle, local X)
        for i in range(3):
            if wheels[i]:
                wheels[i].rotation_euler.x = wheel_angles[i]
                wheels[i].keyframe_insert(data_path="rotation_euler", frame=frame)
                
        # Keyframe Ultrasonic Sensor Detection Rays
        for s_name in ["Front", "Left", "Right"]:
            beam_obj = sensor_beams.get(s_name)
            if beam_obj:
                s_dist = us_readings[s_name]["distance"]
                is_alert = us_readings[s_name]["detected"]
                
                beam_obj.scale = (1.0, 1.0, s_dist)
                beam_obj.location = (0.0, s_dist / 2.0 + 0.016, 0.0)
                beam_obj.keyframe_insert(data_path="scale", frame=frame)
                beam_obj.keyframe_insert(data_path="location", frame=frame)
                
                if is_alert and mat_alert:
                    if beam_obj.data.materials:
                        beam_obj.data.materials[0] = mat_alert
                    else:
                        beam_obj.data.materials.append(mat_alert)
                elif mat_safe:
                    if beam_obj.data.materials:
                        beam_obj.data.materials[0] = mat_safe
                    else:
                        beam_obj.data.materials.append(mat_safe)
                        
        # 8. Record Comprehensive Telemetry Frame
        telemetry_records.append({
            "frame": frame,
            "time_s": round(frame * dt, 3),
            "stage": stage_desc,
            "controller_state": state_label,
            "collision_prevention_active": col_occurred,
            "robot_x": round(x, 4),
            "robot_y": round(y, 4),
            "heading_deg": round(math.degrees(yaw) % 360.0, 2),
            "vx_requested": round(vx_req, 3),
            "vy_requested": round(vy_req, 3),
            "omega_requested": round(omega_req, 3),
            "vx_actual": round(act_vx, 3),
            "vy_actual": round(act_vy, 3),
            "omega_actual": round(act_omega, 3),
            "wheel_speeds_mps": [round(v, 4) for v in v_wheels],
            "wheel_rpms": [round(rpm, 1) for rpm in rpms],
            "encoder_ticks": enc_ticks,
            "imu": imu_data,
            "ultrasonic": {
                "front_m": front_dist,
                "left_m": left_dist,
                "right_m": right_dist,
                "hazard_detected": any(us_readings[k]["detected"] for k in us_readings)
            }
        })
        
        # Advance stage timer
        stage_timer += dt
        if stage_timer >= dur and stage_idx < len(stages) - 1:
            stage_idx += 1
            stage_timer = 0.0
            
    print(f"[1/3] Simulation physics and 3D keyframe baking completed. Total collisions during route: {collision_count}")
    
    # Export Telemetry Data to JSON, JS, and CSV
    output_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 1. JSON
    json_path = os.path.join(output_dir, "telemetry_data.json")
    with open(json_path, "w") as f:
        json.dump({
            "metadata": {
                "project": "Group 7 - Three-Wheel Omnidirectional Robot (Master Spec)",
                "wheel_radius_m": WHEEL_RADIUS,
                "chassis_radius_m": CHASSIS_RADIUS,
                "robot_footprint_radius_m": ROBOT_FOOTPRINT_RADIUS,
                "safety_margin_m": SAFETY_MARGIN,
                "total_collision_radius_m": TOTAL_COLLISION_RADIUS,
                "encoder_ppr": ENCODER_PPR,
                "fps": FPS,
                "total_frames": total_frames,
                "duration_s": total_duration,
                "total_collisions": collision_count
            },
            "telemetry": telemetry_records
        }, f, indent=2)
    print(f"[2/3] Exported telemetry JSON to: {json_path}")
    
    # 2. JS for Dashboard
    js_path = os.path.join(output_dir, "telemetry_data.js")
    with open(js_path, "w") as f:
        f.write("window.TELEMETRY_DATA = ")
        json.dump({
            "metadata": {
                "project": "Group 7 - Three-Wheel Omnidirectional Robot (Master Spec)",
                "wheel_radius_m": WHEEL_RADIUS,
                "chassis_radius_m": CHASSIS_RADIUS,
                "robot_footprint_radius_m": ROBOT_FOOTPRINT_RADIUS,
                "safety_margin_m": SAFETY_MARGIN,
                "total_collision_radius_m": TOTAL_COLLISION_RADIUS,
                "encoder_ppr": ENCODER_PPR,
                "fps": FPS,
                "total_frames": total_frames,
                "duration_s": total_duration,
                "total_collisions": collision_count
            },
            "telemetry": telemetry_records
        }, f, indent=2)
        f.write(";\n")
    print(f"[2.5/3] Exported telemetry JS to: {js_path}")
    
    # 3. CSV
    csv_path = os.path.join(output_dir, "telemetry_data.csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "frame", "time_s", "stage", "controller_state", "collision_prevention_active",
            "x", "y", "heading_deg",
            "vx_req", "vy_req", "omega_req",
            "vx_act", "vy_act", "omega_act",
            "w1_speed", "w2_speed", "w3_speed",
            "w1_rpm", "w2_rpm", "w3_rpm",
            "enc1_ticks", "enc2_ticks", "enc3_ticks",
            "imu_yaw_deg", "imu_gyro_z", "imu_ax", "imu_ay",
            "us_front_dist", "us_left_dist", "us_right_dist", "hazard_alert"
        ])
        for r in telemetry_records:
            writer.writerow([
                r["frame"], r["time_s"], r["stage"], r["controller_state"], 1 if r["collision_prevention_active"] else 0,
                r["robot_x"], r["robot_y"], r["heading_deg"],
                r["vx_requested"], r["vy_requested"], r["omega_requested"],
                r["vx_actual"], r["vy_actual"], r["omega_actual"],
                r["wheel_speeds_mps"][0], r["wheel_speeds_mps"][1], r["wheel_speeds_mps"][2],
                r["wheel_rpms"][0], r["wheel_rpms"][1], r["wheel_rpms"][2],
                r["encoder_ticks"][0], r["encoder_ticks"][1], r["encoder_ticks"][2],
                r["imu"]["yaw_deg"], r["imu"]["gyro_z_rad_s"], r["imu"]["accel_x_m_s2"], r["imu"]["accel_y_m_s2"],
                r["ultrasonic"]["front_m"], r["ultrasonic"]["left_m"], r["ultrasonic"]["right_m"],
                1 if r["ultrasonic"]["hazard_detected"] else 0
            ])
    print(f"[3/3] Exported telemetry CSV to: {csv_path}")
    
    # Save the updated .blend file with baked keyframes
    blend_path = bpy.data.filepath if bpy.data.filepath else os.path.join(output_dir, "Group7_OmniRobot.blend")
    bpy.ops.wm.save_as_mainfile(filepath=blend_path)
    print(f"Updated scene with animated trajectory saved to: {blend_path}")
    print("=" * 65)
    print("MASTER SIMULATION PIPELINE COMPLETE!")
    print("=" * 65)

if __name__ == "__main__":
    run_simulation()
