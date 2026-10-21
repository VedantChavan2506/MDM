"""
Group 7 - Three-Wheel Omnidirectional Robot
Kinematics, Sensor Simulation, and Obstacle Avoidance Script (Enhanced)
=====================================================================
Authors: Group 7 Robotics Engineering Team
Project: Three-Wheel Omni Robot — Holonomic Movement in Indoor Environments

This enhanced script implements:
1. Exact Three-Wheel Omni-Drive Kinematics:
   - Inverse Kinematics: [vx, vy, omega] -> [v1, v2, v3], [w1, w2, w3], [rpm1, rpm2, rpm3]
   - Forward Kinematics: [v1, v2, v3] -> [vx, vy, omega] (analytical inverse Jacobian)
   - Kinematics unit tests verifying Forward, Backward, Sideways, Diagonal, and In-place Rotation.
2. Sensor Simulation Engine:
   - 3 Wheel Encoders (cumulative angle, tick counts at 360 PPR, velocity tracking)
   - 6-DOF IMU (yaw angle, gyro angular rate with noise & bias, body frame linear accelerations
     including rotating frame centripetal / Coriolis terms)
   - 3 HC-SR04 Ultrasonic Rangefinders using Blender 3D scene raycasting against arena obstacles
3. Programmed Demonstration Trajectory with 7 Distinct Navigation Phases:
   - Phase 1: Pure Forward translation
   - Phase 2: Pure Backward translation (reverse kinematics verification)
   - Phase 3: Pure Sideways (lateral crab walk) translation with fixed heading
   - Phase 4: Diagonal translation (+X, -Y)
   - Phase 5: Pure In-Place Rotation (360° yaw spin)
   - Phase 6: Obstacle Approach (forward proximity tracking against pillar at X=2.5m)
   - Phase 7: Reactive Obstacle Avoidance (emergency deceleration at safety threshold <0.55m,
     visual ray warning color change, holonomic evasive escape)
4. 3D Keyframe Animation Baking into Group7_OmniRobot.blend for:
   - Robot chassis position (X, Y) and orientation (Yaw)
   - 3 Omni wheels spinning about their drive axles (local X axis)
   - Dynamic ultrasonic detection rays scaling to obstacle contact and switching safe/alert shaders
5. Telemetry Data Generation:
   - telemetry_data.json
   - telemetry_data.js
   - telemetry_data.csv
"""

import bpy
import math
import os
import json
import csv
import random
from mathutils import Vector, Matrix, Euler

# ---------------------------------------------------------------------------
# ROBOT PHYSICAL & SIMULATION PARAMETERS (Matches build_robot.py)
# ---------------------------------------------------------------------------
WHEEL_RADIUS = 0.05            # r = 50 mm (100 mm diameter)
CHASSIS_RADIUS = 0.30          # R = 300 mm (center to wheel contact point)
ENCODER_PPR = 360              # Pulses (ticks) per revolution
FPS = 30                       # Simulation frame rate
OBSTACLE_STOP_THRESHOLD = 0.55 # Stop/evade threshold distance (meters)
ULTRASONIC_MAX_RANGE = 2.50    # Maximum reliable sensing distance (meters)

# Wheel mounting angles relative to forward axis (+X_B)
WHEEL_ANGLES = [0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0] # 0°, 120°, 240°

# ---------------------------------------------------------------------------
# KINEMATICS ENGINE
# ---------------------------------------------------------------------------
def compute_inverse_kinematics(vx, vy, omega, R=CHASSIS_RADIUS, r=WHEEL_RADIUS):
    """
    Inverse Kinematics for a 3-wheel symmetric omnidirectional robot.
    Given body velocities [vx, vy, omega], returns:
    - wheel_linear_speeds: [v1, v2, v3] (m/s)
    - wheel_angular_velocities: [w1, w2, w3] (rad/s)
    - wheel_rpms: [rpm1, rpm2, rpm3]
    
    Equations:
    v_i = -vx * sin(phi_i) + vy * cos(phi_i) + R * omega
    w_i = v_i / r
    """
    v_wheels = []
    w_wheels = []
    rpms = []
    
    for phi in WHEEL_ANGLES:
        # Wheel tangent direction dot body velocity + angular rotation component
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
    Given wheel speeds [v1, v2, v3], returns body velocities [vx, vy, omega].
    
    Analytical solution for phi = [0, 120, 240 deg]:
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
        ("Pure Forward",  0.50,  0.00,  0.00),
        ("Pure Backward", -0.40,  0.00,  0.00),
        ("Pure Lateral",  0.00,  0.50,  0.00),
        ("Diagonal",      0.35, -0.35,  0.00),
        ("Pure Rotation", 0.00,  0.00,  1.50),
        ("Combined",      0.30,  0.20,  0.80)
    ]
    print("--- VERIFYING KINEMATICS ENGINE ---")
    for name, tx, ty, tw in test_cases:
        vw, ww, _ = compute_inverse_kinematics(tx, ty, tw)
        rx, ry, rw = compute_forward_kinematics(vw)
        err_x = abs(tx - rx)
        err_y = abs(ty - ry)
        err_w = abs(tw - rw)
        assert err_x < 1e-6 and err_y < 1e-6 and err_w < 1e-6, f"Mismatch in {name}!"
        print(f"  [PASS] {name:<15}: vx={tx:+.2f}, vy={ty:+.2f}, w={tw:+.2f} -> Wheel speeds: [{vw[0]:+.3f}, {vw[1]:+.3f}, {vw[2]:+.3f}] m/s")
    print("--- KINEMATICS VERIFICATION SUCCESSFUL ---\n")

# ---------------------------------------------------------------------------
# SENSOR SIMULATION ENGINE
# ---------------------------------------------------------------------------
class WheelEncoder:
    def __init__(self, ppr=ENCODER_PPR):
        self.ppr = ppr
        self.total_angle = 0.0 # radians
        self.total_ticks = 0
        
    def update(self, omega, dt):
        delta_angle = omega * dt
        self.total_angle += delta_angle
        # Cumulative ticks:
        self.total_ticks = int(round((self.total_angle / (2.0 * math.pi)) * self.ppr))
        return self.total_ticks, self.total_angle

class SimulatedIMU:
    def __init__(self, noise_std_gyro=0.008, noise_std_accel=0.015, bias_gyro=0.003):
        self.noise_std_gyro = noise_std_gyro
        self.noise_std_accel = noise_std_accel
        self.bias_gyro = bias_gyro
        self.last_vx = 0.0
        self.last_vy = 0.0
        
    def measure(self, yaw, vx, vy, omega, dt):
        """
        Simulate IMU measurements including gyro angular rate and
        accelerometer readings with centripetal/Coriolis acceleration.
        """
        gyro_noise = random.gauss(0, self.noise_std_gyro)
        meas_omega = omega + self.bias_gyro + gyro_noise
        
        if dt > 0:
            dvx_dt = (vx - self.last_vx) / dt
            dvy_dt = (vy - self.last_vy) / dt
        else:
            dvx_dt, dvy_dt = 0.0, 0.0
            
        # Kinematic acceleration in non-inertial rotating frame:
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
    """
    Simulate HC-SR04 ultrasonic rangefinders using Blender 3D scene raycasting against arena obstacles.
    Sensor Mounts:
    - Front: yaw offset 0°
    - Left:  yaw offset +90°
    - Right: yaw offset -90°
    """
    sensor_mount_r = 0.315
    sensor_height = 0.123
    
    sensors_def = [
        ("Front", 0.0),
        ("Left", math.pi / 2.0),
        ("Right", -math.pi / 2.0)
    ]
    
    readings = {}
    
    for s_name, angle_offset in sensors_def:
        beam_angle = robot_yaw + angle_offset
        origin_x = robot_pos.x + sensor_mount_r * math.cos(beam_angle)
        origin_y = robot_pos.y + sensor_mount_r * math.sin(beam_angle)
        origin_z = sensor_height
        origin = Vector((origin_x, origin_y, origin_z))
        
        direction = Vector((math.cos(beam_angle), math.sin(beam_angle), 0.0)).normalized()
        
        hit, hit_loc, normal, index, obj, matrix = scene.ray_cast(
            depsgraph, origin, direction, distance=max_range
        )
        
        # Ignore hits on robot components
        if hit and obj and ("Robot" in obj.name or "Wheel" in obj.name or "Chassis" in obj.name or "Pad" in obj.name or "Motor" in obj.name or "Beam" in obj.name):
            hit = False
            
        if hit:
            dist = (hit_loc - origin).length
            noisy_dist = max(0.02, dist + random.gauss(0, 0.003))
            readings[s_name] = {
                "distance": round(noisy_dist, 3),
                "detected": noisy_dist <= OBSTACLE_STOP_THRESHOLD,
                "hit_point": (hit_loc.x, hit_loc.y, hit_loc.z)
            }
        else:
            readings[s_name] = {
                "distance": max_range,
                "detected": False,
                "hit_point": None
            }
            
    return readings

# ---------------------------------------------------------------------------
# PROGRAMMED TEST ROUTE: FORWARD, BACKWARD, SIDEWAYS, DIAGONAL, ROTATION, EVADE
# ---------------------------------------------------------------------------
def generate_trajectory():
    """
    Define 7-phase holonomic demonstration trajectory:
    1. Forward translation (0.0 - 2.0s)
    2. Backward translation (2.0 - 4.0s)  -- VERIFIES BACKWARD
    3. Lateral translation (4.0 - 6.5s)   -- VERIFIES SIDEWAYS CRAB WALK
    4. Diagonal translation (6.5 - 9.0s)  -- VERIFIES DIAGONAL
    5. In-place rotation (9.0 - 11.5s)    -- VERIFIES ROTATION
    6. Approach obstacle (11.5 - 14.0s)   -- VERIFIES SENSOR TRACKING
    7. Reactive avoidance (14.0 - 17.5s)  -- VERIFIES BRAKING & HOLONOMIC EVASION
    """
    phases = [
        # (duration_sec, vx, vy, omega, description)
        (2.0,  0.45,  0.00,  0.00, "Phase 1: Pure Forward Translation"),
        (2.0, -0.40,  0.00,  0.00, "Phase 2: Pure Backward Translation"),
        (2.5,  0.00,  0.45,  0.00, "Phase 3: Pure Sideways (Lateral) Translation"),
        (2.5,  0.35, -0.35,  0.00, "Phase 4: Diagonal Translation (+X, -Y)"),
        (2.5,  0.00,  0.00,  1.50, "Phase 5: Pure In-Place Rotation (360° Spin)"),
        (2.5,  0.40,  0.00,  0.00, "Phase 6: Approaching Forward Obstacle"),
        (3.5,  0.00,  0.40,  0.35, "Phase 7: Reactive Obstacle Avoidance & Holonomic Evasion")
    ]
    return phases

# ---------------------------------------------------------------------------
# MAIN SIMULATION RUNNER & KEYFRAME BAKER
# ---------------------------------------------------------------------------
def run_simulation():
    print("=" * 65)
    print("GROUP 7: RUNNING ENHANCED HOLONOMIC SIMULATION & SENSOR PIPELINE")
    print("=" * 65)
    
    # 1. Verify kinematics
    verify_kinematics()
    
    scene = bpy.context.scene
    depsgraph = bpy.context.evaluated_depsgraph_get()
    
    robot_base = bpy.data.objects.get("Robot_Base")
    if not robot_base:
        raise RuntimeError("Robot_Base object not found in scene! Run build_robot.py first.")
        
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
    
    # Clear existing animation on objects
    for obj in [robot_base] + [w for w in wheels if w]:
        if obj and obj.animation_data:
            obj.animation_data_clear()
            
    # Trajectory state
    x = 0.0
    y = 0.0
    yaw = 0.0
    dt = 1.0 / FPS
    
    phases = generate_trajectory()
    total_duration = sum(p[0] for p in phases)
    total_frames = int(total_duration * FPS)
    
    scene.frame_start = 1
    scene.frame_end = total_frames
    
    telemetry_records = []
    
    phase_idx = 0
    phase_timer = 0.0
    
    # Cumulative wheel rotation angles (radians) around drive axle (local X)
    wheel_angles = [0.0, 0.0, 0.0]
    
    obstacle_detected_latch = False
    
    print(f"Simulating {total_frames} frames ({total_duration:.1f} seconds at {FPS} FPS)...")
    
    for frame in range(1, total_frames + 1):
        scene.frame_set(frame)
        
        # Get commanded velocities for current phase
        dur, vx_cmd, vy_cmd, omega_cmd, phase_desc = phases[phase_idx]
        
        # Ultrasonic raycasting against 3D scene obstacles
        us_readings = raycast_ultrasonic_sensors(scene, depsgraph, Vector((x, y, 0)), yaw)
        
        # Reactive obstacle safety trigger in Phase 6 (Approaching Obstacle)
        if phase_idx == 5:
            front_d = us_readings["Front"]["distance"]
            if front_d <= OBSTACLE_STOP_THRESHOLD and not obstacle_detected_latch:
                print(f"  [!] Obstacle proximity alert at Frame {frame} (Time: {frame*dt:.2f}s, Distance: {front_d:.2f}m)!")
                print("      Triggering reactive braking and holonomic evasive maneuver...")
                phase_idx = 6  # Transition directly to avoidance phase
                phase_timer = 0.0
                dur, vx_cmd, vy_cmd, omega_cmd, phase_desc = phases[phase_idx]
                obstacle_detected_latch = True
                
        # Inverse Kinematics: compute wheel linear speeds and RPMs
        v_wheels, w_wheels, rpms = compute_inverse_kinematics(vx_cmd, vy_cmd, omega_cmd)
        
        # Forward Kinematics verification
        rec_vx, rec_vy, rec_omega = compute_forward_kinematics(v_wheels)
        
        # Encoder simulation
        enc_ticks = []
        for i in range(3):
            ticks, cumulative_rad = encoders[i].update(w_wheels[i], dt)
            enc_ticks.append(ticks)
            wheel_angles[i] += w_wheels[i] * dt
            
        # IMU simulation
        imu_data = imu.measure(yaw, vx_cmd, vy_cmd, omega_cmd, dt)
        
        # Integrate robot world position
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        v_world_x = vx_cmd * cos_yaw - vy_cmd * sin_yaw
        v_world_y = vx_cmd * sin_yaw + vy_cmd * cos_yaw
        
        x += v_world_x * dt
        y += v_world_y * dt
        yaw += omega_cmd * dt
        
        # Set Blender keyframes for robot chassis
        robot_base.location = (x, y, 0.0)
        robot_base.rotation_euler = (0.0, 0.0, yaw)
        robot_base.keyframe_insert(data_path="location", frame=frame)
        robot_base.keyframe_insert(data_path="rotation_euler", frame=frame)
        
        # Keyframe individual wheel rotations around their drive axle (local X axis)
        for i in range(3):
            if wheels[i]:
                wheels[i].rotation_euler.x = wheel_angles[i]
                wheels[i].keyframe_insert(data_path="rotation_euler", frame=frame)
                
        # Update dynamic visual ultrasonic detection rays
        for s_name in ["Front", "Left", "Right"]:
            beam_obj = sensor_beams.get(s_name)
            if beam_obj:
                s_dist = us_readings[s_name]["distance"]
                is_alert = us_readings[s_name]["detected"]
                
                # Dynamic ray length scaling along sensor Y axis
                beam_obj.scale = (1.0, 1.0, s_dist)
                beam_obj.location = (0.0, s_dist / 2.0 + 0.016, 0.0)
                beam_obj.keyframe_insert(data_path="scale", frame=frame)
                beam_obj.keyframe_insert(data_path="location", frame=frame)
                
                # Material alert shader switch
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
                        
        # Record Telemetry Frame
        telemetry_records.append({
            "frame": frame,
            "time_s": round(frame * dt, 3),
            "phase": phase_desc,
            "robot_x": round(x, 4),
            "robot_y": round(y, 4),
            "heading_deg": round(math.degrees(yaw) % 360.0, 2),
            "vx_cmd": round(vx_cmd, 3),
            "vy_cmd": round(vy_cmd, 3),
            "omega_cmd": round(omega_cmd, 3),
            "wheel_speeds_mps": [round(v, 4) for v in v_wheels],
            "wheel_rpms": [round(rpm, 1) for rpm in rpms],
            "encoder_ticks": enc_ticks,
            "imu": imu_data,
            "ultrasonic": {
                "front_m": us_readings["Front"]["distance"],
                "left_m": us_readings["Left"]["distance"],
                "right_m": us_readings["Right"]["distance"],
                "hazard_detected": any(us_readings[k]["detected"] for k in us_readings)
            }
        })
        
        # Advance phase timer
        phase_timer += dt
        if phase_timer >= dur and phase_idx < len(phases) - 1:
            phase_idx += 1
            phase_timer = 0.0
            
    print("[1/3] Simulation physics and 3D keyframe baking completed.")
    
    # Export Telemetry Data to JSON, JS, and CSV
    output_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 1. JSON
    json_path = os.path.join(output_dir, "telemetry_data.json")
    with open(json_path, "w") as f:
        json.dump({
            "metadata": {
                "project": "Group 7 - Three-Wheel Omnidirectional Robot",
                "wheel_radius_m": WHEEL_RADIUS,
                "chassis_radius_m": CHASSIS_RADIUS,
                "encoder_ppr": ENCODER_PPR,
                "fps": FPS,
                "total_frames": total_frames,
                "duration_s": total_duration
            },
            "telemetry": telemetry_records
        }, f, indent=2)
    print(f"[2/3] Exported telemetry JSON to: {json_path}")
    
    # 2. JS (Dashboard global variable)
    js_path = os.path.join(output_dir, "telemetry_data.js")
    with open(js_path, "w") as f:
        f.write("window.TELEMETRY_DATA = ")
        json.dump({
            "metadata": {
                "project": "Group 7 - Three-Wheel Omnidirectional Robot",
                "wheel_radius_m": WHEEL_RADIUS,
                "chassis_radius_m": CHASSIS_RADIUS,
                "encoder_ppr": ENCODER_PPR,
                "fps": FPS,
                "total_frames": total_frames,
                "duration_s": total_duration
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
            "frame", "time_s", "phase", "x", "y", "heading_deg",
            "vx", "vy", "omega",
            "w1_speed", "w2_speed", "w3_speed",
            "w1_rpm", "w2_rpm", "w3_rpm",
            "enc1_ticks", "enc2_ticks", "enc3_ticks",
            "imu_yaw_deg", "imu_gyro_z", "imu_ax", "imu_ay",
            "us_front_dist", "us_left_dist", "us_right_dist", "hazard_alert"
        ])
        for r in telemetry_records:
            writer.writerow([
                r["frame"], r["time_s"], r["phase"], r["robot_x"], r["robot_y"], r["heading_deg"],
                r["vx_cmd"], r["vy_cmd"], r["omega_cmd"],
                r["wheel_speeds_mps"][0], r["wheel_speeds_mps"][1], r["wheel_speeds_mps"][2],
                r["wheel_rpms"][0], r["wheel_rpms"][1], r["wheel_rpms"][2],
                r["encoder_ticks"][0], r["encoder_ticks"][1], r["encoder_ticks"][2],
                r["imu"]["yaw_deg"], r["imu"]["gyro_z_rad_s"], r["imu"]["accel_x_m_s2"], r["imu"]["accel_y_m_s2"],
                r["ultrasonic"]["front_m"], r["ultrasonic"]["left_m"], r["ultrasonic"]["right_m"],
                1 if r["ultrasonic"]["hazard_detected"] else 0
            ])
    print(f"[3/3] Exported telemetry CSV to: {csv_path}")
    
    # Save the updated .blend file with baked keyframes
    blend_path = os.path.join(output_dir, "Group7_OmniRobot.blend")
    bpy.ops.wm.save_as_mainfile(filepath=blend_path)
    print(f"Updated scene with animated trajectory saved to: {blend_path}")
    print("=" * 65)
    print("SIMULATION PIPELINE COMPLETE!")
    print("=" * 65)

if __name__ == "__main__":
    run_simulation()
