"""
Group 7 - Three-Wheel Omnidirectional Robot
Automated Verification & Sensor Simulation Test Suite
=====================================================
Authors: Group 7 Robotics Engineering Team

This test suite rigorously validates:
1. Three-wheel inverse and forward kinematics across all 5 motion modes:
   - Pure Forward
   - Pure Backward
   - Pure Lateral (Sideways)
   - Diagonal
   - Pure In-Place Rotation
2. Rotary encoder pulse integration and tracking against wheel angular rotation.
3. 6-DOF IMU response to yaw rotation rate, bias, and centripetal/Coriolis acceleration.
4. Ultrasonic rangefinder 3D raycasting against arena obstacles in Blender.
5. Model hierarchy, separate motor/encoder objects, labelled IMU, and 4 camera configurations.
"""

import bpy
import math
import os
import sys
import random
from mathutils import Vector, Euler

# Ensure local script directory is in sys.path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from simulate_robot import (
    compute_inverse_kinematics,
    compute_forward_kinematics,
    WheelEncoder,
    SimulatedIMU,
    raycast_ultrasonic_sensors,
    WHEEL_RADIUS,
    CHASSIS_RADIUS,
    ENCODER_PPR,
    OBSTACLE_STOP_THRESHOLD
)

passed_tests = []
failed_tests = []

def run_test(name, test_fn):
    print(f"\n[RUNNING TEST] {name}...")
    try:
        test_fn()
        passed_tests.append(name)
        print(f"  --> [PASS] {name}")
    except Exception as e:
        failed_tests.append((name, str(e)))
        print(f"  --> [FAIL] {name}: {e}")

def test_kinematics():
    """Verify kinematics equations and round-trip accuracy."""
    test_cases = [
        ("Pure Forward",   0.45,  0.00,  0.00),
        ("Pure Backward", -0.40,  0.00,  0.00),
        ("Pure Lateral",   0.00,  0.45,  0.00),
        ("Diagonal",       0.35, -0.35,  0.00),
        ("Pure Rotation",  0.00,  0.00,  1.50)
    ]
    for mode, vx, vy, w in test_cases:
        vw, ww, rpms = compute_inverse_kinematics(vx, vy, w)
        rec_vx, rec_vy, rec_w = compute_forward_kinematics(vw)
        
        # Verify forward kinematics recovers commanded velocities within 1e-10
        assert abs(vx - rec_vx) < 1e-10, f"{mode}: vx recovery error ({vx} vs {rec_vx})"
        assert abs(vy - rec_vy) < 1e-10, f"{mode}: vy recovery error ({vy} vs {rec_vy})"
        assert abs(w - rec_w) < 1e-10, f"{mode}: omega recovery error ({w} vs {rec_w})"
        
        # Verify mode-specific physical expectations
        if mode == "Pure Forward":
            assert abs(vw[0]) < 1e-10, "Wheel 1 should have zero speed during pure forward motion"
            assert abs(vw[1] + vw[2]) < 1e-10, "Wheel 2 and Wheel 3 should have equal and opposite speeds"
        elif mode == "Pure Backward":
            assert abs(vw[0]) < 1e-10, "Wheel 1 should have zero speed during pure backward motion"
            assert vw[1] > 0 and vw[2] < 0, "Wheel 2 should be positive and Wheel 3 negative"
        elif mode == "Pure Lateral":
            assert vw[0] > 0, "Wheel 1 should drive at full forward speed"
            assert abs(vw[1] - vw[2]) < 1e-10, "Wheel 2 and Wheel 3 speeds should be identical during lateral crab walk"
        elif mode == "Pure Rotation":
            assert abs(vw[0] - vw[1]) < 1e-10 and abs(vw[1] - vw[2]) < 1e-10, "All 3 wheels should spin at identical speed"

def test_encoders():
    """Verify optical rotary encoder tick integration."""
    enc = WheelEncoder(ppr=ENCODER_PPR)
    dt = 1.0 / 30.0
    omega = 10.0 # 10 rad/s (~95.5 RPM)
    
    # Simulate 30 frames (1.0 second)
    for _ in range(30):
        ticks, angle = enc.update(omega, dt)
        
    expected_angle = 10.0 # 10.0 radians
    expected_ticks = int(round((expected_angle / (2.0 * math.pi)) * ENCODER_PPR)) # ~573 ticks
    
    assert abs(angle - expected_angle) < 1e-6, f"Cumulative angle mismatch: {angle} vs {expected_angle}"
    assert abs(ticks - expected_ticks) <= 1, f"Tick count mismatch: {ticks} vs {expected_ticks}"
    
    # Test reverse direction
    for _ in range(15):
        ticks, angle = enc.update(-omega, dt)
    assert ticks < expected_ticks, "Ticks should decrease when motor spins in reverse"

def test_imu():
    """Verify 6-DOF IMU response to angular rotation and Coriolis terms."""
    random.seed(42)
    imu = SimulatedIMU(noise_std_gyro=0.005, noise_std_accel=0.01, bias_gyro=0.003)
    dt = 1.0 / 30.0
    
    # Test in-place spin: omega = 1.5 rad/s
    meas_spin = imu.measure(yaw=0.5, vx=0.0, vy=0.0, omega=1.5, dt=dt)
    # Measured gyro rate should be omega + bias (~1.503 rad/s) within 3-sigma noise (0.015)
    assert abs(meas_spin["gyro_z_rad_s"] - (1.5 + 0.003)) < 0.02, \
        f"IMU gyro rate mismatch: {meas_spin['gyro_z_rad_s']} vs expected ~1.503"
        
    # Test rotating with linear velocity: Centripetal/Coriolis acceleration
    # vx = 0.4, vy = 0.0, omega = 1.5 rad/s
    # a_body_y = dvy/dt + vx * omega = 0 + 0.4 * 1.5 = 0.6 m/s^2
    meas_coriolis = imu.measure(yaw=0.5, vx=0.4, vy=0.0, omega=1.5, dt=dt)
    assert abs(meas_coriolis["accel_y_m_s2"] - 0.60) < 0.05, \
        f"IMU centripetal acceleration mismatch: {meas_coriolis['accel_y_m_s2']} vs expected ~0.60 m/s^2"

def test_ultrasonic_raycasting():
    """Verify ultrasonic 3D raycasting against scene obstacles."""
    scene = bpy.context.scene
    depsgraph = bpy.context.evaluated_depsgraph_get()
    
    # Robot at origin (0, 0, 0), facing +X (yaw = 0)
    # Obstacle_Pillar_Front is at (2.50, 0.00, 0.65) with radius 0.38m
    # Expected distance from sensor origin (x=0.315) to pillar surface (x=2.50 - 0.38 = 2.12m):
    # dist ~ 2.12 - 0.315 = 1.805m
    readings_origin = raycast_ultrasonic_sensors(scene, depsgraph, Vector((0.0, 0.0, 0.0)), 0.0)
    front_d = readings_origin["Front"]["distance"]
    assert 1.70 <= front_d <= 1.95, f"Front ultrasonic distance at origin unexpected: {front_d}m"
    assert not readings_origin["Front"]["detected"], "Should not be in hazard alert at 1.8m"
    
    # Move robot closer to pillar: X = 1.60m
    # Distance to pillar surface: ~ 2.12 - 1.60 - 0.315 = 0.205m (< 0.55m safety threshold!)
    readings_close = raycast_ultrasonic_sensors(scene, depsgraph, Vector((1.60, 0.0, 0.0)), 0.0)
    close_d = readings_close["Front"]["distance"]
    assert close_d < OBSTACLE_STOP_THRESHOLD, f"Front sensor should detect hazard: {close_d}m"
    assert readings_close["Front"]["detected"] == True, "Hazard detected flag must be True"

def test_scene_objects_and_hierarchy():
    """Verify required scene objects, separate motors, encoders, IMU label, and cameras."""
    # 1. Robot base
    robot = bpy.data.objects.get("Robot_Base")
    assert robot is not None, "Robot_Base not found!"
    
    # 2. 3 Omni wheels
    for i in range(1, 4):
        w = bpy.data.objects.get(f"Omni_Wheel_{i}")
        assert w is not None, f"Omni_Wheel_{i} not found!"
        # Check rollers
        rollers = [o for o in w.children if "Roller" in o.name]
        assert len(rollers) == 10, f"Omni_Wheel_{i} should have 10 rollers, found {len(rollers)}"
        
    # 3. 3 Separate Motors
    for i in range(1, 4):
        m = bpy.data.objects.get(f"Drive_Motor_{i}")
        assert m is not None, f"Drive_Motor_{i} not found!"
        
    # 4. 3 Separate Encoders
    for i in range(1, 4):
        e = bpy.data.objects.get(f"Encoder_{i}")
        assert e is not None, f"Encoder_{i} not found!"
        
    # 5. Labelled IMU Module
    imu = bpy.data.objects.get("IMU_Module")
    assert imu is not None, "IMU_Module not found!"
    text_label = bpy.data.objects.get("IMU_Module_Label_Text")
    assert text_label is not None, "IMU_Module_Label_Text not found!"
    assert text_label.data.body == "IMU 6-DOF", f"IMU label text incorrect: {text_label.data.body}"
    
    # 6. Three Ultrasonic Sensors and Beams
    for side in ["Front", "Left", "Right"]:
        us = bpy.data.objects.get(f"US_Sensor_{side}")
        assert us is not None, f"US_Sensor_{side} not found!"
        beam = bpy.data.objects.get(f"US_Sensor_{side}_Beam_Mesh")
        assert beam is not None, f"US_Sensor_{side}_Beam_Mesh not found!"
        
    # 7. Four Cameras
    for cam_name in ["Camera_Closeup", "Camera_Presentation", "Camera_TopDown", "Camera_Follow"]:
        cam = bpy.data.objects.get(cam_name)
        assert cam is not None, f"{cam_name} not found!"
        assert cam.type == 'CAMERA', f"{cam_name} is not a Camera object!"
        if cam_name == "Camera_TopDown":
            assert cam.data.type == 'ORTHO', "Camera_TopDown must be an ORTHOGRAPHIC camera!"

def main():
    print("=" * 65)
    print("GROUP 7: RUNNING AUTOMATED SENSOR & KINEMATICS TEST SUITE")
    print("=" * 65)
    
    run_test("Kinematics Equations (Forward, Backward, Sideways, Diagonal, Rotation)", test_kinematics)
    run_test("Wheel Optical Encoders (Angle, PPR=360 Ticks, Reversal)", test_encoders)
    run_test("6-DOF IMU (Gyro Rate, Noise/Bias, Centripetal/Coriolis Acceleration)", test_imu)
    run_test("HC-SR04 Ultrasonic Raycasting & Obstacle Detection (3D Ray Cast)", test_ultrasonic_raycasting)
    run_test("Scene Hierarchy, Separate Motors & Encoders, Labelled IMU, 4 Cameras", test_scene_objects_and_hierarchy)
    
    print("\n" + "=" * 65)
    print(f"TEST SUITE SUMMARY: {len(passed_tests)} PASSED, {len(failed_tests)} FAILED")
    print("=" * 65)
    for t in passed_tests:
        print(f"  [PASS] {t}")
    for t, err in failed_tests:
        print(f"  [FAIL] {t}: {err}")
        
    if failed_tests:
        raise SystemExit(1)
    else:
        print("\nALL SENSOR & KINEMATICS VERIFICATION TESTS PASSED SUCCESSFULLY!")

if __name__ == "__main__":
    main()
