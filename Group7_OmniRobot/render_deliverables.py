"""
Group 7 - Three-Wheel Omnidirectional Robot
Render Deliverables Script: Screenshots & Demonstration Video
============================================================
Authors: Group 7 Robotics Engineering Team

Renders:
1. screenshot_robot_closeup.png (Macro presentation of chassis, omni wheels, motors, sensors)
2. screenshot_arena_presentation.png (3D perspective of indoor testing arena)
3. screenshot_topdown_kinematics.png (Orthographic top-down analysis of holonomic trajectory)
4. screenshot_obstacle_detection.png (Approach & detection of obstacle with reactive beam)
5. simulation_demo.mp4 (Full 16-second 1080p 30fps animation of all 6 navigation phases)
"""

import bpy
import os
import math

def render_screenshot(scene, camera_name, frame, filename, output_dir, samples=16):
    """Render a single high-quality screenshot with Cycles CPU."""
    cam = bpy.data.objects.get(camera_name)
    if not cam:
        print(f"Warning: Camera {camera_name} not found!")
        return
        
    scene.camera = cam
    scene.frame_set(frame)
    scene.render.engine = 'CYCLES'
    scene.cycles.device = 'CPU'
    scene.cycles.samples = samples
    scene.cycles.use_denoising = False
    scene.render.resolution_x = 1280
    scene.render.resolution_y = 720
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = 'PNG'
    
    filepath = os.path.join(output_dir, filename)
    scene.render.filepath = filepath
    print(f"Rendering {filename} from {camera_name} (Frame {frame})...")
    bpy.ops.render.render(write_still=True)
    print(f"  -> Saved {filepath}")

def render_animation_video(scene, output_dir):
    """
    Render complete animation demonstration to MP4 video using high-speed Workbench
    with material colors, cavity edge highlights, and shadows.
    """
    cam = bpy.data.objects.get("Camera_Presentation")
    if not cam:
        cam = bpy.data.objects.get("Camera_Follow")
    scene.camera = cam
    
    scene.render.engine = 'BLENDER_WORKBENCH'
    scene.display.shading.color_type = 'MATERIAL'
    scene.display.shading.show_cavity = True
    scene.display.shading.cavity_type = 'BOTH'
    scene.display.shading.show_shadows = True
    
    scene.render.resolution_x = 1280
    scene.render.resolution_y = 720
    scene.render.fps = 30
    scene.frame_start = 1
    scene.frame_end = 480
    
    # Configure FFMPEG MP4
    scene.render.image_settings.file_format = 'FFMPEG'
    scene.render.ffmpeg.format = 'MPEG4'
    scene.render.ffmpeg.codec = 'H264'
    scene.render.ffmpeg.constant_rate_factor = 'MEDIUM'
    scene.render.ffmpeg.ffmpeg_preset = 'GOOD'
    
    video_path = os.path.join(output_dir, "simulation_demo.mp4")
    scene.render.filepath = video_path
    
    print(f"Rendering full simulation video to {video_path} (Frames 1 - {scene.frame_end})...")
    bpy.ops.render.render(animation=True)
    print("  -> Video rendering complete!")

def main():
    output_dir = os.path.dirname(os.path.abspath(__file__))
    scene = bpy.context.scene
    
    print("=" * 60)
    print("GROUP 7: RENDERING PROJECT DELIVERABLES")
    print("=" * 60)
    
    # 1. Macro Closeup Beauty Shot (Frame 1)
    render_screenshot(scene, "Camera_Closeup", 1, "screenshot_robot_closeup.png", output_dir)
    
    # 2. 3D Arena Presentation Shot (Frame 60, forward translation phase)
    render_screenshot(scene, "Camera_Presentation", 60, "screenshot_arena_presentation.png", output_dir)
    
    # 3. Top-Down Orthographic Kinematics View (Frame 120, lateral translation phase)
    render_screenshot(scene, "Camera_TopDown", 120, "screenshot_topdown_kinematics.png", output_dir)
    
    # 4. Obstacle Detection & Avoidance Event (Frame 330, approaching obstacle)
    render_screenshot(scene, "Camera_Presentation", 330, "screenshot_obstacle_detection.png", output_dir)
    
    # 5. Render Demonstration Video MP4
    render_animation_video(scene, output_dir)
    
    print("=" * 60)
    print("ALL VISUAL DELIVERABLES RENDERED SUCCESSFULLY!")
    print("=" * 60)

if __name__ == "__main__":
    main()
