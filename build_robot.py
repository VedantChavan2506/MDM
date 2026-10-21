"""
Group 7 - Three-Wheel Omnidirectional Robot
Blender 3D Modeling and Scene Setup Script (Master Version)
============================================================
Authors: Group 7 Robotics Engineering Team
Project: Three-Wheel Omni Robot — Holonomic Movement in Indoor Environments

Meets all Master Prompt specifications:
- Collections: Robot_Chassis, Omni_Wheels, Motors, Encoders, IMU, Ultrasonic_Sensors, Environment, Obstacles, Cameras
- Triangular dual-deck chassis with Stand-offs, Battery, MCU, Driver
- Three 120° dual-disc omni wheels with staggered passive barrel rollers
- Three separate drive motor assemblies (Drive_Motor_1, Drive_Motor_2, Drive_Motor_3)
- Three separate optical encoder assemblies (Encoder_1, Encoder_2, Encoder_3)
- Clearly labelled IMU sensor module with 3D text badge "IMU 6-DOF" at chassis center
- Three visible ultrasonic sensor modules with dynamic visual detection beams
- Indoor testing arena with grid floor, Start Pad, Goal Pad, Walls with hazard stripes, Reference Station, and 4 Obstacles
- Four specialized cameras: Camera_Presentation, Camera_Closeup, Camera_TopDown, Camera_Follow
"""

import bpy
import math
import os
from mathutils import Vector, Euler

# ---------------------------------------------------------------------------
# PHYSICAL ROBOT CONSTANTS
# ---------------------------------------------------------------------------
CHASSIS_RADIUS = 0.35        # Center to triangular vertex (350 mm)
WHEEL_DIST = 0.30            # Center to wheel hub contact point (300 mm)
WHEEL_RADIUS = 0.05          # Wheel outer radius (50 mm radius = 100 mm diameter)
WHEEL_WIDTH = 0.034          # Dual-disc rim width (34 mm)
GROUND_CLEARANCE = 0.018     # Floor to bottom deck plate (18 mm)
DECK_GAP = 0.075             # Spacing between bottom and top decks (75 mm)
PLATE_THICKNESS = 0.005      # 5mm aluminum/acrylic chassis decks
NUM_ROLLERS = 10             # Passive barrel rollers per wheel

# ---------------------------------------------------------------------------
# SCENE MANAGEMENT
# ---------------------------------------------------------------------------
def clear_existing_scene():
    """Wipe current blend data to guarantee clean generation."""
    bpy.ops.wm.read_factory_settings(use_empty=True)
    for block in bpy.data.meshes:
        bpy.data.meshes.remove(block)
    for block in bpy.data.materials:
        bpy.data.materials.remove(block)
    for block in bpy.data.cameras:
        bpy.data.cameras.remove(block)
    for block in bpy.data.lights:
        bpy.data.lights.remove(block)
    for block in bpy.data.curves:
        bpy.data.curves.remove(block)

def create_collection(name, parent_collection=None):
    """Create or retrieve a collection."""
    if name in bpy.data.collections:
        col = bpy.data.collections[name]
    else:
        col = bpy.data.collections.new(name)
        if parent_collection:
            parent_collection.children.link(col)
        else:
            bpy.context.scene.collection.children.link(col)
    return col

# ---------------------------------------------------------------------------
# MATERIALS SETUP
# ---------------------------------------------------------------------------
def get_or_create_material(name, base_color, metallic=0.0, roughness=0.5, emission_color=None, emission_strength=0.0):
    """Create a Principled BSDF material with specified PBR and Viewport properties."""
    if name in bpy.data.materials:
        return bpy.data.materials[name]
    
    mat = bpy.data.materials.new(name=name)
    mat.diffuse_color = base_color
    mat.roughness = roughness
    mat.metallic = metallic
    
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    nodes.clear()
    
    node_output = nodes.new(type='ShaderNodeOutputMaterial')
    node_output.location = (400, 0)
    
    node_bsdf = nodes.new(type='ShaderNodeBsdfPrincipled')
    node_bsdf.location = (0, 0)
    node_bsdf.inputs['Base Color'].default_value = base_color
    node_bsdf.inputs['Metallic'].default_value = metallic
    node_bsdf.inputs['Roughness'].default_value = roughness
    
    if emission_color and emission_strength > 0:
        if 'Emission Color' in node_bsdf.inputs:
            node_bsdf.inputs['Emission Color'].default_value = emission_color
            node_bsdf.inputs['Emission Strength'].default_value = emission_strength
        elif 'Emission' in node_bsdf.inputs:
            node_bsdf.inputs['Emission'].default_value = emission_color
            if 'Emission Strength' in node_bsdf.inputs:
                node_bsdf.inputs['Emission Strength'].default_value = emission_strength
                
    mat.node_tree.links.new(node_bsdf.outputs['BSDF'], node_output.inputs['Surface'])
    return mat

def create_grid_material(name):
    """Create procedural indoor testing floor grid material."""
    if name in bpy.data.materials:
        return bpy.data.materials[name]
        
    mat = bpy.data.materials.new(name=name)
    mat.diffuse_color = (0.08, 0.09, 0.12, 1.0)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    nodes.clear()
    
    out = nodes.new(type='ShaderNodeOutputMaterial')
    out.location = (600, 0)
    
    bsdf = nodes.new(type='ShaderNodeBsdfPrincipled')
    bsdf.location = (300, 0)
    bsdf.inputs['Roughness'].default_value = 0.28
    bsdf.inputs['Metallic'].default_value = 0.15
    
    tex_coord = nodes.new(type='ShaderNodeTexCoord')
    tex_coord.location = (-400, 0)
    
    grid_tex = nodes.new(type='ShaderNodeTexBrick')
    grid_tex.location = (-100, 0)
    grid_tex.inputs['Color1'].default_value = (0.09, 0.10, 0.13, 1.0)
    grid_tex.inputs['Color2'].default_value = (0.06, 0.07, 0.09, 1.0)
    grid_tex.inputs['Mortar'].default_value = (0.20, 0.45, 0.85, 1.0)
    grid_tex.inputs['Scale'].default_value = 5.0
    grid_tex.inputs['Mortar Size'].default_value = 0.015
    grid_tex.inputs['Mortar Smooth'].default_value = 0.1
    grid_tex.inputs['Row Height'].default_value = 1.0
    
    mat.node_tree.links.new(tex_coord.outputs['Object'], grid_tex.inputs['Vector'])
    mat.node_tree.links.new(grid_tex.outputs['Color'], bsdf.inputs['Base Color'])
    mat.node_tree.links.new(bsdf.outputs['BSDF'], out.inputs['Surface'])
    return mat

def setup_materials():
    """Setup all project materials with distinct PBR shaders and viewport colors."""
    mats = {}
    mats['Chassis_Plate'] = get_or_create_material('Mat_Chassis_Plate', (0.04, 0.04, 0.06, 1.0), metallic=0.85, roughness=0.20)
    mats['Standoff'] = get_or_create_material('Mat_Standoff', (0.85, 0.85, 0.90, 1.0), metallic=0.95, roughness=0.15)
    mats['Fastener'] = get_or_create_material('Mat_Fastener', (0.92, 0.92, 0.96, 1.0), metallic=1.0, roughness=0.10)
    
    mats['Motor_Body'] = get_or_create_material('Mat_Motor_Body', (0.16, 0.17, 0.20, 1.0), metallic=0.80, roughness=0.30)
    mats['Gearbox'] = get_or_create_material('Mat_Gearbox', (0.78, 0.80, 0.84, 1.0), metallic=0.92, roughness=0.18)
    mats['Motor_Bracket'] = get_or_create_material('Mat_Motor_Bracket', (0.05, 0.38, 0.72, 1.0), metallic=0.85, roughness=0.28)
    mats['Shaft'] = get_or_create_material('Mat_Shaft', (0.90, 0.90, 0.93, 1.0), metallic=0.98, roughness=0.08)
    mats['Encoder_Disc'] = get_or_create_material('Mat_Encoder_Disc', (0.95, 0.75, 0.10, 1.0), metallic=0.90, roughness=0.20)
    mats['Encoder_Housing'] = get_or_create_material('Mat_Encoder_Housing', (0.02, 0.50, 0.18, 1.0), metallic=0.15, roughness=0.35)
    
    mats['Wheel_Hub'] = get_or_create_material('Mat_Wheel_Hub', (0.88, 0.10, 0.10, 1.0), metallic=0.88, roughness=0.22)
    mats['Roller_Rubber'] = get_or_create_material('Mat_Roller_Rubber', (0.06, 0.06, 0.07, 1.0), metallic=0.0, roughness=0.85)
    mats['Roller_Pin'] = get_or_create_material('Mat_Roller_Pin', (0.92, 0.92, 0.92, 1.0), metallic=1.0, roughness=0.10)
    
    mats['PCB_Green'] = get_or_create_material('Mat_PCB_Green', (0.02, 0.45, 0.12, 1.0), metallic=0.10, roughness=0.40)
    mats['PCB_Blue'] = get_or_create_material('Mat_PCB_Blue', (0.04, 0.28, 0.88, 1.0), metallic=0.10, roughness=0.35)
    mats['Chip_IC'] = get_or_create_material('Mat_Chip_IC', (0.02, 0.02, 0.02, 1.0), metallic=0.10, roughness=0.40)
    mats['Text_White'] = get_or_create_material('Mat_Text_White', (0.95, 0.95, 0.98, 1.0), metallic=0.20, roughness=0.20,
                                               emission_color=(0.95, 0.95, 0.98, 1.0), emission_strength=2.0)
    mats['LED_Green'] = get_or_create_material('Mat_LED_Green', (0.1, 1.0, 0.2, 1.0), metallic=0.0, roughness=0.1, 
                                               emission_color=(0.1, 1.0, 0.2, 1.0), emission_strength=6.0)
    mats['LED_Amber'] = get_or_create_material('Mat_LED_Amber', (1.0, 0.65, 0.0, 1.0), metallic=0.0, roughness=0.1, 
                                               emission_color=(1.0, 0.65, 0.0, 1.0), emission_strength=6.0)
    mats['Battery'] = get_or_create_material('Mat_Battery', (0.08, 0.52, 0.96, 1.0), metallic=0.20, roughness=0.38)
    
    mats['Ultrasonic_Can'] = get_or_create_material('Mat_Ultrasonic_Can', (0.90, 0.91, 0.94, 1.0), metallic=0.96, roughness=0.12)
    mats['Ultrasonic_Mesh'] = get_or_create_material('Mat_Ultrasonic_Mesh', (0.08, 0.08, 0.08, 1.0), metallic=0.25, roughness=0.85)
    mats['Beam_Safe'] = get_or_create_material('Mat_Beam_Safe', (0.0, 1.0, 0.45, 0.75), metallic=0.0, roughness=0.20,
                                               emission_color=(0.0, 1.0, 0.45, 1.0), emission_strength=4.5)
    mats['Beam_Alert'] = get_or_create_material('Mat_Beam_Alert', (1.0, 0.12, 0.12, 0.85), metallic=0.0, roughness=0.20,
                                                emission_color=(1.0, 0.12, 0.12, 1.0), emission_strength=8.0)
    
    mats['Arena_Floor'] = create_grid_material('Mat_Arena_Floor')
    mats['Wall_Concrete'] = get_or_create_material('Mat_Wall_Concrete', (0.42, 0.44, 0.48, 1.0), metallic=0.0, roughness=0.80)
    mats['Hazard_Stripe'] = get_or_create_material('Mat_Hazard_Stripe', (0.98, 0.82, 0.06, 1.0), metallic=0.10, roughness=0.35)
    mats['Obstacle_Orange'] = get_or_create_material('Mat_Obstacle_Orange', (0.98, 0.38, 0.04, 1.0), metallic=0.10, roughness=0.30)
    mats['Obstacle_Teal'] = get_or_create_material('Mat_Obstacle_Teal', (0.06, 0.68, 0.72, 1.0), metallic=0.10, roughness=0.30)
    mats['Obstacle_Metal'] = get_or_create_material('Mat_Obstacle_Metal', (0.48, 0.50, 0.54, 1.0), metallic=0.88, roughness=0.22)
    mats['Start_Pad'] = get_or_create_material('Mat_Start_Pad', (0.12, 0.54, 0.24, 1.0), metallic=0.20, roughness=0.45)
    mats['Goal_Pad'] = get_or_create_material('Mat_Goal_Pad', (0.05, 0.55, 0.85, 1.0), metallic=0.30, roughness=0.35,
                                              emission_color=(0.05, 0.55, 0.85, 1.0), emission_strength=2.0)
    
    mats['Axis_X_Red'] = get_or_create_material('Mat_Axis_X_Red', (1.0, 0.05, 0.05, 1.0), emission_color=(1.0, 0.05, 0.05, 1.0), emission_strength=3.0)
    mats['Axis_Y_Green'] = get_or_create_material('Mat_Axis_Y_Green', (0.05, 1.0, 0.05, 1.0), emission_color=(0.05, 1.0, 0.05, 1.0), emission_strength=3.0)
    mats['Axis_Z_Blue'] = get_or_create_material('Mat_Axis_Z_Blue', (0.10, 0.25, 1.0, 1.0), emission_color=(0.10, 0.25, 1.0, 1.0), emission_strength=3.0)
    
    return mats

# ---------------------------------------------------------------------------
# GEOMETRY GENERATORS: CHASSIS, OMNI WHEELS, ACTUATORS, SENSORS
# ---------------------------------------------------------------------------
def build_triangular_plate(name, radius=CHASSIS_RADIUS, thickness=PLATE_THICKNESS, corner_radius=0.05, collection=None, material=None):
    """Build a rounded triangular chassis deck plate."""
    mesh = bpy.data.meshes.new(name + "_Mesh")
    obj = bpy.data.objects.new(name, mesh)
    if collection:
        collection.objects.link(obj)
    
    verts = []
    faces = []
    
    num_sub = 10
    angles = [math.radians(a) for a in [0, 120, 240]]
    profile_2d = []
    
    for a in angles:
        cx = (radius - corner_radius) * math.cos(a)
        cy = (radius - corner_radius) * math.sin(a)
        for i in range(num_sub):
            theta = a - math.pi/3 + (2*math.pi/3) * (i / (num_sub - 1))
            px = cx + corner_radius * math.cos(theta)
            py = cy + corner_radius * math.sin(theta)
            profile_2d.append((px, py))
            
    n = len(profile_2d)
    for p in profile_2d:
        verts.append((p[0], p[1], -thickness/2))
    for p in profile_2d:
        verts.append((p[0], p[1], thickness/2))
        
    c_bot = len(verts)
    verts.append((0, 0, -thickness/2))
    c_top = len(verts)
    verts.append((0, 0, thickness/2))
    
    for i in range(n):
        nxt = (i + 1) % n
        faces.append([c_bot, nxt, i])
    for i in range(n):
        nxt = (i + 1) % n
        faces.append([c_top, i + n, nxt + n])
    for i in range(n):
        nxt = (i + 1) % n
        faces.append([i, nxt, nxt + n, i + n])
        
    mesh.from_pydata(verts, [], faces)
    mesh.update()
    
    if material:
        obj.data.materials.append(material)
        
    bev = obj.modifiers.new(name="Bevel", type='BEVEL')
    bev.width = 0.003
    bev.segments = 2
    
    return obj

def build_omni_wheel(wheel_name, radius=WHEEL_RADIUS, width=WHEEL_WIDTH, num_rollers=NUM_ROLLERS, collection=None, materials=None):
    """
    Build a physically and kinematically correct 120° omni wheel assembly.
    Drive axle along local +X (radial). Traction along local +Y (tangent).
    Rollers rotate tangential to rim in Y-Z plane.
    """
    wheel_empty = bpy.data.objects.new(wheel_name, None)
    wheel_empty.empty_display_type = 'ARROWS'
    wheel_empty.empty_display_size = 0.06
    if collection:
        collection.objects.link(wheel_empty)
        
    plate_thk = 0.0035
    half_w = width / 2.0
    
    # Dual anodized aluminum rim plates
    for side, sign in [("L", -1), ("R", 1)]:
        disc_mesh = bpy.data.meshes.new(f"{wheel_name}_Plate_{side}_Mesh")
        disc_obj = bpy.data.objects.new(f"{wheel_name}_Plate_{side}", disc_mesh)
        collection.objects.link(disc_obj)
        disc_obj.parent = wheel_empty
        
        bpy.ops.mesh.primitive_cylinder_add(
            vertices=32, radius=radius * 0.80, depth=plate_thk,
            location=(sign * (half_w - plate_thk/2), 0, 0),
            rotation=(0, math.pi/2, 0)
        )
        temp_obj = bpy.context.active_object
        disc_obj.data = temp_obj.data
        bpy.data.objects.remove(temp_obj, do_unlink=True)
        
        if materials and 'Wheel_Hub' in materials:
            disc_obj.data.materials.append(materials['Wheel_Hub'])
            
    # Central collar along local X
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=24, radius=0.014, depth=width * 1.15,
        location=(0, 0, 0), rotation=(0, math.pi/2, 0)
    )
    collar = bpy.context.active_object
    collar.name = f"{wheel_name}_Collar"
    collar.parent = wheel_empty
    if materials and 'Gearbox' in materials:
        collar.data.materials.append(materials['Gearbox'])
    collection.objects.link(collar)
    bpy.context.scene.collection.objects.unlink(collar)
    
    # Staggered passive barrel rollers around circumference
    roller_orbit_r = radius * 0.82
    roller_rad = radius * 0.22
    roller_len = (2 * math.pi * roller_orbit_r) / (num_rollers * 1.35)
    
    for i in range(num_rollers):
        ang = (2 * math.pi * i) / num_rollers
        row_offset = (half_w * 0.50) if (i % 2 == 0) else (-half_w * 0.50)
        
        rx = row_offset
        ry = roller_orbit_r * math.cos(ang)
        rz = roller_orbit_r * math.sin(ang)
        roller_rot = (ang, 0.0, 0.0)
        
        bpy.ops.mesh.primitive_cylinder_add(
            vertices=20, radius=roller_rad, depth=roller_len,
            location=(rx, ry, rz),
            rotation=roller_rot
        )
        roller = bpy.context.active_object
        roller.name = f"{wheel_name}_Roller_{i+1}"
        roller.parent = wheel_empty
        if materials and 'Roller_Rubber' in materials:
            roller.data.materials.append(materials['Roller_Rubber'])
            
        bev = roller.modifiers.new("Bevel", 'BEVEL')
        bev.width = roller_rad * 0.40
        bev.segments = 3
        
        collection.objects.link(roller)
        bpy.context.scene.collection.objects.unlink(roller)
        
        # Steel axle pin
        bpy.ops.mesh.primitive_cylinder_add(
            vertices=10, radius=0.0022, depth=roller_len * 1.25,
            location=(rx, ry, rz),
            rotation=roller_rot
        )
        pin = bpy.context.active_object
        pin.name = f"{wheel_name}_Pin_{i+1}"
        pin.parent = wheel_empty
        if materials and 'Roller_Pin' in materials:
            pin.data.materials.append(materials['Roller_Pin'])
        collection.objects.link(pin)
        bpy.context.scene.collection.objects.unlink(pin)
        
    return wheel_empty

def build_motor(motor_name, collection=None, materials=None):
    """Build a standalone DC gearmotor object with planetary gearbox and mounting bracket."""
    root = bpy.data.objects.new(motor_name, None)
    root.empty_display_type = 'PLAIN_AXES'
    root.empty_display_size = 0.04
    if collection:
        collection.objects.link(root)
        
    # Planetary Gearbox housing (along X axis)
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=24, radius=0.022, depth=0.035,
        location=(0.0175, 0, 0), rotation=(0, math.pi/2, 0)
    )
    gearbox = bpy.context.active_object
    gearbox.name = f"{motor_name}_Gearbox"
    gearbox.parent = root
    if materials and 'Gearbox' in materials:
        gearbox.data.materials.append(materials['Gearbox'])
    collection.objects.link(gearbox)
    bpy.context.scene.collection.objects.unlink(gearbox)
    
    # Motor stator can
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=24, radius=0.021, depth=0.055,
        location=(-0.0275, 0, 0), rotation=(0, math.pi/2, 0)
    )
    motor_can = bpy.context.active_object
    motor_can.name = f"{motor_name}_Stator"
    motor_can.parent = root
    if materials and 'Motor_Body' in materials:
        motor_can.data.materials.append(materials['Motor_Body'])
    collection.objects.link(motor_can)
    bpy.context.scene.collection.objects.unlink(motor_can)
    
    # Output drive shaft
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=16, radius=0.004, depth=0.030,
        location=(0.045, 0, 0), rotation=(0, math.pi/2, 0)
    )
    shaft = bpy.context.active_object
    shaft.name = f"{motor_name}_Output_Shaft"
    shaft.parent = root
    if materials and 'Shaft' in materials:
        shaft.data.materials.append(materials['Shaft'])
    collection.objects.link(shaft)
    bpy.context.scene.collection.objects.unlink(shaft)
    
    # Aluminum L-Bracket
    bpy.ops.mesh.primitive_cube_add(
        size=1.0, location=(0.01, 0, -0.025),
        scale=(0.038, 0.052, 0.005)
    )
    bracket = bpy.context.active_object
    bracket.name = f"{motor_name}_Mounting_Bracket"
    bracket.parent = root
    if materials and 'Motor_Bracket' in materials:
        bracket.data.materials.append(materials['Motor_Bracket'])
    collection.objects.link(bracket)
    bpy.context.scene.collection.objects.unlink(bracket)
    
    return root

def build_encoder(encoder_name, collection=None, materials=None):
    """Build a standalone optical rotary encoder object."""
    root = bpy.data.objects.new(encoder_name, None)
    root.empty_display_type = 'PLAIN_AXES'
    root.empty_display_size = 0.035
    if collection:
        collection.objects.link(root)
        
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=24, radius=0.020, depth=0.018,
        location=(-0.009, 0, 0), rotation=(0, math.pi/2, 0)
    )
    housing = bpy.context.active_object
    housing.name = f"{encoder_name}_Housing"
    housing.parent = root
    if materials and 'Encoder_Housing' in materials:
        housing.data.materials.append(materials['Encoder_Housing'])
    collection.objects.link(housing)
    bpy.context.scene.collection.objects.unlink(housing)
    
    # Optical code disc
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=32, radius=0.018, depth=0.002,
        location=(-0.002, 0, 0), rotation=(0, math.pi/2, 0)
    )
    disc = bpy.context.active_object
    disc.name = f"{encoder_name}_Code_Disc"
    disc.parent = root
    if materials and 'Encoder_Disc' in materials:
        disc.data.materials.append(materials['Encoder_Disc'])
    collection.objects.link(disc)
    bpy.context.scene.collection.objects.unlink(disc)
    
    # Optical photo-sensor
    bpy.ops.mesh.primitive_cube_add(
        size=1.0, location=(-0.006, 0.012, 0.008),
        scale=(0.010, 0.008, 0.012)
    )
    sensor = bpy.context.active_object
    sensor.name = f"{encoder_name}_PhotoSensor"
    sensor.parent = root
    if materials and 'Chip_IC' in materials:
        sensor.data.materials.append(materials['Chip_IC'])
    collection.objects.link(sensor)
    bpy.context.scene.collection.objects.unlink(sensor)
    
    # 4-pin connector
    bpy.ops.mesh.primitive_cube_add(
        size=1.0, location=(-0.016, 0.0, 0.012),
        scale=(0.008, 0.016, 0.008)
    )
    conn = bpy.context.active_object
    conn.name = f"{encoder_name}_Connector"
    conn.parent = root
    if materials and 'Fastener' in materials:
        conn.data.materials.append(materials['Fastener'])
    collection.objects.link(conn)
    bpy.context.scene.collection.objects.unlink(conn)
    
    return root

def build_labelled_imu(name="IMU_Module", collection=None, materials=None):
    """Build clearly labelled 6-DOF IMU sensor module with 3D text badge."""
    root = bpy.data.objects.new(name, None)
    root.empty_display_type = 'PLAIN_AXES'
    root.empty_display_size = 0.04
    if collection:
        collection.objects.link(root)
        
    bpy.ops.mesh.primitive_cube_add(
        size=1.0, location=(0, 0, 0.00125),
        scale=(0.044, 0.036, 0.0025)
    )
    pcb = bpy.context.active_object
    pcb.name = f"{name}_PCB"
    pcb.parent = root
    if materials and 'PCB_Blue' in materials:
        pcb.data.materials.append(materials['PCB_Blue'])
    collection.objects.link(pcb)
    bpy.context.scene.collection.objects.unlink(pcb)
    
    bpy.ops.mesh.primitive_cube_add(
        size=1.0, location=(0, 0, 0.0035),
        scale=(0.010, 0.010, 0.002)
    )
    chip = bpy.context.active_object
    chip.name = f"{name}_MEMS_Chip"
    chip.parent = root
    if materials and 'Chip_IC' in materials:
        chip.data.materials.append(materials['Chip_IC'])
    collection.objects.link(chip)
    bpy.context.scene.collection.objects.unlink(chip)
    
    for led_name, loc, mat_name in [
        (f"{name}_LED_Power", (0.014, 0.012, 0.003), 'LED_Green'),
        (f"{name}_LED_Data", (0.014, 0.006, 0.003), 'LED_Amber')
    ]:
        bpy.ops.mesh.primitive_cube_add(
            size=1.0, location=loc,
            scale=(0.003, 0.002, 0.0015)
        )
        led = bpy.context.active_object
        led.name = led_name
        led.parent = root
        if materials and mat_name in materials:
            led.data.materials.append(materials[mat_name])
        collection.objects.link(led)
        bpy.context.scene.collection.objects.unlink(led)
        
    # 3D Extruded Text Label "IMU 6-DOF"
    bpy.ops.object.text_add(location=(-0.018, -0.012, 0.0028))
    text_obj = bpy.context.active_object
    text_obj.name = f"{name}_Label_Text"
    text_obj.data.body = "IMU 6-DOF"
    text_obj.data.size = 0.0055
    text_obj.data.extrude = 0.001
    text_obj.parent = root
    if materials and 'Text_White' in materials:
        text_obj.data.materials.append(materials['Text_White'])
    collection.objects.link(text_obj)
    bpy.context.scene.collection.objects.unlink(text_obj)
    
    # Miniature axis arrows
    axis_l = 0.015
    for aname, vec, mname in [
        ("IMU_Axis_X", (axis_l, 0, 0), 'Axis_X_Red'),
        ("IMU_Axis_Y", (0, axis_l, 0), 'Axis_Y_Green'),
        ("IMU_Axis_Z", (0, 0, axis_l), 'Axis_Z_Blue')
    ]:
        bpy.ops.mesh.primitive_cylinder_add(
            vertices=8, radius=0.0008, depth=axis_l,
            location=(vec[0]/2, vec[1]/2, 0.004 + vec[2]/2)
        )
        ax = bpy.context.active_object
        ax.name = f"{name}_{aname}"
        ax.parent = root
        if vec[0] != 0:
            ax.rotation_euler = (0, math.pi/2, 0)
        elif vec[1] != 0:
            ax.rotation_euler = (-math.pi/2, 0, 0)
        if materials and mname in materials:
            ax.data.materials.append(materials[mname])
        collection.objects.link(ax)
        bpy.context.scene.collection.objects.unlink(ax)
        
    return root

def build_ultrasonic_sensor(sensor_name, collection=None, materials=None):
    """Build HC-SR04 ultrasonic sensor with dual transducers and visible detection ray."""
    root = bpy.data.objects.new(sensor_name, None)
    root.empty_display_type = 'PLAIN_AXES'
    root.empty_display_size = 0.035
    if collection:
        collection.objects.link(root)
        
    bpy.ops.mesh.primitive_cube_add(
        size=1.0, location=(0, 0, 0),
        scale=(0.052, 0.002, 0.024)
    )
    pcb = bpy.context.active_object
    pcb.name = f"{sensor_name}_PCB"
    pcb.parent = root
    if materials and 'PCB_Blue' in materials:
        pcb.data.materials.append(materials['PCB_Blue'])
    collection.objects.link(pcb)
    bpy.context.scene.collection.objects.unlink(pcb)
    
    trans_spacing = 0.015
    for i, side_name in [(-1, "Transmitter"), (1, "Receiver")]:
        bpy.ops.mesh.primitive_cylinder_add(
            vertices=24, radius=0.009, depth=0.014,
            location=(i * trans_spacing, 0.008, 0),
            rotation=(math.pi/2, 0, 0)
        )
        can = bpy.context.active_object
        can.name = f"{sensor_name}_{side_name}_Can"
        can.parent = root
        if materials and 'Ultrasonic_Can' in materials:
            can.data.materials.append(materials['Ultrasonic_Can'])
        collection.objects.link(can)
        bpy.context.scene.collection.objects.unlink(can)
        
        bpy.ops.mesh.primitive_cylinder_add(
            vertices=20, radius=0.0078, depth=0.0015,
            location=(i * trans_spacing, 0.0152, 0),
            rotation=(math.pi/2, 0, 0)
        )
        mesh_dot = bpy.context.active_object
        mesh_dot.name = f"{sensor_name}_{side_name}_Mesh"
        mesh_dot.parent = root
        if materials and 'Ultrasonic_Mesh' in materials:
            mesh_dot.data.materials.append(materials['Ultrasonic_Mesh'])
        collection.objects.link(mesh_dot)
        bpy.context.scene.collection.objects.unlink(mesh_dot)
        
    # Dynamic Visible Detection Ray Mesh
    beam_mesh = bpy.data.meshes.new(f"{sensor_name}_Beam_Mesh_Data")
    beam_obj = bpy.data.objects.new(f"{sensor_name}_Beam_Mesh", beam_mesh)
    collection.objects.link(beam_obj)
    beam_obj.parent = root
    
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=20, radius=0.014, depth=1.0,
        location=(0, 0, 0)
    )
    temp = bpy.context.active_object
    beam_obj.data = temp.data
    bpy.data.objects.remove(temp, do_unlink=True)
    
    beam_obj.rotation_euler = (math.pi/2, 0, 0)
    beam_obj.location = (0, 0.5 + 0.016, 0)
    
    if materials and 'Beam_Safe' in materials:
        beam_obj.data.materials.append(materials['Beam_Safe'])
        
    return root

def build_electronics_pack(collection=None, materials=None):
    """Build LiPo battery, main robotics controller board, and motor driver."""
    root = bpy.data.objects.new("Electronics_Assembly", None)
    if collection:
        collection.objects.link(root)
        
    bpy.ops.mesh.primitive_cube_add(
        size=1.0, location=(0, -0.05, 0.022),
        scale=(0.105, 0.045, 0.032)
    )
    bat = bpy.context.active_object
    bat.name = "Battery_Pack_3S"
    bat.parent = root
    if materials and 'Battery' in materials:
        bat.data.materials.append(materials['Battery'])
    collection.objects.link(bat)
    bpy.context.scene.collection.objects.unlink(bat)
    
    bpy.ops.mesh.primitive_cube_add(
        size=1.0, location=(0, 0.04, 0.010),
        scale=(0.110, 0.075, 0.003)
    )
    mcu = bpy.context.active_object
    mcu.name = "Controller_Board"
    mcu.parent = root
    if materials and 'PCB_Green' in materials:
        mcu.data.materials.append(materials['PCB_Green'])
    collection.objects.link(mcu)
    bpy.context.scene.collection.objects.unlink(mcu)
    
    bpy.ops.mesh.primitive_cube_add(
        size=1.0, location=(0, 0.04, 0.026),
        scale=(0.040, 0.035, 0.024)
    )
    heatsink = bpy.context.active_object
    heatsink.name = "Motor_Driver_Heatsink"
    heatsink.parent = root
    if materials and 'Gearbox' in materials:
        heatsink.data.materials.append(materials['Gearbox'])
    collection.objects.link(heatsink)
    bpy.context.scene.collection.objects.unlink(heatsink)
    
    return root

# ---------------------------------------------------------------------------
# ROBOT COMPLETE ASSEMBLY
# ---------------------------------------------------------------------------
def build_robot_model(collections, materials):
    """Assemble complete robot model into Master-specified collections."""
    robot_root = bpy.data.objects.new("Robot_Base", None)
    robot_root.empty_display_type = 'ARROWS'
    robot_root.empty_display_size = 0.45
    collections['Robot_Chassis'].objects.link(robot_root)
    
    # 1. Dual-deck chassis
    bot_deck = build_triangular_plate(
        "Chassis_Lower_Plate", radius=CHASSIS_RADIUS, thickness=PLATE_THICKNESS,
        corner_radius=0.055, collection=collections['Robot_Chassis'], material=materials['Chassis_Plate']
    )
    bot_deck.location = (0, 0, GROUND_CLEARANCE + PLATE_THICKNESS/2)
    bot_deck.parent = robot_root
    
    top_deck = build_triangular_plate(
        "Chassis_Upper_Plate", radius=CHASSIS_RADIUS * 0.94, thickness=PLATE_THICKNESS,
        corner_radius=0.050, collection=collections['Robot_Chassis'], material=materials['Chassis_Plate']
    )
    top_deck.location = (0, 0, GROUND_CLEARANCE + DECK_GAP + PLATE_THICKNESS/2)
    top_deck.parent = robot_root
    
    # 2. Six hex standoffs
    standoff_angles = [math.radians(a) for a in [30, 90, 150, 210, 270, 330]]
    standoff_r = CHASSIS_RADIUS * 0.65
    for i, sa in enumerate(standoff_angles):
        sx = standoff_r * math.cos(sa)
        sy = standoff_r * math.sin(sa)
        bpy.ops.mesh.primitive_cylinder_add(
            vertices=6, radius=0.0055, depth=DECK_GAP,
            location=(sx, sy, GROUND_CLEARANCE + DECK_GAP/2)
        )
        post = bpy.context.active_object
        post.name = f"Chassis_Standoff_{i+1}"
        post.parent = robot_root
        if 'Standoff' in materials:
            post.data.materials.append(materials['Standoff'])
        collections['Robot_Chassis'].objects.link(post)
        bpy.context.scene.collection.objects.unlink(post)
        
    # 3. Electronics package
    elec = build_electronics_pack(collections['Robot_Chassis'], materials)
    elec.location = (0, 0, GROUND_CLEARANCE + PLATE_THICKNESS)
    elec.parent = robot_root
    
    # 4. Labelled IMU module
    imu = build_labelled_imu("IMU_Module", collections['IMU'], materials)
    imu.location = (0, 0, GROUND_CLEARANCE + DECK_GAP + PLATE_THICKNESS)
    imu.parent = robot_root
    
    # 5. Exactly 3 Drive Units (Omni Wheel + Motor + Encoder) at 0°, 120°, 240°
    wheel_angles_deg = [0, 120, 240]
    wheels = []
    motors = []
    encoders = []
    
    for idx, deg in enumerate(wheel_angles_deg):
        rad = math.radians(deg)
        w_name = f"Omni_Wheel_{idx+1}"
        m_name = f"Drive_Motor_{idx+1}"
        e_name = f"Encoder_{idx+1}"
        
        wx = WHEEL_DIST * math.cos(rad)
        wy = WHEEL_DIST * math.sin(rad)
        wz = WHEEL_RADIUS
        
        wheel = build_omni_wheel(w_name, radius=WHEEL_RADIUS, width=WHEEL_WIDTH, num_rollers=NUM_ROLLERS, 
                                 collection=collections['Omni_Wheels'], materials=materials)
        wheel.location = (wx, wy, wz)
        wheel.rotation_euler = (0, 0, rad)
        wheel.parent = robot_root
        wheels.append(wheel)
        
        motor_dist = 0.205
        mx = motor_dist * math.cos(rad)
        my = motor_dist * math.sin(rad)
        mz = WHEEL_RADIUS
        
        motor = build_motor(m_name, collection=collections['Motors'], materials=materials)
        motor.location = (mx, my, mz)
        motor.rotation_euler = (0, 0, rad)
        motor.parent = robot_root
        motors.append(motor)
        
        enc_dist = 0.142
        ex = enc_dist * math.cos(rad)
        ey = enc_dist * math.sin(rad)
        ez = WHEEL_RADIUS
        
        encoder = build_encoder(e_name, collection=collections['Encoders'], materials=materials)
        encoder.location = (ex, ey, ez)
        encoder.rotation_euler = (0, 0, rad)
        encoder.parent = robot_root
        encoders.append(encoder)

    # 6. Three Ultrasonic Sensors (Front=0°, Left=+90°, Right=-90°)
    us_mount_r = CHASSIS_RADIUS * 0.90
    us_configs = [
        ("US_Sensor_Front", (us_mount_r, 0.0, GROUND_CLEARANCE + DECK_GAP + 0.030), 0.0),
        ("US_Sensor_Left", (0.0, us_mount_r, GROUND_CLEARANCE + DECK_GAP + 0.030), math.pi/2),
        ("US_Sensor_Right", (0.0, -us_mount_r, GROUND_CLEARANCE + DECK_GAP + 0.030), -math.pi/2)
    ]
    
    us_sensors = []
    for s_name, loc, yaw in us_configs:
        us_obj = build_ultrasonic_sensor(s_name, collection=collections['Ultrasonic_Sensors'], materials=materials)
        us_obj.location = loc
        us_obj.rotation_euler = (0, 0, yaw - math.pi/2)
        us_obj.parent = robot_root
        us_sensors.append(us_obj)
        
    return robot_root, wheels, motors, encoders, us_sensors

# ---------------------------------------------------------------------------
# ARENA & TESTING ENVIRONMENT
# ---------------------------------------------------------------------------
def build_environment(collections, materials):
    """Build arena with grid floor, boundary walls, Start Pad, Goal Pad, axes, and 4 obstacles."""
    arena_size = 10.0
    wall_height = 0.85
    wall_thk = 0.18
    
    # 1. Floor
    bpy.ops.mesh.primitive_plane_add(size=arena_size, location=(0, 0, 0))
    floor = bpy.context.active_object
    floor.name = "Arena_Floor"
    if 'Arena_Floor' in materials:
        floor.data.materials.append(materials['Arena_Floor'])
    collections['Environment'].objects.link(floor)
    bpy.context.scene.collection.objects.unlink(floor)
    
    # 2. Boundary Walls with hazard stripes
    half = arena_size / 2.0
    wall_defs = [
        ("Wall_North", (0, half, wall_height/2), (arena_size, wall_thk, wall_height)),
        ("Wall_South", (0, -half, wall_height/2), (arena_size, wall_thk, wall_height)),
        ("Wall_East", (half, 0, wall_height/2), (wall_thk, arena_size, wall_height)),
        ("Wall_West", (-half, 0, wall_height/2), (wall_thk, arena_size, wall_height))
    ]
    for w_name, loc, sz in wall_defs:
        bpy.ops.mesh.primitive_cube_add(size=1.0, location=loc, scale=sz)
        wall = bpy.context.active_object
        wall.name = w_name
        if 'Wall_Concrete' in materials:
            wall.data.materials.append(materials['Wall_Concrete'])
        collections['Environment'].objects.link(wall)
        bpy.context.scene.collection.objects.unlink(wall)
        
        bpy.ops.mesh.primitive_cube_add(
            size=1.0, location=(loc[0], loc[1], wall_height - 0.05),
            scale=(sz[0] * 1.002, sz[1] * 1.002, 0.10)
        )
        stripe = bpy.context.active_object
        stripe.name = f"{w_name}_Hazard_Stripe"
        if 'Hazard_Stripe' in materials:
            stripe.data.materials.append(materials['Hazard_Stripe'])
        collections['Environment'].objects.link(stripe)
        bpy.context.scene.collection.objects.unlink(stripe)

    # 3. Robot Starting Launch Pad
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=48, radius=0.75, depth=0.002,
        location=(0, 0, 0.001)
    )
    start_pad = bpy.context.active_object
    start_pad.name = "Start_Launch_Pad"
    if 'Start_Pad' in materials:
        start_pad.data.materials.append(materials['Start_Pad'])
    collections['Environment'].objects.link(start_pad)
    bpy.context.scene.collection.objects.unlink(start_pad)
    
    # 4. Robot Goal Pad at Destination (X=2.8m, Y=2.8m)
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=48, radius=0.75, depth=0.002,
        location=(2.80, 2.80, 0.001)
    )
    goal_pad = bpy.context.active_object
    goal_pad.name = "Goal_Destination_Pad"
    if 'Goal_Pad' in materials:
        goal_pad.data.materials.append(materials['Goal_Pad'])
    collections['Environment'].objects.link(goal_pad)
    bpy.context.scene.collection.objects.unlink(goal_pad)
    
    # 5. Global Coordinate Axes at Reference Station Corner (-4.0, -4.0, 0.0)
    ref_origin = (-4.0, -4.0, 0.0)
    axis_l = 1.0
    for aname, vec, mname in [
        ("Arena_Axis_X_Forward", (axis_l, 0, 0), 'Axis_X_Red'),
        ("Arena_Axis_Y_Lateral", (0, axis_l, 0), 'Axis_Y_Green'),
        ("Arena_Axis_Z_Vertical", (0, 0, axis_l), 'Axis_Z_Blue')
    ]:
        bpy.ops.mesh.primitive_cylinder_add(
            vertices=16, radius=0.015, depth=axis_l,
            location=(ref_origin[0] + vec[0]/2, ref_origin[1] + vec[1]/2, ref_origin[2] + 0.008 + vec[2]/2)
        )
        ax = bpy.context.active_object
        ax.name = aname
        if vec[0] != 0:
            ax.rotation_euler = (0, math.pi/2, 0)
        elif vec[1] != 0:
            ax.rotation_euler = (-math.pi/2, 0, 0)
        if mname in materials:
            ax.data.materials.append(materials[mname])
        collections['Environment'].objects.link(ax)
        bpy.context.scene.collection.objects.unlink(ax)

    # 6. Four Arena Obstacles in dedicated 'Obstacles' collection
    # Obstacle 1: Front Cylindrical Pillar (Target for Forward Ultrasonic Detection)
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=32, radius=0.38, depth=1.30,
        location=(2.50, 0.00, 0.65)
    )
    obs1 = bpy.context.active_object
    obs1.name = "Obstacle_Pillar_Front"
    if 'Obstacle_Orange' in materials:
        obs1.data.materials.append(materials['Obstacle_Orange'])
    collections['Obstacles'].objects.link(obs1)
    bpy.context.scene.collection.objects.unlink(obs1)
    
    # Obstacle 2: Lateral Rectangular Barrier
    bpy.ops.mesh.primitive_cube_add(
        size=1.0, location=(1.30, 2.30, 0.45),
        scale=(0.75, 1.40, 0.90)
    )
    obs2 = bpy.context.active_object
    obs2.name = "Obstacle_Crate_Lateral"
    if 'Obstacle_Teal' in materials:
        obs2.data.materials.append(materials['Obstacle_Teal'])
    collections['Obstacles'].objects.link(obs2)
    bpy.context.scene.collection.objects.unlink(obs2)
    
    # Obstacle 3: Diagonal Partition Wall
    bpy.ops.mesh.primitive_cube_add(
        size=1.0, location=(-1.60, -1.90, 0.45),
        scale=(2.0, 0.35, 0.90)
    )
    obs3 = bpy.context.active_object
    obs3.name = "Obstacle_Partition_Wall"
    if 'Obstacle_Metal' in materials:
        obs3.data.materials.append(materials['Obstacle_Metal'])
    collections['Obstacles'].objects.link(obs3)
    bpy.context.scene.collection.objects.unlink(obs3)
    
    # Obstacle 4: Corner Column
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=24, radius=0.50, depth=1.50,
        location=(-2.60, 2.60, 0.75)
    )
    obs4 = bpy.context.active_object
    obs4.name = "Obstacle_Corner_Pillar"
    if 'Obstacle_Orange' in materials:
        obs4.data.materials.append(materials['Obstacle_Orange'])
    collections['Obstacles'].objects.link(obs4)
    bpy.context.scene.collection.objects.unlink(obs4)

# ---------------------------------------------------------------------------
# LIGHTING & CAMERAS
# ---------------------------------------------------------------------------
def setup_lighting(collection):
    """Setup four-point arena studio lighting."""
    sun_data = bpy.data.lights.new(name="Key_Sun_Light", type='SUN')
    sun_data.energy = 4.5
    sun_data.color = (1.0, 0.98, 0.95)
    sun_obj = bpy.data.objects.new("Key_Sun_Light", sun_data)
    sun_obj.location = (5.0, -4.0, 8.0)
    sun_obj.rotation_euler = (math.radians(48), math.radians(24), math.radians(-32))
    collection.objects.link(sun_obj)
    
    area_data = bpy.data.lights.new(name="Overhead_Softbox", type='AREA')
    area_data.energy = 400.0
    area_data.size = 8.5
    area_data.color = (0.92, 0.96, 1.0)
    area_obj = bpy.data.objects.new("Overhead_Softbox", area_data)
    area_obj.location = (0, 0, 5.8)
    collection.objects.link(area_obj)
    
    rim_data = bpy.data.lights.new(name="Rim_Light", type='POINT')
    rim_data.energy = 220.0
    rim_data.color = (0.2, 0.75, 1.0)
    rim_obj = bpy.data.objects.new("Rim_Light", rim_data)
    rim_obj.location = (-4.0, 3.8, 2.8)
    collection.objects.link(rim_obj)
    
    fill_data = bpy.data.lights.new(name="Fill_Light", type='POINT')
    fill_data.energy = 160.0
    fill_data.color = (1.0, 0.90, 0.80)
    fill_obj = bpy.data.objects.new("Fill_Light", fill_data)
    fill_obj.location = (-3.0, -3.5, 3.5)
    collection.objects.link(fill_obj)

def setup_cameras(collection, target_obj):
    """Setup 4 specialized project cameras."""
    cam_close_data = bpy.data.cameras.new("Camera_Closeup")
    cam_close_data.lens = 58
    cam_close_obj = bpy.data.objects.new("Camera_Closeup", cam_close_data)
    cam_close_obj.location = (0.72, -0.72, 0.48)
    cam_close_obj.rotation_euler = (math.radians(64), 0, math.radians(45))
    collection.objects.link(cam_close_obj)
    
    cam_pres_data = bpy.data.cameras.new("Camera_Presentation")
    cam_pres_data.lens = 42
    cam_pres_obj = bpy.data.objects.new("Camera_Presentation", cam_pres_data)
    cam_pres_obj.location = (1.85, -2.10, 1.45)
    cam_pres_obj.rotation_euler = (math.radians(62), 0, math.radians(40))
    collection.objects.link(cam_pres_obj)
    
    cam_top_data = bpy.data.cameras.new("Camera_TopDown")
    cam_top_data.type = 'ORTHO'
    cam_top_data.ortho_scale = 6.8
    cam_top_obj = bpy.data.objects.new("Camera_TopDown", cam_top_data)
    cam_top_obj.location = (0.80, 0.50, 8.5)
    cam_top_obj.rotation_euler = (0, 0, math.radians(-90))
    collection.objects.link(cam_top_obj)
    
    cam_fol_data = bpy.data.cameras.new("Camera_Follow")
    cam_fol_data.lens = 36
    cam_fol_obj = bpy.data.objects.new("Camera_Follow", cam_fol_data)
    cam_fol_obj.location = (1.6, -1.6, 1.3)
    collection.objects.link(cam_fol_obj)
    
    tt = cam_fol_obj.constraints.new(type='TRACK_TO')
    tt.target = target_obj
    tt.track_axis = 'TRACK_NEGATIVE_Z'
    tt.up_axis = 'UP_Y'
    
    bpy.context.scene.camera = cam_pres_obj
    return cam_close_obj, cam_pres_obj, cam_top_obj, cam_fol_obj

def configure_render_settings():
    """Configure render settings."""
    scene = bpy.context.scene
    scene.render.resolution_x = 1280
    scene.render.resolution_y = 720
    scene.render.resolution_percentage = 100
    scene.render.fps = 30
    scene.render.engine = 'CYCLES'
    scene.cycles.device = 'CPU'
    scene.cycles.samples = 16
    scene.cycles.use_denoising = False

# ---------------------------------------------------------------------------
# MAIN EXECUTION
# ---------------------------------------------------------------------------
def main():
    print("=" * 65)
    print("GROUP 7: BUILDING THREE-WHEEL OMNI ROBOT & ARENA (MASTER SPEC)")
    print("=" * 65)
    
    clear_existing_scene()
    
    # Collections strictly named to Master Specification:
    # Robot_Chassis, Omni_Wheels, Motors, Encoders, IMU, Ultrasonic_Sensors, Environment, Obstacles, Cameras
    col_chassis = create_collection("Robot_Chassis")
    col_wheels = create_collection("Omni_Wheels")
    col_motors = create_collection("Motors")
    col_encoders = create_collection("Encoders")
    col_imu = create_collection("IMU")
    col_us = create_collection("Ultrasonic_Sensors")
    col_env = create_collection("Environment")
    col_obstacles = create_collection("Obstacles")
    col_cameras = create_collection("Cameras")
    col_lights = create_collection("Lighting")
    
    collections_map = {
        'Robot_Chassis': col_chassis,
        'Omni_Wheels': col_wheels,
        'Motors': col_motors,
        'Encoders': col_encoders,
        'IMU': col_imu,
        'Ultrasonic_Sensors': col_us,
        'Environment': col_env,
        'Obstacles': col_obstacles,
        'Cameras': col_cameras
    }
    
    materials = setup_materials()
    print("[1/5] PBR Materials created successfully.")
    
    robot_root, wheels, motors, encoders, us_sensors = build_robot_model(collections_map, materials)
    print(f"[2/5] Robot model built with:")
    print(f"      - {len(wheels)} 120° omni wheels with {NUM_ROLLERS} realistic passive rollers each")
    print(f"      - {len(motors)} separate drive motor assemblies")
    print(f"      - {len(encoders)} separate optical encoder modules")
    print(f"      - Labelled 6-DOF IMU module at chassis center ('IMU 6-DOF')")
    print(f"      - {len(us_sensors)} ultrasonic sensor modules with visible detection rays")
    
    build_environment(collections_map, materials)
    print("[3/5] Testing arena (Start Pad, Goal Pad, boundary walls, 4 obstacles) built.")
    
    setup_lighting(col_lights)
    cam_close, cam_pres, cam_top, cam_fol = setup_cameras(col_cameras, robot_root)
    configure_render_settings()
    print("[4/5] Multi-point lighting, 4 cameras (Closeup, Presentation, TopDown, Follow) configured.")
    
    output_dir = os.path.dirname(os.path.abspath(__file__))
    blend_path = os.path.join(output_dir, "Group7_OmniRobot.blend")
    bpy.ops.wm.save_as_mainfile(filepath=blend_path)
    print(f"[5/5] Complete project scene saved to: {blend_path}")
    print("=" * 65)
    print("MASTER SCENE GENERATION COMPLETED SUCCESSFULLY!")
    print("=" * 65)

if __name__ == "__main__":
    main()
