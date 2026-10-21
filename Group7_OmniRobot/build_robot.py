"""
Group 7 - Three-Wheel Omnidirectional Robot
Blender 3D Modeling and Scene Setup Script (Enhanced)
=====================================================
Authors: Group 7 Robotics Engineering Team
Project: Three-Wheel Omni Robot — Holonomic Movement in Indoor Environments

This script builds:
1. High-detail 3-wheel omnidirectional mobile robot with:
   - Triangular dual-deck chassis with chamfered edges and aluminum standoffs
   - 3 independently driven DC gearmotors with planetary gearboxes and mounting brackets
   - 3 optical rotary encoders attached to motor rear shafts
   - Central IMU sensor module with SMD components
   - 3 HC-SR04 ultrasonic sensors (Front, Left, Right) with dual transducers
   - Battery pack, motor drivers, controller board, wiring, and fasteners
   - 3 double-disc omni wheels with 10 passive barrel rollers each, placed 120 deg apart
2. Testing arena (10x10m) with grid floor, boundary walls, hazard stripes, coordinate axes
   placed in corner reference station, starting pad, and 4 navigation obstacles.
3. Materials with realistic PBR shaders AND viewport/workbench diffuse colors.
4. 4 dedicated cameras:
   - Camera_Closeup (for macro presentation of the robot)
   - Camera_Presentation (angled 3D arena view)
   - Camera_TopDown (orthographic kinematics & trajectory view)
   - Camera_Follow (tracking follow cam)
5. Saves the scene as Group7_OmniRobot.blend.
"""

import bpy
import math
import os
from mathutils import Vector, Euler

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

def get_or_create_material(name, base_color, metallic=0.0, roughness=0.5, emission_color=None, emission_strength=0.0):
    """Create a Principled BSDF material with specified PBR and Viewport properties."""
    if name in bpy.data.materials:
        return bpy.data.materials[name]
    
    mat = bpy.data.materials.new(name=name)
    mat.diffuse_color = base_color # For Workbench / Solid viewport rendering
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
    bsdf.inputs['Roughness'].default_value = 0.25
    bsdf.inputs['Metallic'].default_value = 0.1
    
    tex_coord = nodes.new(type='ShaderNodeTexCoord')
    tex_coord.location = (-400, 0)
    
    grid_tex = nodes.new(type='ShaderNodeTexBrick')
    grid_tex.location = (-100, 0)
    grid_tex.inputs['Color1'].default_value = (0.07, 0.08, 0.10, 1.0)
    grid_tex.inputs['Color2'].default_value = (0.05, 0.06, 0.08, 1.0)
    grid_tex.inputs['Mortar'].default_value = (0.25, 0.40, 0.65, 1.0)
    grid_tex.inputs['Scale'].default_value = 5.0
    grid_tex.inputs['Mortar Size'].default_value = 0.015
    grid_tex.inputs['Mortar Smooth'].default_value = 0.1
    grid_tex.inputs['Row Height'].default_value = 1.0
    
    mat.node_tree.links.new(tex_coord.outputs['Object'], grid_tex.inputs['Vector'])
    mat.node_tree.links.new(grid_tex.outputs['Color'], bsdf.inputs['Base Color'])
    mat.node_tree.links.new(bsdf.outputs['BSDF'], out.inputs['Surface'])
    return mat

def setup_materials():
    """Setup all project materials."""
    materials = {}
    materials['Chassis_Plate'] = get_or_create_material('Mat_Chassis_Plate', (0.04, 0.04, 0.05, 1.0), metallic=0.8, roughness=0.25)
    materials['Standoff'] = get_or_create_material('Mat_Standoff', (0.8, 0.8, 0.85, 1.0), metallic=0.95, roughness=0.15)
    materials['Fastener'] = get_or_create_material('Mat_Fastener', (0.9, 0.9, 0.95, 1.0), metallic=1.0, roughness=0.1)
    
    materials['Motor_Body'] = get_or_create_material('Mat_Motor_Body', (0.15, 0.16, 0.18, 1.0), metallic=0.7, roughness=0.3)
    materials['Gearbox'] = get_or_create_material('Mat_Gearbox', (0.75, 0.77, 0.80, 1.0), metallic=0.9, roughness=0.2)
    materials['Motor_Bracket'] = get_or_create_material('Mat_Motor_Bracket', (0.05, 0.35, 0.65, 1.0), metallic=0.85, roughness=0.3)
    materials['Shaft'] = get_or_create_material('Mat_Shaft', (0.85, 0.85, 0.88, 1.0), metallic=0.95, roughness=0.1)
    
    materials['Wheel_Hub'] = get_or_create_material('Mat_Wheel_Hub', (0.85, 0.08, 0.08, 1.0), metallic=0.85, roughness=0.22)
    materials['Roller_Rubber'] = get_or_create_material('Mat_Roller_Rubber', (0.05, 0.05, 0.05, 1.0), metallic=0.0, roughness=0.85)
    materials['Roller_Pin'] = get_or_create_material('Mat_Roller_Pin', (0.9, 0.9, 0.9, 1.0), metallic=1.0, roughness=0.1)
    
    materials['PCB_Green'] = get_or_create_material('Mat_PCB_Green', (0.02, 0.45, 0.12, 1.0), metallic=0.1, roughness=0.4)
    materials['PCB_Blue'] = get_or_create_material('Mat_PCB_Blue', (0.05, 0.25, 0.85, 1.0), metallic=0.1, roughness=0.35)
    materials['Ultrasonic_Can'] = get_or_create_material('Mat_Ultrasonic_Can', (0.88, 0.89, 0.92, 1.0), metallic=0.95, roughness=0.15)
    materials['Ultrasonic_Mesh'] = get_or_create_material('Mat_Ultrasonic_Mesh', (0.08, 0.08, 0.08, 1.0), metallic=0.2, roughness=0.9)
    materials['Chip_IC'] = get_or_create_material('Mat_Chip_IC', (0.02, 0.02, 0.02, 1.0), metallic=0.1, roughness=0.4)
    materials['LED_Green'] = get_or_create_material('Mat_LED_Green', (0.1, 1.0, 0.2, 1.0), metallic=0.0, roughness=0.1, 
                                                    emission_color=(0.1, 1.0, 0.2, 1.0), emission_strength=5.0)
    materials['LED_Red'] = get_or_create_material('Mat_LED_Red', (1.0, 0.05, 0.05, 1.0), metallic=0.0, roughness=0.1, 
                                                  emission_color=(1.0, 0.05, 0.05, 1.0), emission_strength=5.0)
    materials['Battery'] = get_or_create_material('Mat_Battery', (0.08, 0.50, 0.95, 1.0), metallic=0.2, roughness=0.4)
    materials['Wire_Red'] = get_or_create_material('Mat_Wire_Red', (0.85, 0.05, 0.05, 1.0), metallic=0.0, roughness=0.5)
    materials['Wire_Black'] = get_or_create_material('Mat_Wire_Black', (0.03, 0.03, 0.03, 1.0), metallic=0.0, roughness=0.5)
    
    materials['Arena_Floor'] = create_grid_material('Mat_Arena_Floor')
    materials['Wall_Concrete'] = get_or_create_material('Mat_Wall_Concrete', (0.40, 0.42, 0.45, 1.0), metallic=0.0, roughness=0.8)
    materials['Hazard_Stripe'] = get_or_create_material('Mat_Hazard_Stripe', (0.95, 0.80, 0.05, 1.0), metallic=0.1, roughness=0.4)
    materials['Obstacle_Orange'] = get_or_create_material('Mat_Obstacle_Orange', (0.95, 0.35, 0.05, 1.0), metallic=0.1, roughness=0.35)
    materials['Obstacle_Teal'] = get_or_create_material('Mat_Obstacle_Teal', (0.05, 0.65, 0.70, 1.0), metallic=0.1, roughness=0.35)
    materials['Obstacle_Metal'] = get_or_create_material('Mat_Obstacle_Metal', (0.45, 0.48, 0.52, 1.0), metallic=0.85, roughness=0.25)
    materials['Start_Pad'] = get_or_create_material('Mat_Start_Pad', (0.12, 0.50, 0.20, 1.0), metallic=0.2, roughness=0.5)
    
    materials['Beam_Safe'] = get_or_create_material('Mat_Beam_Safe', (0.0, 1.0, 0.4, 0.8), metallic=0.0, roughness=0.2,
                                                    emission_color=(0.0, 1.0, 0.4, 1.0), emission_strength=4.0)
    materials['Beam_Alert'] = get_or_create_material('Mat_Beam_Alert', (1.0, 0.1, 0.1, 0.9), metallic=0.0, roughness=0.2,
                                                     emission_color=(1.0, 0.1, 0.1, 1.0), emission_strength=8.0)
    
    materials['Axis_X_Red'] = get_or_create_material('Mat_Axis_X_Red', (1.0, 0.05, 0.05, 1.0), emission_color=(1.0, 0.05, 0.05, 1.0), emission_strength=3.0)
    materials['Axis_Y_Green'] = get_or_create_material('Mat_Axis_Y_Green', (0.05, 1.0, 0.05, 1.0), emission_color=(0.05, 1.0, 0.05, 1.0), emission_strength=3.0)
    materials['Axis_Z_Blue'] = get_or_create_material('Mat_Axis_Z_Blue', (0.1, 0.2, 1.0, 1.0), emission_color=(0.1, 0.2, 1.0, 1.0), emission_strength=3.0)
    
    return materials

def build_triangular_plate(name, radius=0.28, thickness=0.005, corner_radius=0.04, collection=None, material=None):
    """Build a rounded triangular chassis deck with weight-reduction cutouts."""
    mesh = bpy.data.meshes.new(name + "_Mesh")
    obj = bpy.data.objects.new(name, mesh)
    if collection:
        collection.objects.link(obj)
    
    verts = []
    faces = []
    
    num_sub = 8
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
    bev.width = 0.002
    bev.segments = 2
    
    return obj

def build_omni_wheel(wheel_name, radius=0.04, width=0.024, num_rollers=10, collection=None, materials=None):
    """
    Build a detailed omni wheel assembly:
    - Central hub with dual anodized aluminum rim plates
    - Central motor shaft mounting hub / coupler
    - Periphery barrel rollers mounted on miniature steel pins
    """
    wheel_empty = bpy.data.objects.new(wheel_name, None)
    wheel_empty.empty_display_type = 'ARROWS'
    wheel_empty.empty_display_size = 0.04
    if collection:
        collection.objects.link(wheel_empty)
        
    plate_thk = 0.0025
    half_w = width / 2.0
    for side, sign in [("L", -1), ("R", 1)]:
        disc_mesh = bpy.data.meshes.new(f"{wheel_name}_Plate_{side}_Mesh")
        disc_obj = bpy.data.objects.new(f"{wheel_name}_Plate_{side}", disc_mesh)
        collection.objects.link(disc_obj)
        disc_obj.parent = wheel_empty
        
        bpy.ops.mesh.primitive_cylinder_add(
            vertices=32, radius=radius * 0.78, depth=plate_thk,
            location=(0, sign * (half_w - plate_thk/2), 0),
            rotation=(math.pi/2, 0, 0)
        )
        temp_obj = bpy.context.active_object
        disc_obj.data = temp_obj.data
        bpy.data.objects.remove(temp_obj, do_unlink=True)
        
        if materials and 'Wheel_Hub' in materials:
            disc_obj.data.materials.append(materials['Wheel_Hub'])
            
    # Central collar
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=24, radius=0.012, depth=width * 1.1,
        location=(0, 0, 0), rotation=(math.pi/2, 0, 0)
    )
    collar = bpy.context.active_object
    collar.name = f"{wheel_name}_Collar"
    collar.parent = wheel_empty
    if materials and 'Gearbox' in materials:
        collar.data.materials.append(materials['Gearbox'])
    collection.objects.link(collar)
    bpy.context.scene.collection.objects.unlink(collar)
    
    # Passive barrel rollers around circumference
    roller_orbit_r = radius * 0.82
    roller_rad = radius * 0.22
    roller_len = (2 * math.pi * roller_orbit_r) / (num_rollers * 1.45)
    
    for i in range(num_rollers):
        ang = (2 * math.pi * i) / num_rollers
        row_offset = (half_w * 0.45) if (i % 2 == 0) else (-half_w * 0.45)
        
        rx = roller_orbit_r * math.cos(ang)
        ry = row_offset
        rz = roller_orbit_r * math.sin(ang)
        
        tangent_ang = ang + math.pi/2
        
        bpy.ops.mesh.primitive_cylinder_add(
            vertices=16, radius=roller_rad, depth=roller_len,
            location=(rx, ry, rz),
            rotation=(math.sin(tangent_ang)*math.pi/2, 0, -math.cos(tangent_ang)*math.pi/2)
        )
        roller = bpy.context.active_object
        roller.name = f"{wheel_name}_Roller_{i+1}"
        roller.parent = wheel_empty
        if materials and 'Roller_Rubber' in materials:
            roller.data.materials.append(materials['Roller_Rubber'])
            
        bev = roller.modifiers.new("Bevel", 'BEVEL')
        bev.width = roller_rad * 0.45
        bev.segments = 3
        
        collection.objects.link(roller)
        bpy.context.scene.collection.objects.unlink(roller)
        
        # Axle pin
        bpy.ops.mesh.primitive_cylinder_add(
            vertices=8, radius=0.0018, depth=roller_len * 1.25,
            location=(rx, ry, rz),
            rotation=(math.sin(tangent_ang)*math.pi/2, 0, -math.cos(tangent_ang)*math.pi/2)
        )
        pin = bpy.context.active_object
        pin.name = f"{wheel_name}_Pin_{i+1}"
        pin.parent = wheel_empty
        if materials and 'Roller_Pin' in materials:
            pin.data.materials.append(materials['Roller_Pin'])
        collection.objects.link(pin)
        bpy.context.scene.collection.objects.unlink(pin)
        
    return wheel_empty

def build_motor_assembly(motor_name, collection=None, materials=None):
    """Build a complete DC Gearmotor assembly with encoder and bracket."""
    root = bpy.data.objects.new(motor_name, None)
    root.empty_display_type = 'PLAIN_AXES'
    root.empty_display_size = 0.03
    if collection:
        collection.objects.link(root)
        
    # Gearbox
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=24, radius=0.0185, depth=0.03,
        location=(0, 0.015, 0), rotation=(math.pi/2, 0, 0)
    )
    gearbox = bpy.context.active_object
    gearbox.name = f"{motor_name}_Gearbox"
    gearbox.parent = root
    if materials and 'Gearbox' in materials:
        gearbox.data.materials.append(materials['Gearbox'])
    collection.objects.link(gearbox)
    bpy.context.scene.collection.objects.unlink(gearbox)
    
    # Motor body
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=24, radius=0.018, depth=0.045,
        location=(0, -0.0225, 0), rotation=(math.pi/2, 0, 0)
    )
    motor_can = bpy.context.active_object
    motor_can.name = f"{motor_name}_Can"
    motor_can.parent = root
    if materials and 'Motor_Body' in materials:
        motor_can.data.materials.append(materials['Motor_Body'])
    collection.objects.link(motor_can)
    bpy.context.scene.collection.objects.unlink(motor_can)
    
    # Output shaft
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=16, radius=0.003, depth=0.025,
        location=(0, 0.0425, 0), rotation=(math.pi/2, 0, 0)
    )
    shaft = bpy.context.active_object
    shaft.name = f"{motor_name}_Shaft"
    shaft.parent = root
    if materials and 'Shaft' in materials:
        shaft.data.materials.append(materials['Shaft'])
    collection.objects.link(shaft)
    bpy.context.scene.collection.objects.unlink(shaft)
    
    # L-bracket
    bpy.ops.mesh.primitive_cube_add(
        size=1.0, location=(0, 0.01, -0.021),
        scale=(0.042, 0.032, 0.004)
    )
    bracket_base = bpy.context.active_object
    bracket_base.name = f"{motor_name}_Bracket"
    bracket_base.parent = root
    if materials and 'Motor_Bracket' in materials:
        bracket_base.data.materials.append(materials['Motor_Bracket'])
    collection.objects.link(bracket_base)
    bpy.context.scene.collection.objects.unlink(bracket_base)
    
    # Optical Encoder Module at rear
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=20, radius=0.0175, depth=0.015,
        location=(0, -0.0525, 0), rotation=(math.pi/2, 0, 0)
    )
    encoder = bpy.context.active_object
    encoder.name = f"{motor_name}_Encoder_Housing"
    encoder.parent = root
    if materials and 'PCB_Green' in materials:
        encoder.data.materials.append(materials['PCB_Green'])
    collection.objects.link(encoder)
    bpy.context.scene.collection.objects.unlink(encoder)
    
    # Encoder terminal connector
    bpy.ops.mesh.primitive_cube_add(
        size=1.0, location=(0, -0.062, 0.008),
        scale=(0.012, 0.006, 0.008)
    )
    term = bpy.context.active_object
    term.name = f"{motor_name}_Encoder_Connector"
    term.parent = root
    if materials and 'Chip_IC' in materials:
        term.data.materials.append(materials['Chip_IC'])
    collection.objects.link(term)
    bpy.context.scene.collection.objects.unlink(term)
    
    return root

def build_ultrasonic_sensor(sensor_name, collection=None, materials=None):
    """Build an accurate HC-SR04 ultrasonic sensor with dual cylindrical canisters."""
    root = bpy.data.objects.new(sensor_name, None)
    root.empty_display_type = 'PLAIN_AXES'
    root.empty_display_size = 0.025
    if collection:
        collection.objects.link(root)
        
    # Blue PCB substrate: 45 x 20 x 1.6 mm
    bpy.ops.mesh.primitive_cube_add(
        size=1.0, location=(0, 0, 0),
        scale=(0.045, 0.0016, 0.020)
    )
    pcb = bpy.context.active_object
    pcb.name = f"{sensor_name}_PCB"
    pcb.parent = root
    if materials and 'PCB_Blue' in materials:
        pcb.data.materials.append(materials['PCB_Blue'])
    collection.objects.link(pcb)
    bpy.context.scene.collection.objects.unlink(pcb)
    
    # Dual ultrasonic transducers (Transmitter and Receiver)
    trans_spacing = 0.013
    for i, side_name in [(-1, "Transmitter"), (1, "Receiver")]:
        bpy.ops.mesh.primitive_cylinder_add(
            vertices=20, radius=0.008, depth=0.012,
            location=(i * trans_spacing, 0.0068, 0),
            rotation=(math.pi/2, 0, 0)
        )
        can = bpy.context.active_object
        can.name = f"{sensor_name}_{side_name}_Can"
        can.parent = root
        if materials and 'Ultrasonic_Can' in materials:
            can.data.materials.append(materials['Ultrasonic_Can'])
        collection.objects.link(can)
        bpy.context.scene.collection.objects.unlink(can)
        
        # Front mesh inset
        bpy.ops.mesh.primitive_cylinder_add(
            vertices=16, radius=0.0068, depth=0.001,
            location=(i * trans_spacing, 0.0129, 0),
            rotation=(math.pi/2, 0, 0)
        )
        mesh_dot = bpy.context.active_object
        mesh_dot.name = f"{sensor_name}_{side_name}_Mesh"
        mesh_dot.parent = root
        if materials and 'Ultrasonic_Mesh' in materials:
            mesh_dot.data.materials.append(materials['Ultrasonic_Mesh'])
        collection.objects.link(mesh_dot)
        bpy.context.scene.collection.objects.unlink(mesh_dot)
        
    # Crystal oscillator
    bpy.ops.mesh.primitive_cube_add(
        size=1.0, location=(0, 0.003, -0.004),
        scale=(0.010, 0.003, 0.004)
    )
    osc = bpy.context.active_object
    osc.name = f"{sensor_name}_Oscillator"
    osc.parent = root
    if materials and 'Shaft' in materials:
        osc.data.materials.append(materials['Shaft'])
    collection.objects.link(osc)
    bpy.context.scene.collection.objects.unlink(osc)
    
    # Header
    bpy.ops.mesh.primitive_cube_add(
        size=1.0, location=(0, -0.004, -0.008),
        scale=(0.012, 0.006, 0.003)
    )
    header = bpy.context.active_object
    header.name = f"{sensor_name}_Header"
    header.parent = root
    if materials and 'Chip_IC' in materials:
        header.data.materials.append(materials['Chip_IC'])
    collection.objects.link(header)
    bpy.context.scene.collection.objects.unlink(header)
    
    # Dynamic Sensor Beam Mesh:
    # A slender cylinder pointing horizontally along sensor's forward axis (+Y)
    beam_mesh = bpy.data.meshes.new(f"{sensor_name}_Beam_Mesh_Data")
    beam_obj = bpy.data.objects.new(f"{sensor_name}_Beam_Mesh", beam_mesh)
    collection.objects.link(beam_obj)
    beam_obj.parent = root
    
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=16, radius=0.005, depth=1.0,
        location=(0, 0, 0)
    )
    temp = bpy.context.active_object
    beam_obj.data = temp.data
    bpy.data.objects.remove(temp, do_unlink=True)
    
    beam_obj.rotation_euler = (math.pi/2, 0, 0)
    beam_obj.location = (0, 0.5 + 0.013, 0)
    
    if materials and 'Beam_Safe' in materials:
        beam_obj.data.materials.append(materials['Beam_Safe'])
        
    return root

def build_imu_module(name="IMU_Module", collection=None, materials=None):
    """Build a 6-DOF / 9-DOF IMU sensor module."""
    root = bpy.data.objects.new(name, None)
    root.empty_display_type = 'PLAIN_AXES'
    root.empty_display_size = 0.02
    if collection:
        collection.objects.link(root)
        
    bpy.ops.mesh.primitive_cube_add(
        size=1.0, location=(0, 0, 0.0008),
        scale=(0.021, 0.016, 0.0016)
    )
    pcb = bpy.context.active_object
    pcb.name = f"{name}_PCB"
    pcb.parent = root
    if materials and 'PCB_Blue' in materials:
        pcb.data.materials.append(materials['PCB_Blue'])
    collection.objects.link(pcb)
    bpy.context.scene.collection.objects.unlink(pcb)
    
    bpy.ops.mesh.primitive_cube_add(
        size=1.0, location=(0, 0, 0.0022),
        scale=(0.006, 0.006, 0.0012)
    )
    chip = bpy.context.active_object
    chip.name = f"{name}_Sensor_Chip"
    chip.parent = root
    if materials and 'Chip_IC' in materials:
        chip.data.materials.append(materials['Chip_IC'])
    collection.objects.link(chip)
    bpy.context.scene.collection.objects.unlink(chip)
    
    bpy.ops.mesh.primitive_cube_add(
        size=1.0, location=(0.006, 0.004, 0.002),
        scale=(0.0016, 0.0012, 0.001)
    )
    led = bpy.context.active_object
    led.name = f"{name}_LED"
    led.parent = root
    if materials and 'LED_Green' in materials:
        led.data.materials.append(materials['LED_Green'])
    collection.objects.link(led)
    bpy.context.scene.collection.objects.unlink(led)
    
    return root

def build_electronics_pack(collection=None, materials=None):
    """Build LiPo battery, main robotics controller board, and motor driver."""
    root = bpy.data.objects.new("Electronics_Assembly", None)
    if collection:
        collection.objects.link(root)
        
    bpy.ops.mesh.primitive_cube_add(
        size=1.0, location=(0, -0.04, 0.018),
        scale=(0.075, 0.038, 0.026)
    )
    bat = bpy.context.active_object
    bat.name = "Battery_Pack_3S"
    bat.parent = root
    if materials and 'Battery' in materials:
        bat.data.materials.append(materials['Battery'])
    collection.objects.link(bat)
    bpy.context.scene.collection.objects.unlink(bat)
    
    bpy.ops.mesh.primitive_cube_add(
        size=1.0, location=(0, 0.03, 0.008),
        scale=(0.085, 0.060, 0.002)
    )
    mcu = bpy.context.active_object
    mcu.name = "Controller_Board"
    mcu.parent = root
    if materials and 'PCB_Green' in materials:
        mcu.data.materials.append(materials['PCB_Green'])
    collection.objects.link(mcu)
    bpy.context.scene.collection.objects.unlink(mcu)
    
    bpy.ops.mesh.primitive_cube_add(
        size=1.0, location=(0, 0.03, 0.022),
        scale=(0.032, 0.028, 0.020)
    )
    heatsink = bpy.context.active_object
    heatsink.name = "Motor_Driver_Heatsink"
    heatsink.parent = root
    if materials and 'Gearbox' in materials:
        heatsink.data.materials.append(materials['Gearbox'])
    collection.objects.link(heatsink)
    bpy.context.scene.collection.objects.unlink(heatsink)
    
    return root

def build_robot_model(collections, materials):
    """Assemble complete robot model."""
    robot_root = bpy.data.objects.new("Robot_Base", None)
    robot_root.empty_display_type = 'ARROWS'
    robot_root.empty_display_size = 0.35
    collections['Robot'].objects.link(robot_root)
    
    chassis_radius = 0.28
    deck_gap = 0.065
    wheel_dist = 0.25
    wheel_radius = 0.04
    ground_clearance = 0.015
    
    # Bottom deck
    bot_deck = build_triangular_plate(
        "Chassis_Lower_Plate", radius=chassis_radius, thickness=0.004,
        corner_radius=0.045, collection=collections['Chassis'], material=materials['Chassis_Plate']
    )
    bot_deck.location = (0, 0, ground_clearance + 0.002)
    bot_deck.parent = robot_root
    
    # Top deck
    top_deck = build_triangular_plate(
        "Chassis_Upper_Plate", radius=chassis_radius * 0.95, thickness=0.004,
        corner_radius=0.040, collection=collections['Chassis'], material=materials['Chassis_Plate']
    )
    top_deck.location = (0, 0, ground_clearance + deck_gap + 0.002)
    top_deck.parent = robot_root
    
    # Standoffs
    standoff_angles = [math.radians(a) for a in [30, 90, 150, 210, 270, 330]]
    standoff_r = chassis_radius * 0.65
    for i, sa in enumerate(standoff_angles):
        sx = standoff_r * math.cos(sa)
        sy = standoff_r * math.sin(sa)
        bpy.ops.mesh.primitive_cylinder_add(
            vertices=6, radius=0.0045, depth=deck_gap,
            location=(sx, sy, ground_clearance + deck_gap/2)
        )
        post = bpy.context.active_object
        post.name = f"Chassis_Standoff_{i+1}"
        post.parent = robot_root
        if 'Standoff' in materials:
            post.data.materials.append(materials['Standoff'])
        collections['Chassis'].objects.link(post)
        bpy.context.scene.collection.objects.unlink(post)
        
    elec = build_electronics_pack(collections['Robot'], materials)
    elec.location = (0, 0, ground_clearance + 0.004)
    elec.parent = robot_root
    
    imu = build_imu_module("IMU_Sensor", collections['IMU'], materials)
    imu.location = (0, 0, ground_clearance + deck_gap + 0.006)
    imu.parent = robot_root
    
    # Coordinate indicators on robot top deck
    axis_len = 0.12
    axis_rad = 0.003
    for axis_name, vec, mat_name in [
        ("Robot_Axis_X_Fwd", (axis_len, 0, 0), 'Axis_X_Red'),
        ("Robot_Axis_Y_Left", (0, axis_len, 0), 'Axis_Y_Green'),
        ("Robot_Axis_Z_Up", (0, 0, axis_len), 'Axis_Z_Blue')
    ]:
        bpy.ops.mesh.primitive_cylinder_add(
            vertices=12, radius=axis_rad, depth=axis_len,
            location=(vec[0]/2, vec[1]/2, ground_clearance + deck_gap + 0.02 + vec[2]/2)
        )
        axis_obj = bpy.context.active_object
        axis_obj.name = axis_name
        axis_obj.parent = robot_root
        if vec[0] != 0:
            axis_obj.rotation_euler = (0, math.pi/2, 0)
        elif vec[1] != 0:
            axis_obj.rotation_euler = (-math.pi/2, 0, 0)
        if mat_name in materials:
            axis_obj.data.materials.append(materials[mat_name])
        collections['Robot'].objects.link(axis_obj)
        bpy.context.scene.collection.objects.unlink(axis_obj)

    # 3 Drive Units (Motor + Encoder + Omni Wheel) at 0 deg, 120 deg, 240 deg
    wheel_angles = [0, 120, 240]
    wheels = []
    motors = []
    
    for idx, deg in enumerate(wheel_angles):
        rad = math.radians(deg)
        w_name = f"Omni_Wheel_{idx+1}"
        m_name = f"Drive_Motor_{idx+1}"
        
        wx = wheel_dist * math.cos(rad)
        wy = wheel_dist * math.sin(rad)
        wz = wheel_radius
        
        wheel = build_omni_wheel(w_name, radius=wheel_radius, width=0.024, num_rollers=10, 
                                 collection=collections['Wheels'], materials=materials)
        wheel.location = (wx, wy, wz)
        wheel.rotation_euler = (0, 0, rad)
        wheel.parent = robot_root
        wheels.append(wheel)
        
        motor_dist = 0.165
        mx = motor_dist * math.cos(rad)
        my = motor_dist * math.sin(rad)
        mz = wheel_radius
        
        motor = build_motor_assembly(m_name, collection=collections['Motors'], materials=materials)
        motor.location = (mx, my, mz)
        motor.rotation_euler = (0, 0, rad - math.pi/2)
        motor.parent = robot_root
        motors.append(motor)

    # 3 Ultrasonic Sensors (Front, Left, Right)
    us_configs = [
        ("US_Sensor_Front", (chassis_radius * 0.88, 0, ground_clearance + deck_gap + 0.025), 0),
        ("US_Sensor_Left", (0, chassis_radius * 0.88, ground_clearance + deck_gap + 0.025), math.pi/2),
        ("US_Sensor_Right", (0, -chassis_radius * 0.88, ground_clearance + deck_gap + 0.025), -math.pi/2)
    ]
    
    us_sensors = []
    for s_name, loc, yaw in us_configs:
        us_obj = build_ultrasonic_sensor(s_name, collection=collections['Ultrasonic_Sensors'], materials=materials)
        us_obj.location = loc
        us_obj.rotation_euler = (0, 0, yaw - math.pi/2)
        us_obj.parent = robot_root
        us_sensors.append(us_obj)
        
    return robot_root, wheels, motors, us_sensors

def build_environment(collections, materials):
    """Build arena with floor, walls, reference coordinate axes, and obstacles."""
    arena_size = 10.0
    wall_height = 0.8
    wall_thk = 0.15
    
    # Floor
    bpy.ops.mesh.primitive_plane_add(size=arena_size, location=(0, 0, 0))
    floor = bpy.context.active_object
    floor.name = "Arena_Floor"
    if 'Arena_Floor' in materials:
        floor.data.materials.append(materials['Arena_Floor'])
    collections['Environment'].objects.link(floor)
    bpy.context.scene.collection.objects.unlink(floor)
    
    # Boundary Walls
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
            size=1.0, location=(loc[0], loc[1], wall_height - 0.04),
            scale=(sz[0] * 1.002, sz[1] * 1.002, 0.08)
        )
        stripe = bpy.context.active_object
        stripe.name = f"{w_name}_Hazard_Stripe"
        if 'Hazard_Stripe' in materials:
            stripe.data.materials.append(materials['Hazard_Stripe'])
        collections['Environment'].objects.link(stripe)
        bpy.context.scene.collection.objects.unlink(stripe)

    # Robot Starting Launch Pad
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=32, radius=0.6, depth=0.002,
        location=(0, 0, 0.001)
    )
    pad = bpy.context.active_object
    pad.name = "Start_Launch_Pad"
    if 'Start_Pad' in materials:
        pad.data.materials.append(materials['Start_Pad'])
    collections['Environment'].objects.link(pad)
    bpy.context.scene.collection.objects.unlink(pad)
    
    # Global Coordinate Axes located at Reference Station Corner (-4.0, -4.0, 0.0)
    ref_origin = (-4.0, -4.0, 0.0)
    axis_l = 0.8
    for aname, vec, mname in [
        ("Arena_Axis_X_Forward", (axis_l, 0, 0), 'Axis_X_Red'),
        ("Arena_Axis_Y_Lateral", (0, axis_l, 0), 'Axis_Y_Green'),
        ("Arena_Axis_Z_Vertical", (0, 0, axis_l), 'Axis_Z_Blue')
    ]:
        bpy.ops.mesh.primitive_cylinder_add(
            vertices=16, radius=0.012, depth=axis_l,
            location=(ref_origin[0] + vec[0]/2, ref_origin[1] + vec[1]/2, ref_origin[2] + 0.006 + vec[2]/2)
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

    # Obstacles
    # Obstacle 1: Front Cylindrical Pillar (Target for Forward Detection)
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=32, radius=0.35, depth=1.2,
        location=(2.5, 0.0, 0.6)
    )
    obs1 = bpy.context.active_object
    obs1.name = "Obstacle_Pillar_Front"
    if 'Obstacle_Orange' in materials:
        obs1.data.materials.append(materials['Obstacle_Orange'])
    collections['Environment'].objects.link(obs1)
    bpy.context.scene.collection.objects.unlink(obs1)
    
    # Obstacle 2: Lateral Rectangular Barrier
    bpy.ops.mesh.primitive_cube_add(
        size=1.0, location=(1.2, 2.2, 0.4),
        scale=(0.7, 1.2, 0.8)
    )
    obs2 = bpy.context.active_object
    obs2.name = "Obstacle_Crate_Lateral"
    if 'Obstacle_Teal' in materials:
        obs2.data.materials.append(materials['Obstacle_Teal'])
    collections['Environment'].objects.link(obs2)
    bpy.context.scene.collection.objects.unlink(obs2)
    
    # Obstacle 3: Diagonal Partition Wall
    bpy.ops.mesh.primitive_cube_add(
        size=1.0, location=(-1.5, -1.8, 0.4),
        scale=(1.8, 0.3, 0.8)
    )
    obs3 = bpy.context.active_object
    obs3.name = "Obstacle_Partition_Wall"
    if 'Obstacle_Metal' in materials:
        obs3.data.materials.append(materials['Obstacle_Metal'])
    collections['Environment'].objects.link(obs3)
    bpy.context.scene.collection.objects.unlink(obs3)
    
    # Obstacle 4: Corner Column
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=24, radius=0.45, depth=1.4,
        location=(-2.5, 2.5, 0.7)
    )
    obs4 = bpy.context.active_object
    obs4.name = "Obstacle_Corner_Pillar"
    if 'Obstacle_Orange' in materials:
        obs4.data.materials.append(materials['Obstacle_Orange'])
    collections['Environment'].objects.link(obs4)
    bpy.context.scene.collection.objects.unlink(obs4)

def setup_lighting(collection):
    """Setup studio/arena lighting."""
    sun_data = bpy.data.lights.new(name="Key_Sun_Light", type='SUN')
    sun_data.energy = 4.0
    sun_data.color = (1.0, 0.98, 0.95)
    sun_obj = bpy.data.objects.new("Key_Sun_Light", sun_data)
    sun_obj.location = (5.0, -4.0, 8.0)
    sun_obj.rotation_euler = (math.radians(45), math.radians(25), math.radians(-30))
    collection.objects.link(sun_obj)
    
    area_data = bpy.data.lights.new(name="Overhead_Softbox", type='AREA')
    area_data.energy = 350.0
    area_data.size = 8.0
    area_data.color = (0.92, 0.96, 1.0)
    area_obj = bpy.data.objects.new("Overhead_Softbox", area_data)
    area_obj.location = (0, 0, 5.5)
    collection.objects.link(area_obj)
    
    rim_data = bpy.data.lights.new(name="Rim_Light", type='POINT')
    rim_data.energy = 180.0
    rim_data.color = (0.3, 0.7, 1.0)
    rim_obj = bpy.data.objects.new("Rim_Light", rim_data)
    rim_obj.location = (-3.5, 3.5, 2.5)
    collection.objects.link(rim_obj)

def setup_cameras(collection, target_obj):
    """Setup 4 specialized project cameras."""
    cam_close_data = bpy.data.cameras.new("Camera_Closeup")
    cam_close_data.lens = 55
    cam_close_obj = bpy.data.objects.new("Camera_Closeup", cam_close_data)
    cam_close_obj.location = (0.58, -0.62, 0.42)
    cam_close_obj.rotation_euler = (math.radians(65), 0, math.radians(42))
    collection.objects.link(cam_close_obj)
    
    cam_pres_data = bpy.data.cameras.new("Camera_Presentation")
    cam_pres_data.lens = 38
    cam_pres_obj = bpy.data.objects.new("Camera_Presentation", cam_pres_data)
    cam_pres_obj.location = (2.2, -2.4, 1.7)
    cam_pres_obj.rotation_euler = (math.radians(60), 0, math.radians(42))
    collection.objects.link(cam_pres_obj)
    
    cam_top_data = bpy.data.cameras.new("Camera_TopDown")
    cam_top_data.type = 'ORTHO'
    cam_top_data.ortho_scale = 6.5
    cam_top_obj = bpy.data.objects.new("Camera_TopDown", cam_top_data)
    cam_top_obj.location = (0.6, 0.0, 8.0)
    cam_top_obj.rotation_euler = (0, 0, math.radians(-90))
    collection.objects.link(cam_top_obj)
    
    cam_fol_data = bpy.data.cameras.new("Camera_Follow")
    cam_fol_data.lens = 35
    cam_fol_obj = bpy.data.objects.new("Camera_Follow", cam_fol_data)
    cam_fol_obj.location = (1.5, -1.5, 1.2)
    collection.objects.link(cam_fol_obj)
    
    tt = cam_fol_obj.constraints.new(type='TRACK_TO')
    tt.target = target_obj
    tt.track_axis = 'TRACK_NEGATIVE_Z'
    tt.up_axis = 'UP_Y'
    
    bpy.context.scene.camera = cam_close_obj
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

def main():
    print("=" * 60)
    print("GROUP 7: BUILDING THREE-WHEEL OMNI ROBOT & ARENA SCENE")
    print("=" * 60)
    
    clear_existing_scene()
    
    col_robot = create_collection("Robot")
    col_chassis = create_collection("Chassis", col_robot)
    col_wheels = create_collection("Wheels", col_robot)
    col_motors = create_collection("Motors", col_robot)
    col_encoders = create_collection("Encoders", col_robot)
    col_imu = create_collection("IMU", col_robot)
    col_us = create_collection("Ultrasonic_Sensors", col_robot)
    col_env = create_collection("Environment")
    col_lights = create_collection("Lighting")
    col_cams = create_collection("Cameras")
    
    collections_map = {
        'Robot': col_robot,
        'Chassis': col_chassis,
        'Wheels': col_wheels,
        'Motors': col_motors,
        'Encoders': col_encoders,
        'IMU': col_imu,
        'Ultrasonic_Sensors': col_us,
        'Environment': col_env,
        'Lighting': col_lights,
        'Cameras': col_cams
    }
    
    materials = setup_materials()
    print("[1/5] PBR Materials created successfully.")
    
    robot_root, wheels, motors, us_sensors = build_robot_model(collections_map, materials)
    print(f"[2/5] Robot model built with {len(wheels)} omni wheels, {len(motors)} motors, {len(us_sensors)} ultrasonic sensors.")
    
    build_environment(collections_map, materials)
    print("[3/5] Testing arena and obstacles built successfully.")
    
    setup_lighting(col_lights)
    cam_close, cam_pres, cam_top, cam_fol = setup_cameras(col_cams, robot_root)
    configure_render_settings()
    print("[4/5] Lighting, 4 cameras, and render settings configured.")
    
    output_dir = os.path.dirname(os.path.abspath(__file__))
    blend_path = os.path.join(output_dir, "Group7_OmniRobot.blend")
    bpy.ops.wm.save_as_mainfile(filepath=blend_path)
    print(f"[5/5] Complete project scene saved to: {blend_path}")
    print("=" * 60)
    print("MODEL & SCENE BUILD COMPLETED SUCCESSFULLY!")
    print("=" * 60)

if __name__ == "__main__":
    main()
