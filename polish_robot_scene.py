"""
Group 7 - Three-Wheel Omnidirectional Robot
Master Visual Polish Script: Professional Robotics Laboratory Environment
==========================================================================
Authors: Group 7 Robotics Engineering Team

This script performs a complete visual finishing and presentation upgrade on
Group7_OmniRobot.blend without altering kinematics, physics, object names,
or simulation logic:

1. Transform Environment:
   - Matte industrial dark slate epoxy floor with subtle technical grid markings.
   - Clear arena safety boundary line and corner guards.
   - Laboratory acoustic/modular wall panels with aluminum baseboards.
   - High-grade industrial test obstacles with beveled edges and contrasting finishes.
   - Realistic Start Pad (docking bay decal) and Goal Pad (destination target).
2. Finish Robot Model:
   - Aircraft-grade dark graphite / matte black anodized chassis.
   - Turned aluminum dual-disc omni wheels with distinct peripheral rubber rollers.
   - Metallic brushed motor stators, planetary gearboxes, and anodized brackets.
   - Optical encoder discs with brass/copper finish and 3D labels (M1/ENC1, etc.).
   - Refined IMU module with dark PCB, IC chip, LEDs, and sleek orientation axes.
   - Sleek, semi-transparent ultrasonic sensor rays (precision scientific beams).
   - Interior under-deck illumination to highlight internal mechanics.
3. Materials and Lighting:
   - 4-point studio lab lighting (Key, Fill, Rim, Overhead Softbox, Interior LED).
   - Cycles PBR materials with subtle clearcoat/specular reflections.
   - Cycles Denoising (OpenImageDenoise) enabled for noise-free renders.
4. Professional Presentation:
   - 3D technical header plaque: "GROUP 7 — THREE-WHEEL OMNI ROBOT"
   - Subtitle: "Holonomic Movement in Indoor Environments"
   - Technical component legend plaque.
5. Optimized Presentation Cameras:
   - Camera_Presentation (3/4 laboratory perspective).
   - Camera_Closeup (telephoto macro hero shot of chassis and mechanics).
   - Camera_TopDown (orthographic kinematics and arena layout).
6. Preserves all Master collections, objects, and simulation compatibility.
7. Saves final result to Group7_OmniRobot_Final.blend.
"""

import bpy
import math
import os
from mathutils import Vector, Euler

def setup_node_material(mat, base_color, metallic=0.0, roughness=0.5, specular=0.5, emission_color=None, emission_strength=0.0, alpha=1.0):
    """Utility to configure a clean Principled BSDF node setup."""
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    nodes.clear()
    
    out = nodes.new(type='ShaderNodeOutputMaterial')
    out.location = (400, 0)
    
    bsdf = nodes.new(type='ShaderNodeBsdfPrincipled')
    bsdf.location = (0, 0)
    
    # Base Color
    bsdf.inputs['Base Color'].default_value = base_color
    bsdf.inputs['Metallic'].default_value = metallic
    bsdf.inputs['Roughness'].default_value = roughness
    
    # Specular (handle Blender 4.x input name variations)
    if 'Specular IOR Level' in bsdf.inputs:
        bsdf.inputs['Specular IOR Level'].default_value = specular
    elif 'Specular' in bsdf.inputs:
        bsdf.inputs['Specular'].default_value = specular
        
    # Alpha
    if 'Alpha' in bsdf.inputs:
        bsdf.inputs['Alpha'].default_value = alpha
        if alpha < 1.0:
            mat.blend_method = 'BLEND' if hasattr(mat, 'blend_method') else 'OPAQUE'
            
    # Emission
    if emission_color and emission_strength > 0:
        if 'Emission Color' in bsdf.inputs:
            bsdf.inputs['Emission Color'].default_value = emission_color
            bsdf.inputs['Emission Strength'].default_value = emission_strength
        elif 'Emission' in bsdf.inputs:
            bsdf.inputs['Emission'].default_value = emission_color
            if 'Emission Strength' in bsdf.inputs:
                bsdf.inputs['Emission Strength'].default_value = emission_strength
                
    mat.node_tree.links.new(bsdf.outputs['BSDF'], out.inputs['Surface'])
    return mat

def create_procedural_lab_floor_material(mat_name="Mat_Arena_Floor"):
    """Create a high-end matte industrial robotics laboratory epoxy floor with subtle technical grid."""
    mat = bpy.data.materials.get(mat_name)
    if not mat:
        mat = bpy.data.materials.new(name=mat_name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    nodes.clear()
    links = mat.node_tree.links
    
    out = nodes.new(type='ShaderNodeOutputMaterial')
    out.location = (800, 0)
    
    bsdf = nodes.new(type='ShaderNodeBsdfPrincipled')
    bsdf.location = (500, 0)
    bsdf.inputs['Roughness'].default_value = 0.28
    bsdf.inputs['Metallic'].default_value = 0.15
    if 'Specular IOR Level' in bsdf.inputs:
        bsdf.inputs['Specular IOR Level'].default_value = 0.55
    elif 'Specular' in bsdf.inputs:
        bsdf.inputs['Specular'].default_value = 0.55
        
    # Texture coordinate
    tex_coord = nodes.new(type='ShaderNodeTexCoord')
    tex_coord.location = (-600, 0)
    
    # Mapping
    mapping = nodes.new(type='ShaderNodeMapping')
    mapping.location = (-400, 0)
    links.new(tex_coord.outputs['Object'], mapping.inputs['Vector'])
    
    # Separate XYZ for precise technical grid
    sep_xyz = nodes.new(type='ShaderNodeSeparateXYZ')
    sep_xyz.location = (-200, 0)
    links.new(mapping.outputs['Vector'], sep_xyz.inputs['Vector'])
    
    # Fraction / math for 1m primary grid lines
    # X grid line
    math_x1 = nodes.new(type='ShaderNodeMath')
    math_x1.operation = 'PINGPONG'
    math_x1.inputs[1].default_value = 1.0  # 1m grid
    math_x1.location = (0, 150)
    links.new(sep_xyz.outputs['X'], math_x1.inputs[0])
    
    math_x2 = nodes.new(type='ShaderNodeMath')
    math_x2.operation = 'LESS_THAN'
    math_x2.inputs[1].default_value = 0.018 # 18mm grid line width
    math_x2.location = (160, 150)
    links.new(math_x1.outputs['Value'], math_x2.inputs[0])
    
    # Y grid line
    math_y1 = nodes.new(type='ShaderNodeMath')
    math_y1.operation = 'PINGPONG'
    math_y1.inputs[1].default_value = 1.0  # 1m grid
    math_y1.location = (0, -100)
    links.new(sep_xyz.outputs['Y'], math_y1.inputs[0])
    
    math_y2 = nodes.new(type='ShaderNodeMath')
    math_y2.operation = 'LESS_THAN'
    math_y2.inputs[1].default_value = 0.018
    math_y2.location = (160, -100)
    links.new(math_y1.outputs['Value'], math_y2.inputs[0])
    
    # Combine X and Y grid lines (MAX)
    math_or = nodes.new(type='ShaderNodeMath')
    math_or.operation = 'MAXIMUM'
    math_or.location = (320, 20)
    links.new(math_x2.outputs['Value'], math_or.inputs[0])
    links.new(math_y2.outputs['Value'], math_or.inputs[1])
    
    # Color Ramp for subtle grid contrast
    mix_rgb = nodes.new(type='ShaderNodeMix')
    mix_rgb.data_type = 'RGBA'
    mix_rgb.location = (340, -150)
    # Factor from grid lines
    links.new(math_or.outputs['Value'], mix_rgb.inputs['Factor'])
    # Floor base color: Dark slate epoxy
    mix_rgb.inputs[6].default_value = (0.11, 0.12, 0.15, 1.0) # Slate Grey Floor
    # Grid lines color: Subtle technical cool blue-grey
    mix_rgb.inputs[7].default_value = (0.24, 0.32, 0.44, 1.0) # Clean technical line
    
    links.new(mix_rgb.outputs['Result'], bsdf.inputs['Base Color'])
    links.new(bsdf.outputs['BSDF'], out.inputs['Surface'])
    
    mat.diffuse_color = (0.11, 0.12, 0.15, 1.0)
    return mat

def create_hazard_stripe_material(mat_name="Mat_Hazard_Stripe"):
    """Create authentic 45-degree yellow and black industrial hazard caution stripes."""
    mat = bpy.data.materials.get(mat_name)
    if not mat:
        mat = bpy.data.materials.new(name=mat_name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    nodes.clear()
    links = mat.node_tree.links
    
    out = nodes.new(type='ShaderNodeOutputMaterial')
    out.location = (600, 0)
    
    bsdf = nodes.new(type='ShaderNodeBsdfPrincipled')
    bsdf.location = (300, 0)
    bsdf.inputs['Roughness'].default_value = 0.35
    bsdf.inputs['Metallic'].default_value = 0.05
    
    tex_coord = nodes.new(type='ShaderNodeTexCoord')
    tex_coord.location = (-500, 0)
    
    mapping = nodes.new(type='ShaderNodeMapping')
    mapping.location = (-300, 0)
    mapping.inputs['Rotation'].default_value = (0, 0, math.radians(45))
    mapping.inputs['Scale'].default_value = (8.0, 8.0, 8.0)
    links.new(tex_coord.outputs['Object'], mapping.inputs['Vector'])
    
    wave = nodes.new(type='ShaderNodeTexWave')
    wave.location = (-100, 0)
    wave.wave_type = 'BANDS'
    wave.inputs['Scale'].default_value = 4.0
    wave.inputs['Distortion'].default_value = 0.0
    links.new(mapping.outputs['Vector'], wave.inputs['Vector'])
    
    ramp = nodes.new(type='ShaderNodeValToRGB')
    ramp.location = (100, 0)
    ramp.color_ramp.interpolation = 'CONSTANT'
    ramp.color_ramp.elements[0].position = 0.5
    ramp.color_ramp.elements[0].color = (0.04, 0.04, 0.04, 1.0) # Matte Black
    ramp.color_ramp.elements[1].position = 0.501
    ramp.color_ramp.elements[1].color = (0.95, 0.78, 0.04, 1.0) # Safety Yellow
    links.new(wave.outputs['Color'], ramp.inputs['Fac'])
    
    links.new(ramp.outputs['Color'], bsdf.inputs['Base Color'])
    links.new(bsdf.outputs['BSDF'], out.inputs['Surface'])
    
    mat.diffuse_color = (0.95, 0.78, 0.04, 1.0)
    return mat

def create_technical_pad_material(mat_name, base_ring_color, accent_color, is_goal=False):
    """Create high-tech launch / destination circular target decal with concentric rings."""
    mat = bpy.data.materials.get(mat_name)
    if not mat:
        mat = bpy.data.materials.new(name=mat_name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    nodes.clear()
    links = mat.node_tree.links
    
    out = nodes.new(type='ShaderNodeOutputMaterial')
    out.location = (600, 0)
    
    bsdf = nodes.new(type='ShaderNodeBsdfPrincipled')
    bsdf.location = (300, 0)
    bsdf.inputs['Roughness'].default_value = 0.30
    bsdf.inputs['Metallic'].default_value = 0.20
    
    tex_coord = nodes.new(type='ShaderNodeTexCoord')
    tex_coord.location = (-400, 0)
    
    grad = nodes.new(type='ShaderNodeTexGradient')
    grad.gradient_type = 'SPHERICAL'
    grad.location = (-200, 0)
    links.new(tex_coord.outputs['Object'], grad.inputs['Vector'])
    
    ramp = nodes.new(type='ShaderNodeValToRGB')
    ramp.location = (0, 0)
    ramp.color_ramp.interpolation = 'CONSTANT'
    
    # Create concentric target rings
    cr = ramp.color_ramp
    while len(cr.elements) > 1:
        cr.elements.remove(cr.elements[-1])
    cr.elements[0].position = 0.0
    cr.elements[0].color = (0.08, 0.10, 0.13, 1.0)
    
    e1 = cr.elements.new(0.25)
    e1.color = base_ring_color
    e2 = cr.elements.new(0.48)
    e2.color = (0.06, 0.08, 0.10, 1.0)
    e3 = cr.elements.new(0.72)
    e3.color = accent_color
    
    links.new(grad.outputs['Fac'], ramp.inputs['Fac'])
    links.new(ramp.outputs['Color'], bsdf.inputs['Base Color'])
    
    if is_goal:
        if 'Emission Color' in bsdf.inputs:
            bsdf.inputs['Emission Color'].default_value = accent_color
            bsdf.inputs['Emission Strength'].default_value = 1.2
        elif 'Emission' in bsdf.inputs:
            bsdf.inputs['Emission'].default_value = accent_color
            if 'Emission Strength' in bsdf.inputs:
                bsdf.inputs['Emission Strength'].default_value = 1.2
                
    links.new(bsdf.outputs['BSDF'], out.inputs['Surface'])
    mat.diffuse_color = accent_color
    return mat

def upgrade_all_materials():
    """Upgrade every material in the scene to high-end realistic robotics PBR shaders."""
    print("[1/6] Upgrading PBR Materials to Laboratory Standard...")
    
    # 1. Industrial Floor
    create_procedural_lab_floor_material('Mat_Arena_Floor')
    
    # 2. Safety Hazard Stripes
    create_hazard_stripe_material('Mat_Hazard_Stripe')
    
    # 3. Start Pad (Docking Station Green / Teal) & Goal Pad (Cyan / Neon Blue Target)
    create_technical_pad_material('Mat_Start_Pad', (0.12, 0.38, 0.22, 1.0), (0.18, 0.72, 0.45, 1.0), is_goal=False)
    create_technical_pad_material('Mat_Goal_Pad', (0.08, 0.32, 0.58, 1.0), (0.00, 0.75, 1.00, 1.0), is_goal=True)
    
    # 4. Robot Chassis Plate (Matte graphite anodized carbon/aluminum)
    mat_chassis = bpy.data.materials.get('Mat_Chassis_Plate')
    if mat_chassis:
        setup_node_material(mat_chassis, (0.05, 0.052, 0.06, 1.0), metallic=0.75, roughness=0.22, specular=0.6)
        
    # 5. Standoffs (Turned Brass / Nickel Stainless Steel)
    mat_standoff = bpy.data.materials.get('Mat_Standoff')
    if mat_standoff:
        setup_node_material(mat_standoff, (0.88, 0.88, 0.92, 1.0), metallic=0.98, roughness=0.10, specular=0.9)
        
    # 6. Fasteners (High-torque hex socket screws)
    mat_fastener = bpy.data.materials.get('Mat_Fastener')
    if mat_fastener:
        setup_node_material(mat_fastener, (0.92, 0.93, 0.96, 1.0), metallic=1.0, roughness=0.08, specular=0.95)
        
    # 7. Motor Assemblies
    mat_motor_body = bpy.data.materials.get('Mat_Motor_Body')
    if mat_motor_body:
        setup_node_material(mat_motor_body, (0.18, 0.19, 0.22, 1.0), metallic=0.92, roughness=0.20, specular=0.7)
        
    mat_gearbox = bpy.data.materials.get('Mat_Gearbox')
    if mat_gearbox:
        setup_node_material(mat_gearbox, (0.85, 0.87, 0.90, 1.0), metallic=0.96, roughness=0.12, specular=0.85)
        
    mat_bracket = bpy.data.materials.get('Mat_Motor_Bracket')
    if mat_bracket:
        setup_node_material(mat_bracket, (0.08, 0.35, 0.65, 1.0), metallic=0.88, roughness=0.22, specular=0.7)
        
    mat_shaft = bpy.data.materials.get('Mat_Shaft')
    if mat_shaft:
        setup_node_material(mat_shaft, (0.92, 0.93, 0.95, 1.0), metallic=1.0, roughness=0.06, specular=0.95)
        
    # 8. Omni Wheel Hubs & Rollers
    mat_wheel_hub = bpy.data.materials.get('Mat_Wheel_Hub')
    if mat_wheel_hub:
        setup_node_material(mat_wheel_hub, (0.75, 0.12, 0.15, 1.0), metallic=0.92, roughness=0.18, specular=0.8)
        
    mat_roller = bpy.data.materials.get('Mat_Roller_Rubber')
    if mat_roller:
        setup_node_material(mat_roller, (0.045, 0.045, 0.05, 1.0), metallic=0.0, roughness=0.78, specular=0.3)
        
    mat_pin = bpy.data.materials.get('Mat_Roller_Pin')
    if mat_pin:
        setup_node_material(mat_pin, (0.92, 0.92, 0.94, 1.0), metallic=1.0, roughness=0.06, specular=0.9)
        
    # 9. Optical Encoders
    mat_enc_disc = bpy.data.materials.get('Mat_Encoder_Disc')
    if mat_enc_disc:
        setup_node_material(mat_enc_disc, (0.92, 0.72, 0.15, 1.0), metallic=0.94, roughness=0.14, specular=0.85)
        
    mat_enc_hsg = bpy.data.materials.get('Mat_Encoder_Housing')
    if mat_enc_hsg:
        setup_node_material(mat_enc_hsg, (0.08, 0.09, 0.11, 1.0), metallic=0.20, roughness=0.30, specular=0.5)
        
    # 10. IMU & PCBs
    mat_pcb_blue = bpy.data.materials.get('Mat_PCB_Blue')
    if mat_pcb_blue:
        setup_node_material(mat_pcb_blue, (0.05, 0.22, 0.65, 1.0), metallic=0.15, roughness=0.30, specular=0.5)
        
    mat_pcb_green = bpy.data.materials.get('Mat_PCB_Green')
    if mat_pcb_green:
        setup_node_material(mat_pcb_green, (0.03, 0.35, 0.12, 1.0), metallic=0.15, roughness=0.30, specular=0.5)
        
    mat_chip = bpy.data.materials.get('Mat_Chip_IC')
    if mat_chip:
        setup_node_material(mat_chip, (0.03, 0.03, 0.035, 1.0), metallic=0.10, roughness=0.35, specular=0.6)
        
    mat_battery = bpy.data.materials.get('Mat_Battery')
    if mat_battery:
        setup_node_material(mat_battery, (0.08, 0.42, 0.82, 1.0), metallic=0.25, roughness=0.35, specular=0.55)
        
    # 11. Ultrasonic Sensors & Sleek Rays
    mat_us_can = bpy.data.materials.get('Mat_Ultrasonic_Can')
    if mat_us_can:
        setup_node_material(mat_us_can, (0.88, 0.90, 0.93, 1.0), metallic=0.96, roughness=0.12, specular=0.85)
        
    mat_us_mesh = bpy.data.materials.get('Mat_Ultrasonic_Mesh')
    if mat_us_mesh:
        setup_node_material(mat_us_mesh, (0.06, 0.06, 0.07, 1.0), metallic=0.40, roughness=0.75, specular=0.3)
        
    # Sleek scientific semi-transparent detection rays (subtle energetic pulse instead of giant laser sabers)
    mat_beam_safe = bpy.data.materials.get('Mat_Beam_Safe')
    if mat_beam_safe:
        setup_node_material(
            mat_beam_safe, (0.0, 0.90, 0.50, 0.35), metallic=0.0, roughness=0.15,
            emission_color=(0.0, 0.90, 0.50, 1.0), emission_strength=1.8, alpha=0.35
        )
        
    mat_beam_alert = bpy.data.materials.get('Mat_Beam_Alert')
    if mat_beam_alert:
        setup_node_material(
            mat_beam_alert, (1.0, 0.15, 0.15, 0.60), metallic=0.0, roughness=0.15,
            emission_color=(1.0, 0.15, 0.15, 1.0), emission_strength=3.2, alpha=0.60
        )
        
    # 12. Walls & Obstacles (Restrained, high-tech industrial finishes)
    mat_wall = bpy.data.materials.get('Mat_Wall_Concrete')
    if mat_wall:
        setup_node_material(mat_wall, (0.24, 0.26, 0.30, 1.0), metallic=0.15, roughness=0.60, specular=0.45)
        
    mat_obs_pillar = bpy.data.materials.get('Mat_Obstacle_Orange')
    if mat_obs_pillar:
        # Front Cylinder Pillar: Dark carbon composite with safety yellow technical ring
        setup_node_material(mat_obs_pillar, (0.16, 0.18, 0.21, 1.0), metallic=0.30, roughness=0.28, specular=0.6)
        
    mat_obs_crate = bpy.data.materials.get('Mat_Obstacle_Teal')
    if mat_obs_crate:
        # Lateral Equipment Crate: Deep industrial slate navy with metallic corner trims
        setup_node_material(mat_obs_crate, (0.12, 0.18, 0.26, 1.0), metallic=0.40, roughness=0.32, specular=0.55)
        
    mat_obs_metal = bpy.data.materials.get('Mat_Obstacle_Metal')
    if mat_obs_metal:
        # Partition Wall: Brushed structural aluminum frame & composite panel
        setup_node_material(mat_obs_metal, (0.55, 0.58, 0.62, 1.0), metallic=0.90, roughness=0.20, specular=0.85)

def refine_sensor_beams():
    """Make the ultrasonic sensor beam meshes sleeker (5mm precision ray instead of 28mm thick cylinder)."""
    print("[2/6] Refining Ultrasonic Sensor Beams...")
    for s_name in ["US_Sensor_Front_Beam_Mesh", "US_Sensor_Left_Beam_Mesh", "US_Sensor_Right_Beam_Mesh"]:
        beam = bpy.data.objects.get(s_name)
        if beam and beam.type == 'MESH':
            # Scale X and Y vertices down to 0.004m radius while keeping Z length intact
            me = beam.data
            for v in me.vertices:
                v.co.x *= 0.28  # Reduce beam thickness to sleek 4mm ray
                v.co.y *= 0.28
            me.update()

def add_laboratory_architecture(col_env):
    """Add professional laboratory trim, corner guards, and boundary safety markings."""
    print("[3/6] Adding Laboratory Architecture (Trim, Guards, Safety Border)...")
    
    # 1. Arena Safety Border (Fine yellow boundary line 0.3m inside walls)
    if not bpy.data.objects.get("Arena_Safety_Border"):
        arena_size = 10.0
        border_size = 9.2
        border_width = 0.04
        half_b = border_size / 2.0
        
        # Create 4 safety border line segments on the floor
        border_lines = [
            ("Border_N", (0, half_b, 0.0012), (border_size, border_width, 0.001)),
            ("Border_S", (0, -half_b, 0.0012), (border_size, border_width, 0.001)),
            ("Border_E", (half_b, 0, 0.0012), (border_width, border_size, 0.001)),
            ("Border_W", (-half_b, 0, 0.0012), (border_width, border_size, 0.001)),
        ]
        mat_hazard = bpy.data.materials.get('Mat_Hazard_Stripe')
        for b_name, b_loc, b_scale in border_lines:
            bpy.ops.mesh.primitive_cube_add(size=1.0, location=b_loc, scale=b_scale)
            b_obj = bpy.context.active_object
            b_obj.name = f"Arena_Safety_{b_name}"
            if mat_hazard:
                b_obj.data.materials.append(mat_hazard)
            col_env.objects.link(b_obj)
            bpy.context.scene.collection.objects.unlink(b_obj)
            
    # 2. Heavy-Duty Laboratory Corner Guards (At the 4 arena wall corners)
    corners = [(-4.85, -4.85), (4.85, -4.85), (-4.85, 4.85), (4.85, 4.85)]
    mat_metal = bpy.data.materials.get('Mat_Obstacle_Metal')
    mat_hazard = bpy.data.materials.get('Mat_Hazard_Stripe')
    
    for idx, (cx, cy) in enumerate(corners):
        guard_name = f"Lab_Corner_Guard_{idx+1}"
        if not bpy.data.objects.get(guard_name):
            bpy.ops.mesh.primitive_cylinder_add(
                vertices=16, radius=0.08, depth=0.85,
                location=(cx, cy, 0.425)
            )
            c_obj = bpy.context.active_object
            c_obj.name = guard_name
            if mat_hazard:
                c_obj.data.materials.append(mat_hazard)
            col_env.objects.link(c_obj)
            bpy.context.scene.collection.objects.unlink(c_obj)
            
            # Corner post base flange
            bpy.ops.mesh.primitive_cylinder_add(
                vertices=16, radius=0.12, depth=0.04,
                location=(cx, cy, 0.02)
            )
            flange = bpy.context.active_object
            flange.name = f"{guard_name}_Flange"
            if mat_metal:
                flange.data.materials.append(mat_metal)
            col_env.objects.link(flange)
            bpy.context.scene.collection.objects.unlink(flange)

    # 3. Obstacle Bevels & Detail Rings
    pillar = bpy.data.objects.get("Obstacle_Pillar_Front")
    if pillar and not pillar.modifiers.get("Bevel"):
        bev = pillar.modifiers.new(name="Bevel", type='BEVEL')
        bev.width = 0.02
        bev.segments = 3
        
        # Add safety warning stripe collar around the front pillar
        bpy.ops.mesh.primitive_cylinder_add(
            vertices=32, radius=0.385, depth=0.18,
            location=(2.50, 0.00, 0.65)
        )
        band = bpy.context.active_object
        band.name = "Obstacle_Pillar_Front_Band"
        if mat_hazard:
            band.data.materials.append(mat_hazard)
        col_obs = bpy.data.collections.get('Obstacles')
        if col_obs:
            col_obs.objects.link(band)
            bpy.context.scene.collection.objects.unlink(band)

    crate = bpy.data.objects.get("Obstacle_Crate_Lateral")
    if crate and not crate.modifiers.get("Bevel"):
        bev = crate.modifiers.new(name="Bevel", type='BEVEL')
        bev.width = 0.02
        bev.segments = 2
        
    part_wall = bpy.data.objects.get("Obstacle_Partition_Wall")
    if part_wall and not part_wall.modifiers.get("Bevel"):
        bev = part_wall.modifiers.new(name="Bevel", type='BEVEL')
        bev.width = 0.015
        bev.segments = 2

def add_robot_detail_labels(col_chassis):
    """Add small legible 3D identification badges for Motors, Encoders, and Chassis."""
    print("[4/6] Adding Robot Mechanical Identification Labels...")
    mat_white = bpy.data.materials.get('Mat_Text_White')
    robot_base = bpy.data.objects.get("Robot_Base")
    
    # Motor & Encoder labels
    # Wheel 1 (Front, 0 deg): X=0.22, Y=0.0
    # Wheel 2 (Rear-Left, 120 deg): X=-0.11, Y=0.19
    # Wheel 3 (Rear-Right, 240 deg): X=-0.11, Y=-0.19
    labels_data = [
        ("Label_Motor_1", "M1 / ENC1", (0.16, 0.05, 0.065), 0),
        ("Label_Motor_2", "M2 / ENC2", (-0.12, 0.16, 0.065), math.radians(120)),
        ("Label_Motor_3", "M3 / ENC3", (-0.05, -0.16, 0.065), math.radians(240)),
    ]
    for lbl_name, lbl_text, lbl_pos, lbl_rot in labels_data:
        if not bpy.data.objects.get(lbl_name):
            bpy.ops.object.text_add(location=lbl_pos)
            txt = bpy.context.active_object
            txt.name = lbl_name
            txt.data.body = lbl_text
            txt.data.size = 0.012
            txt.data.extrude = 0.0015
            txt.rotation_euler = (0, 0, lbl_rot)
            if robot_base:
                txt.parent = robot_base
            if mat_white:
                txt.data.materials.append(mat_white)
            col_chassis.objects.link(txt)
            bpy.context.scene.collection.objects.unlink(txt)
            
    # Add subtle interior accent light inside chassis so mechanics are clearly illuminated
    if not bpy.data.objects.get("Chassis_Internal_LED"):
        led_data = bpy.data.lights.new(name="Chassis_Internal_LED", type='POINT')
        led_data.energy = 14.0 # Soft gentle glow
        led_data.color = (0.95, 0.98, 1.0)
        led_data.shadow_soft_size = 0.05
        led_obj = bpy.data.objects.new("Chassis_Internal_LED", led_data)
        led_obj.location = (0.0, 0.0, 0.055)
        if robot_base:
            led_obj.parent = robot_base
        col_chassis.objects.link(led_obj)

def add_presentation_graphics(col_env):
    """Add sleek, non-intrusive 3D title header and technical legend to the testing arena."""
    print("[5/6] Adding Presentation Title Header and Technical Legend...")
    mat_white = bpy.data.materials.get('Mat_Text_White')
    mat_metal = bpy.data.materials.get('Mat_Obstacle_Metal')
    mat_gearbox = bpy.data.materials.get('Mat_Gearbox')
    
    # 1. Arena Title Plaque (Mounted neatly on North Wall)
    if not bpy.data.objects.get("Arena_Title_Header"):
        # Backing mounting plate
        bpy.ops.mesh.primitive_cube_add(
            size=1.0, location=(0, 4.90, 0.72),
            scale=(3.8, 0.02, 0.28)
        )
        plaque = bpy.context.active_object
        plaque.name = "Arena_Title_Header_Plaque"
        if mat_metal:
            plaque.data.materials.append(mat_metal)
        col_env.objects.link(plaque)
        bpy.context.scene.collection.objects.unlink(plaque)
        
        # Main Title Text
        bpy.ops.object.text_add(location=(-1.70, 4.88, 0.74))
        title = bpy.context.active_object
        title.name = "Arena_Title_Header"
        title.data.body = "GROUP 7 — THREE-WHEEL OMNI ROBOT"
        title.data.size = 0.125
        title.data.extrude = 0.008
        title.rotation_euler = (math.radians(90), 0, 0)
        if mat_white:
            title.data.materials.append(mat_white)
        col_env.objects.link(title)
        bpy.context.scene.collection.objects.unlink(title)
        
        # Subtitle Text
        bpy.ops.object.text_add(location=(-1.68, 4.88, 0.64))
        subtitle = bpy.context.active_object
        subtitle.name = "Arena_Subtitle_Header"
        subtitle.data.body = "Holonomic Kinematics & Dual-Layer Collision Architecture"
        subtitle.data.size = 0.065
        subtitle.data.extrude = 0.004
        subtitle.rotation_euler = (math.radians(90), 0, 0)
        if mat_white:
            subtitle.data.materials.append(mat_white)
        col_env.objects.link(subtitle)
        bpy.context.scene.collection.objects.unlink(subtitle)
        
    # 2. Technical Component Legend (Mounted cleanly on East Wall)
    if not bpy.data.objects.get("Arena_Legend_Plaque"):
        bpy.ops.mesh.primitive_cube_add(
            size=1.0, location=(4.90, -1.8, 0.65),
            scale=(0.02, 2.2, 0.45)
        )
        leg_plaque = bpy.context.active_object
        leg_plaque.name = "Arena_Legend_Plaque"
        if mat_metal:
            leg_plaque.data.materials.append(mat_metal)
        col_env.objects.link(leg_plaque)
        bpy.context.scene.collection.objects.unlink(leg_plaque)
        
        legend_lines = [
            ("SYSTEM TELEMETRY & HARDWARE", 0.18, 0.065),
            ("1. Optical Encoders (360 PPR Quadrature)", 0.08, 0.045),
            ("2. 6-DOF IMU Sensor (Acc/Gyro + Coriolis)", -0.01, 0.045),
            ("3. HC-SR04 Rangefinders (Raycast Cone)", -0.10, 0.045),
            ("4. Dual-Disc Omni Wheels (10 Barrel Rollers)", -0.19, 0.045),
        ]
        for l_idx, (l_text, l_z_offset, l_sz) in enumerate(legend_lines):
            bpy.ops.object.text_add(location=(4.88, -0.85, 0.65 + l_z_offset))
            leg_txt = bpy.context.active_object
            leg_txt.name = f"Arena_Legend_Text_{l_idx}"
            leg_txt.data.body = l_text
            leg_txt.data.size = l_sz
            leg_txt.data.extrude = 0.003
            leg_txt.rotation_euler = (math.radians(90), 0, math.radians(-90))
            if mat_white:
                leg_txt.data.materials.append(mat_white)
            col_env.objects.link(leg_txt)
            bpy.context.scene.collection.objects.unlink(leg_txt)

def optimize_lighting_and_cameras():
    """Configure professional studio 4-point laboratory lighting and framing cameras."""
    print("[6/6] Optimizing Lighting, Cycles Settings, and Cameras...")
    
    # Lighting upgrades
    sun = bpy.data.objects.get("Key_Sun_Light")
    if sun and sun.data:
        sun.data.energy = 5.5
        sun.data.color = (1.0, 0.98, 0.95)
        sun.data.angle = math.radians(2.5) # Soft shadow edge
        
    softbox = bpy.data.objects.get("Overhead_Softbox")
    if softbox and softbox.data:
        softbox.data.energy = 550.0
        softbox.data.size = 9.5
        softbox.data.color = (0.92, 0.96, 1.0) # Crisp daylight lab fill
        
    rim = bpy.data.objects.get("Rim_Light")
    if rim and rim.data:
        rim.data.energy = 380.0
        rim.data.color = (0.90, 0.95, 1.0)
        rim.location = (-4.2, 4.0, 3.2)
        
    fill = bpy.data.objects.get("Fill_Light")
    if fill and fill.data:
        fill.data.energy = 240.0
        fill.data.color = (1.0, 0.94, 0.88)
        fill.location = (-3.2, -3.8, 3.8)

    # Cycles Rendering Configuration (High-end Denoising + Crisp Shading)
    scene = bpy.context.scene
    scene.render.engine = 'CYCLES'
    scene.cycles.device = 'CPU'
    scene.cycles.samples = 64
    scene.cycles.use_denoising = True
    scene.cycles.denoiser = 'OPENIMAGEDENOISE'
    
    # Color Management (Filmic / AgX with Medium High Contrast)
    if hasattr(scene.view_settings, 'view_transform'):
        scene.view_settings.view_transform = 'AgX' if 'AgX' in [v.identifier for v in scene.view_settings.bl_rna.properties['view_transform'].enum_items] else 'Filmic'
    if hasattr(scene.view_settings, 'look'):
        scene.view_settings.look = 'Medium High Contrast'
        
    # Cameras fine-tuning for presentation
    # 1. Target Empties for perfect framing
    col_cam = bpy.data.collections.get("Cameras")
    
    # Presentation Target Empty at center of action
    t_pres = bpy.data.objects.get("Target_Focus_Presentation")
    if not t_pres:
        t_pres = bpy.data.objects.new("Target_Focus_Presentation", None)
        t_pres.empty_display_type = 'PLAIN_AXES'
        t_pres.empty_display_size = 0.2
        t_pres.location = (1.4, 0.8, 0.35)
        if col_cam:
            col_cam.objects.link(t_pres)
            
    # Closeup Target Empty on robot
    t_close = bpy.data.objects.get("Target_Focus_Closeup")
    if not t_close:
        t_close = bpy.data.objects.new("Target_Focus_Closeup", None)
        t_close.empty_display_type = 'PLAIN_AXES'
        t_close.empty_display_size = 0.1
        t_close.location = (0.05, 0.0, 0.08)
        if col_cam:
            col_cam.objects.link(t_close)

    # 1. Camera_Presentation: Three-quarter lab hero perspective framing complete robot & arena
    cam_pres = bpy.data.objects.get("Camera_Presentation")
    if cam_pres and cam_pres.data:
        cam_pres.data.lens = 34.0 # Wide cinematic lens captures arena, walls, and obstacles
        cam_pres.location = (4.40, -5.20, 3.40)
        
        # Ensure Track To constraint to presentation target
        tt = cam_pres.constraints.get("Track_Presentation")
        if not tt:
            tt = cam_pres.constraints.new(type='TRACK_TO')
            tt.name = "Track_Presentation"
        tt.target = t_pres
        tt.track_axis = 'TRACK_NEGATIVE_Z'
        tt.up_axis = 'UP_Y'
        
    # 2. Camera_Closeup: Telephoto macro perspective showing all wheels, motors, IMU, sensors
    cam_close = bpy.data.objects.get("Camera_Closeup")
    if cam_close and cam_close.data:
        cam_close.data.lens = 65.0 # Macro perspective
        cam_close.location = (0.70, -0.68, 0.34)
        
        tt_c = cam_close.constraints.get("Track_Closeup")
        if not tt_c:
            tt_c = cam_close.constraints.new(type='TRACK_TO')
            tt_c.name = "Track_Closeup"
        tt_c.target = t_close
        tt_c.track_axis = 'TRACK_NEGATIVE_Z'
        tt_c.up_axis = 'UP_Y'
        
    # 3. Camera_TopDown: Orthographic arena and trajectory map showing full 10m x 10m arena
    cam_top = bpy.data.objects.get("Camera_TopDown")
    if cam_top and cam_top.data:
        cam_top.data.type = 'ORTHO'
        cam_top.data.ortho_scale = 11.4 # Full arena with clean border margin
        cam_top.location = (0.0, 0.0, 12.0)
        cam_top.rotation_euler = (0, 0, math.radians(-90))

def main():
    print("=" * 70)
    print("GROUP 7: EXECUTING MASTER SCENE POLISH & LABORATORY TRANSFORMATION")
    print("=" * 70)
    
    col_env = bpy.data.collections.get("Environment")
    col_chassis = bpy.data.collections.get("Robot_Chassis")
    
    upgrade_all_materials()
    refine_sensor_beams()
    
    if col_env:
        add_laboratory_architecture(col_env)
        add_presentation_graphics(col_env)
        
    if col_chassis:
        add_robot_detail_labels(col_chassis)
        
    optimize_lighting_and_cameras()
    
    # Save the polished project as Group7_OmniRobot_Final.blend
    output_dir = os.path.dirname(os.path.abspath(__file__))
    final_blend_path = os.path.join(output_dir, "Group7_OmniRobot_Final.blend")
    bpy.ops.wm.save_as_mainfile(filepath=final_blend_path)
    
    print("\n" + "=" * 70)
    print(f"POLISHED SCENE SUCCESSFULLY SAVED TO: {final_blend_path}")
    print("=" * 70)

if __name__ == "__main__":
    main()
