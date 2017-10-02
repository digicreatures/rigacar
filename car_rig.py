# ##### BEGIN GPL LICENSE BLOCK #####
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 3
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software Foundation,
#  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
# ##### END GPL LICENSE BLOCK #####

# <pep8 compliant>

import bpy
import math
import bpy_extras
import mathutils

DEF_BONE_LAYER = 30
MCH_BONE_LAYER = 31


def deselect_edit_bones(ob):
    for b in ob.data.edit_bones:
        b.select = False
        b.select_head = False
        b.select_tail = False


def create_constraint_influence_driver(ob, cns, driver_data_path, base_influence=1.0):
    fcurve = cns.driver_add('influence')
    drv = fcurve.driver
    drv.type = 'AVERAGE'
    var = drv.variables.new()
    var.name = 'influence'
    var.type = 'SINGLE_PROP'

    targ = var.targets[0]
    targ.id_type = 'OBJECT'
    targ.id = ob
    targ.data_path = driver_data_path

    if base_influence != 1.0:
        fmod = fcurve.modifiers[0]
        fmod.mode = 'POLYNOMIAL'
        fmod.poly_order = 1
        fmod.coefficients = (0, base_influence)


def generate_animation_rig(context):
    from . import widgets
    widgets.create()

    ob = context.active_object
    ob['wheels_on_y_axis'] = False
    ob['suspension_factor'] = .5
    ob['suspension_rolling_factor'] = .5
    amt = ob.data

    bpy.ops.object.mode_set(mode='EDIT')

    wheelFtR = amt.edit_bones['DEF-Wheel.Ft.R']
    wheelFtL = amt.edit_bones['DEF-Wheel.Ft.L']
    wheelBkR = amt.edit_bones['DEF-Wheel.Bk.R']
    wheelBkL = amt.edit_bones['DEF-Wheel.Bk.L']
    body = amt.edit_bones['DEF-Body']

    pos_front = (wheelFtR.head + wheelFtL.head) / 2
    pos_back = (wheelBkR.head + wheelBkL.head) / 2
    pos_body = body.head
    body_width = max([abs(w.head.x) + w.length for w in (wheelFtR, wheelFtL, wheelBkL, wheelBkR)])
    body_length = max(body.length, body_width)
    body_height = min(body_width, body_length) * 1.5
    body_height = max(body_height, pos_body.z * 3)

    root = amt.edit_bones.new('Root')
    root.head = (pos_front.x, pos_front.y, 0)
    root.tail = (pos_front.x, pos_front.y + body_length, 0)
    root.use_deform = False

    rootCenter = amt.edit_bones.new('Root.center')
    rootCenter.head = (body.head.x, body.head.y, 0)
    rootCenter.tail = (body.head.x, body.head.y + body_length, 0)
    rootCenter.use_deform = False
    rootCenter.parent = root

    drift = amt.edit_bones.new('Drift')
    drift.head = pos_front
    drift.tail = pos_front
    drift.tail.y -= body_length / 4
    drift.head.z = body_height * .8
    drift.tail.z = body_height * .8
    drift.roll = math.pi
    drift.use_deform = False
    drift.parent = root

    generate_animation_wheel_bones(amt, 'Ft.L', drift)
    generate_animation_wheel_bones(amt, 'Ft.R', drift)
    generate_animation_wheel_bones(amt, 'Bk.L', drift)
    generate_animation_wheel_bones(amt, 'Bk.R', drift)

    wheels = amt.edit_bones.new('Front Wheels')
    wheels.head = wheelFtL.head
    wheels.tail = wheelFtL.tail
    wheels.head.x = math.copysign(wheelFtL.head.x + 1.1 * wheelFtL.length, wheels.head.x)
    wheels.tail.x = math.copysign(wheelFtL.tail.x + 1.1 * wheelFtL.length, wheels.tail.x)
    wheels.use_deform = False
    wheels.parent = amt.edit_bones['WheelDamper.Ft.L']

    mch_wheels = amt.edit_bones.new('MCH-Wheels')
    mch_wheels.head = wheelFtL.head
    mch_wheels.tail = wheelFtL.tail
    mch_wheels.head.x /= 2
    mch_wheels.tail.x /= 2
    mch_wheels.use_deform = False
    mch_wheels.parent = drift

    wheels = amt.edit_bones.new('Back Wheels')
    wheels.head = wheelBkL.head
    wheels.tail = wheelBkL.tail
    wheels.head.x = math.copysign(wheelBkL.head.x + 1.1 * wheelBkL.length, wheels.head.x)
    wheels.tail.x = math.copysign(wheelBkL.tail.x + 1.1 * wheelBkL.length, wheels.tail.x)
    wheels.use_deform = False
    wheels.parent = amt.edit_bones['WheelDamper.Bk.L']

    axisFt = amt.edit_bones.new('MCH-Axis.Ft')
    axisFt.head = wheelFtR.head
    axisFt.tail = wheelFtL.head
    axisFt.use_deform = False
    axisFt.parent = drift

    axisBk = amt.edit_bones.new('MCH-Axis.Bk')
    axisBk.head = wheelBkR.head
    axisBk.tail = wheelBkL.head
    axisBk.use_deform = False
    axisBk.parent = drift

    suspensionBk = amt.edit_bones.new('MCH-Suspension.Bk')
    suspensionBk.head = pos_back
    suspensionBk.tail = pos_back
    suspensionBk.tail.y += 2
    suspensionBk.use_deform = False
    suspensionBk.parent = drift

    suspensionFt = amt.edit_bones.new('MCH-Suspension.Ft')
    suspensionFt.head = pos_front
    align_vector = suspensionBk.head - suspensionFt.head
    align_vector.magnitude = 2
    suspensionFt.tail = pos_front + align_vector
    suspensionFt.use_deform = False
    suspensionFt.parent = drift

    axis = amt.edit_bones.new('MCH-Axis')
    axis.head = suspensionFt.head
    axis.tail = suspensionBk.head
    axis.use_deform = False
    axis.parent = suspensionFt

    mchBody = amt.edit_bones.new('MCH-Body')
    mchBody.head = body.head
    mchBody.tail = body.tail
    mchBody.tail.y += 1
    mchBody.use_deform = False
    mchBody.parent = axis

    suspension = amt.edit_bones.new('Suspension')
    suspension.head = body.head
    suspension.tail = body.head
    suspension.tail.y += body_width
    suspension.head.z = body_height * 1.2
    suspension.tail.z = body_height * 1.2
    suspension.use_deform = False
    suspension.parent = axis

    mchSteering = amt.edit_bones.new('MCH-Steering')
    mchSteering.head = pos_front
    mchSteering.tail = pos_front
    mchSteering.tail.y += body_width
    mchSteering.use_deform = False
    mchSteering.parent = root

    steeringController = amt.edit_bones.new('MCH-Steering.controller')
    steeringController.head = mchSteering.head
    steeringController.tail = mchSteering.tail
    steeringController.head.y -= body.length * .5
    steeringController.tail.y -= body.length * .5
    steeringController.use_deform = False
    steeringController.parent = root

    steering = amt.edit_bones.new('Steering')
    steering.head = steeringController.head
    steering.tail = steeringController.tail
    steering.head.y -= body.length * .5
    steering.tail.y -= body.length * .5
    steering.use_deform = False
    steering.parent = steeringController

    deselect_edit_bones(ob)

    amt['Car Rig'] = True


def generate_animation_wheel_bones(amt, name_suffix, parent_bone):
    def_wheel_bone = amt.edit_bones['DEF-Wheel.%s' % name_suffix]

    ground_sensor = amt.edit_bones.new('GroundSensor.%s' % name_suffix)
    ground_sensor.head = def_wheel_bone.head
    ground_sensor.head.x = math.copysign(abs(def_wheel_bone.head.x) + def_wheel_bone.length * 1.1 / 2, ground_sensor.head.x)
    ground_sensor.tail = def_wheel_bone.tail
    ground_sensor.tail.x = math.copysign(abs(def_wheel_bone.tail.x) + def_wheel_bone.length * 1.1 / 2, ground_sensor.tail.x)
    ground_sensor.head.z = 0
    ground_sensor.tail.z = 0
    ground_sensor.use_deform = False
    ground_sensor.parent = parent_bone

    wheel_damper = amt.edit_bones.new('WheelDamper.%s' % name_suffix)
    wheel_damper.head = def_wheel_bone.head
    wheel_damper.tail = def_wheel_bone.tail
    wheel_damper.head.x = math.copysign(abs(def_wheel_bone.head.x) + 1.1 * def_wheel_bone.length, wheel_damper.head.x)
    wheel_damper.tail.x = math.copysign(abs(def_wheel_bone.tail.x) + 1.1 * def_wheel_bone.length, wheel_damper.tail.x)
    wheel_damper.head.z *= 1.5
    wheel_damper.tail.z *= 1.5
    wheel_damper.use_deform = False
    wheel_damper.parent = ground_sensor

    mch_wheel = amt.edit_bones.new('MCH-Wheel.%s' % name_suffix)
    mch_wheel.head = def_wheel_bone.head
    mch_wheel.tail = def_wheel_bone.tail
    mch_wheel.tail.y += 1
    mch_wheel.use_deform = False
    mch_wheel.parent = wheel_damper


def generate_constraints_on_rig(context):
    bpy.ops.object.mode_set(mode='POSE')
    ob = context.object
    ob.draw_type = 'WIRE'
    pose = ob.pose

    for b in pose.bones:
        if b.name.startswith('DEF-') or b.name.startswith('MCH-'):
            b.lock_location = (True, True, True)
            b.lock_rotation = (True, True, True)
            b.lock_scale = (True, True, True)
            b.lock_rotation_w = True

    generate_constraints_on_wheel_bones(ob, 'Ft.L')
    generate_constraints_on_wheel_bones(ob, 'Ft.R')
    generate_constraints_on_wheel_bones(ob, 'Bk.L')
    generate_constraints_on_wheel_bones(ob, 'Bk.R')

    wheels = pose.bones['Front Wheels']
    wheels.rotation_mode = "XYZ"
    wheels.lock_location = (True, True, True)
    wheels.lock_rotation = (False, True, True)
    wheels.lock_scale = (True, True, True)
    wheels.custom_shape = bpy.data.objects['WGT-CarRig.Wheel']
    cns = wheels.constraints.new('COPY_ROTATION')
    cns.name = 'Steering rotation'
    cns.target = ob
    cns.subtarget = 'MCH-Steering'
    cns.use_x = False
    cns.use_y = False
    cns.use_z = True
    cns.owner_space = 'LOCAL'
    cns.target_space = 'LOCAL'

    wheels = pose.bones['Back Wheels']
    wheels.rotation_mode = "XYZ"
    wheels.lock_location = (True, True, True)
    wheels.lock_rotation = (False, True, True)
    wheels.lock_scale = (True, True, True)
    wheels.custom_shape = bpy.data.objects['WGT-CarRig.Wheel']

    mch_wheel = pose.bones['MCH-Wheels']
    mch_wheel.rotation_mode = "XYZ"

    for suspension_pos in ('Ft', 'Bk'):
        mch_suspension = pose.bones['MCH-Suspension.%s' % suspension_pos]
        subtarget = 'MCH-Axis.%s' % suspension_pos
        cns = mch_suspension.constraints.new('COPY_LOCATION')
        cns.name = 'Location from %s' % subtarget
        cns.target = ob
        cns.subtarget = subtarget
        cns.head_tail = .5
        cns.use_x = False
        cns.use_y = False
        cns.use_z = True
        cns.owner_space = 'WORLD'
        cns.target_space = 'WORLD'
        create_constraint_influence_driver(ob, cns, '["suspension_factor"]')

        if suspension_pos == 'Ft':
            cns = mch_suspension.constraints.new('DAMPED_TRACK')
            cns.name = 'Track suspension back'
            cns.target = ob
            cns.subtarget = 'MCH-Suspension.Bk'
            cns.track_axis = 'TRACK_Y'

    for axis_pos in ('Ft', 'Bk'):
        mch_axis = pose.bones['MCH-Axis.%s' % axis_pos]
        cns = mch_axis.constraints.new('COPY_LOCATION')
        cns.name = 'Copy location from right wheel'
        cns.target = ob
        cns.subtarget = 'MCH-Wheel.%s.R' % axis_pos
        cns.use_x = True
        cns.use_y = True
        cns.use_z = True
        cns.owner_space = 'WORLD'
        cns.target_space = 'WORLD'

        mch_axis = pose.bones['MCH-Axis.%s' % axis_pos]
        cns = mch_axis.constraints.new('DAMPED_TRACK')
        cns.name = 'Track Left Wheel'
        cns.target = ob
        cns.subtarget = 'MCH-Wheel.%s.L' % axis_pos
        cns.track_axis = 'TRACK_Y'

    mch_axis = pose.bones['MCH-Axis']
    for axis_pos, influence in (('Ft', 1), ('Bk', .5)):
        subtarget = 'MCH-Axis.%s' % axis_pos
        cns = mch_axis.constraints.new('TRANSFORM')
        cns.name = 'Rotation from %s' % subtarget
        cns.target = ob
        cns.subtarget = subtarget
        cns.map_from = 'ROTATION'
        cns.from_min_x_rot = math.radians(-180)
        cns.from_max_x_rot = math.radians(180)
        cns.map_to_y_from = 'X'
        cns.map_to = 'ROTATION'
        cns.to_min_y_rot = math.radians(180)
        cns.to_max_y_rot = math.radians(-180)
        cns.owner_space = 'LOCAL'
        cns.target_space = 'LOCAL'
        create_constraint_influence_driver(ob, cns, '["suspension_rolling_factor"]', base_influence=influence)

    rootCenter = pose.bones['Root.center']
    rootCenter.lock_location = (True, True, True)
    rootCenter.lock_rotation = (True, True, True)
    rootCenter.lock_scale = (True, True, True)
    rootCenter.lock_rotation_w = True
    rootCenter.custom_shape = bpy.data.objects['WGT-CarRig.Root.center']

    root = pose.bones['Root']
    root.lock_scale = (True, True, True)
    root.custom_shape = bpy.data.objects['WGT-CarRig.Root']
    root.custom_shape_transform = rootCenter

    drift = pose.bones['Drift']
    drift.lock_location = (True, True, True)
    drift.lock_rotation = (True, True, False)
    drift.lock_scale = (True, True, True)
    drift.rotation_mode = 'ZYX'
    drift.custom_shape = bpy.data.objects['WGT-CarRig.Drift']

    suspension = pose.bones['Suspension']
    suspension.lock_rotation = (True, True, True)
    suspension.lock_scale = (True, True, True)
    suspension.lock_rotation_w = True
    suspension.custom_shape = bpy.data.objects['WGT-CarRig.Suspension']

    steering = pose.bones['Steering']
    steering.lock_location = (False, True, True)
    steering.lock_rotation = (True, True, True)
    steering.lock_scale = (True, True, True)
    steering.lock_rotation_w = True
    steering.custom_shape = bpy.data.objects['WGT-CarRig.Steering']

    mch_steering = pose.bones['MCH-Steering']
    cns = mch_steering.constraints.new('DAMPED_TRACK')
    cns.name = 'Track steering bone'
    cns.target = ob
    cns.subtarget = 'Steering'
    cns.track_axis = 'TRACK_NEGATIVE_Y'

    cns = mch_steering.constraints.new('COPY_ROTATION')
    cns.name = 'Drift counter animation'
    cns.target = ob
    cns.subtarget = 'Drift'
    cns.use_x = False
    cns.use_y = False
    cns.use_z = True
    cns.use_offset = True
    cns.owner_space = 'LOCAL'
    cns.target_space = 'LOCAL'

    mch_body = ob.pose.bones['MCH-Body']
    cns = mch_body.constraints.new('TRANSFORM')
    cns.name = 'Suspension on rollover'
    cns.target = ob
    cns.subtarget = 'Suspension'
    cns.map_from = 'LOCATION'
    cns.from_min_x = -2
    cns.from_max_x = 2
    cns.from_min_y = -2
    cns.from_max_y = 2
    cns.map_to_x_from = 'Y'
    cns.map_to_y_from = 'X'
    cns.map_to = 'ROTATION'
    cns.to_min_x_rot = math.radians(6)
    cns.to_max_x_rot = math.radians(-6)
    cns.to_min_y_rot = math.radians(-7)
    cns.to_max_y_rot = math.radians(7)
    cns.owner_space = 'LOCAL'
    cns.target_space = 'LOCAL'

    cns = mch_body.constraints.new('TRANSFORM')
    cns.name = 'Suspension on vertical'
    cns.target = ob
    cns.subtarget = 'Suspension'
    cns.map_from = 'LOCATION'
    cns.from_min_z = -0.5
    cns.from_max_z = 0.5
    cns.map_to_z_from = 'Z'
    cns.map_to = 'LOCATION'
    cns.to_min_z = -0.1
    cns.to_max_z = 0.1
    cns.owner_space = 'LOCAL'
    cns.target_space = 'LOCAL'

    body = ob.pose.bones['DEF-Body']
    cns = body.constraints.new('COPY_TRANSFORMS')
    cns.target = ob
    cns.subtarget = 'MCH-Body'

    create_bone_group(pose, 'Direction', color_set='THEME04', bone_names=('Root', 'Steering', 'Drift'))
    create_bone_group(pose, 'Wheel', color_set='THEME03', bone_names=('Front Wheels', 'Back Wheels'))
    create_bone_group(pose, 'Suspension', color_set='THEME09', bone_names=('Suspension', 'WheelDamper.Ft.L', 'WheelDamper.Ft.R', 'WheelDamper.Bk.L', 'WheelDamper.Bk.R'))
    create_bone_group(pose, 'GroundSensor', color_set='THEME02', bone_names=('GroundSensor.Ft.L', 'GroundSensor.Ft.R', 'GroundSensor.Bk.L', 'GroundSensor.Bk.R'))


def create_bone_group(pose, group_name, color_set, bone_names):
    group = pose.bone_groups.new(group_name)
    group.color_set = color_set
    for b in bone_names:
        pose.bones[b].bone_group = group


def generate_constraints_on_wheel_bones(ob, name_suffix):
    pose = ob.pose

    ground_sensor = pose.bones['GroundSensor.%s' % name_suffix]
    ground_sensor.lock_location = (True, True, False)
    ground_sensor.lock_rotation = (True, True, True)
    ground_sensor.lock_rotation_w = True
    ground_sensor.lock_scale = (True, True, True)
    ground_sensor.custom_shape = bpy.data.objects['WGT-CarRig.GroundSensor']
    cns = ground_sensor.constraints.new('SHRINKWRAP')
    cns.name = 'Ground projection'
    cns.shrinkwrap_type = 'PROJECT'
    cns.project_axis_space = 'LOCAL'
    cns.project_axis = 'NEG_Z'
    cns.distance = 0
    cns.is_proxy_local = True

    wheel_damper = pose.bones['WheelDamper.%s' % name_suffix]
    wheel_damper.lock_location = (True, True, False)
    wheel_damper.lock_rotation = (True, True, True)
    wheel_damper.lock_rotation_w = True
    wheel_damper.lock_scale = (True, True, True)
    wheel_damper.custom_shape = bpy.data.objects['WGT-CarRig.WheelDamper']

    mch_wheel = pose.bones['MCH-Wheel.%s' % name_suffix]
    mch_wheel.rotation_mode = "XYZ"

    if name_suffix.startswith('Ft.'):
        cns = mch_wheel.constraints.new('COPY_ROTATION')
        cns.name = 'Steering rotation'
        cns.target = ob
        cns.subtarget = 'MCH-Steering'
        cns.use_x = False
        cns.use_y = False
        cns.use_z = True
        cns.owner_space = 'LOCAL'
        cns.target_space = 'LOCAL'

    fcurve = mch_wheel.driver_add('rotation_euler', 0)
    drv = fcurve.driver
    drv.type = 'AVERAGE'
    var = drv.variables.new()
    var.name = 'x'
    var.type = 'TRANSFORMS'

    targ = var.targets[0]
    targ.id = ob
    targ.bone_target = 'MCH-Wheels'
    targ.transform_type = 'ROT_X'
    targ.transform_space = "LOCAL_SPACE"

    fmod = fcurve.modifiers[0]
    fmod.mode = 'POLYNOMIAL'
    fmod.poly_order = 1
    fmod.coefficients = (0, abs(1 / (mch_wheel.head.z if mch_wheel.head.z != 0 else 1)))

    cns = mch_wheel.constraints.new('TRANSFORM')
    cns.name = 'Wheel rotation along Y axis'
    cns.target = ob
    cns.subtarget = 'Root'
    cns.use_motion_extrapolate = True
    cns.map_from = 'LOCATION'
    cns.from_min_y = - math.pi * abs(mch_wheel.head.z if mch_wheel.head.z != 0 else 1)
    cns.from_max_y = - cns.from_min_y
    cns.map_to_x_from = 'Y'
    cns.map_to = 'ROTATION'
    cns.to_min_x_rot = math.pi
    cns.to_max_x_rot = -math.pi
    cns.owner_space = 'LOCAL'
    cns.target_space = 'LOCAL'

    create_constraint_influence_driver(ob, cns, '["wheels_on_y_axis"]')

    cns = mch_wheel.constraints.new('COPY_ROTATION')
    cns.name = 'Animation wheels'
    cns.target = ob
    cns.subtarget = '%s Wheels' % ('Front' if name_suffix.startswith('Ft.') else 'Back')
    cns.use_x = True
    cns.use_y = False
    cns.use_z = False
    cns.use_offset = True
    cns.owner_space = 'LOCAL'
    cns.target_space = 'LOCAL'

    def_wheel = pose.bones['DEF-Wheel.%s' % name_suffix]
    def_wheel.rotation_mode = "XYZ"
    cns = def_wheel.constraints.new('COPY_TRANSFORMS')
    cns.target = ob
    cns.subtarget = 'MCH-Wheel.%s' % name_suffix


def dispatch_bones_to_armature_layers(context):
    ob = context.object
    default_visible_layers = [False] * 32
    for b in ob.data.bones:
        layers = [False] * 32
        if b.name.startswith('DEF-'):
            layers[DEF_BONE_LAYER] = True
        elif b.name.startswith('MCH-'):
            layers[MCH_BONE_LAYER] = True
        else:
            layer_num = ob.pose.bones[b.name].bone_group_index
            layers[layer_num] = True
            default_visible_layers[layer_num] = True
        b.layers = layers

    ob.data.layers = default_visible_layers


class AddCarDeformationRigOperator(bpy.types.Operator):
    bl_idname = "object.armature_car_deformation_rig"
    bl_label = "Add car deformation rig"
    bl_options = {'REGISTER', 'UNDO'}

    body_pos_delta = bpy.props.FloatVectorProperty(name='Body Delta',
                                                   description='Adjust car body location',
                                                   size=3,
                                                   default=(0, 0, 0),
                                                   subtype='TRANSLATION')

    body_size_delta = bpy.props.FloatProperty(name='Body End Delta',
                                              description='Adjust car body bone length',
                                              default=.0,
                                              min=0,
                                              subtype='DISTANCE')

    front_wheel_pos_delta = bpy.props.FloatVectorProperty(name='Front Wheel Delta',
                                                          description='Adjust front wheels location',
                                                          size=3,
                                                          default=(0, 0, 0),
                                                          subtype='TRANSLATION')

    front_wheel_size_delta = bpy.props.FloatProperty(name='Front Wheel Radius delta',
                                                     description='Adjust front wheels radius',
                                                     default=.0,
                                                     min=0,
                                                     subtype='DISTANCE')

    back_wheel_pos_delta = bpy.props.FloatVectorProperty(name='Back Wheel Delta',
                                                         description='Adjust back wheels location',
                                                         size=3,
                                                         default=(0, 0, 0),
                                                         subtype='TRANSLATION')

    back_wheel_size_delta = bpy.props.FloatProperty(name='Back Wheel Radius Delta',
                                                    description='Adjust back wheels radius',
                                                    default=.0,
                                                    min=0,
                                                    subtype='DISTANCE')

    default_position = {
        'Body':       mathutils.Vector((0.0,  0,  .8)),
        'Wheel.Ft.L': mathutils.Vector((0.9, -2,  .5)),
        'Wheel.Ft.R': mathutils.Vector((-.9, -2,  .5)),
        'Wheel.Bk.L': mathutils.Vector((0.9,  2,  .5)),
        'Wheel.Bk.R': mathutils.Vector((-.9,  2,  .5))
    }

    def _create_bone(self, selected_objects, rig, name, delta_pos, delta_length):
        b = rig.data.edit_bones.new('DEF-' + name)

        for target_obj in selected_objects:
            if target_obj.name.endswith(name):
                b.head = target_obj.location + delta_pos
                b.tail = b.head
                b.tail.y += target_obj.dimensions[1] / 2 if target_obj.dimensions and target_obj.dimensions[0] != 0 else 1
                b.tail.y += delta_length
                target_obj.parent = rig
                target_obj.parent_bone = b.name
                target_obj.parent_type = 'BONE'
                target_obj.location += rig.matrix_world.to_translation()
                target_obj.matrix_parent_inverse = (rig.matrix_world * mathutils.Matrix.Translation(b.tail)).inverted()
                return

        b.head = self.default_position[name] + delta_pos
        b.tail = b.head
        b.tail.y += 1.0 + delta_length

    def execute(self, context):
        """Creates the meta rig with basic bones"""
        selected_objects = context.selected_objects

        amt = bpy.data.armatures.new('Car Rig Data')
        amt['Car Rig'] = False

        obj_data = bpy_extras.object_utils.object_data_add(context, amt, name='Car Rig')
        rig = obj_data.object

        bpy.ops.object.mode_set(mode='EDIT')

        self._create_bone(selected_objects, rig, 'Body', delta_pos=self.body_pos_delta, delta_length=self.body_size_delta)
        self._create_bone(selected_objects, rig, 'Wheel.Ft.L', delta_pos=self.front_wheel_pos_delta, delta_length=self.front_wheel_size_delta)
        self._create_bone(selected_objects, rig, 'Wheel.Ft.R', delta_pos=self.front_wheel_pos_delta.reflect(mathutils.Vector((1, 0, 0))), delta_length=self.front_wheel_size_delta)
        self._create_bone(selected_objects, rig, 'Wheel.Bk.L', delta_pos=self.back_wheel_pos_delta, delta_length=self.back_wheel_size_delta)
        self._create_bone(selected_objects, rig, 'Wheel.Bk.R', delta_pos=self.back_wheel_pos_delta.reflect(mathutils.Vector((1, 0, 0))), delta_length=self.back_wheel_size_delta)

        deselect_edit_bones(rig)

        bpy.ops.object.mode_set(mode='OBJECT')
        context.scene.update()
        return{'FINISHED'}


class GenerateCarAnimationRigOperator(bpy.types.Operator):
    bl_idname = "pose.car_animation_rig_generate"
    bl_label = "Generate car animation rig"
    bl_options = {'UNDO'}

    @classmethod
    def poll(cls, context):
        return (context.object is not None and context.object.data is not None and
                'Car Rig' in context.object.data and not context.object.data['Car Rig'])

    def execute(self, context):
        generate_animation_rig(context)
        generate_constraints_on_rig(context)
        dispatch_bones_to_armature_layers(context)
        return {"FINISHED"}


def register():
    bpy.utils.register_class(GenerateCarAnimationRigOperator)
    bpy.utils.register_class(AddCarDeformationRigOperator)


def unregister():
    bpy.utils.unregister_class(GenerateCarAnimationRigOperator)
    bpy.utils.unregister_class(AddCarDeformationRigOperator)


if __name__ == "__main__":
    register()
