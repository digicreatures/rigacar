# ##### BEGIN GPL LICENSE BLOCK #####
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
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

bl_info = {
    "name": "Car Rig",
    "author": "David Gayerie (based on Ondrej Raha script)",
    "version": (0, 9),
    "blender": (2, 7, 9),
    "location": "View3D > Add > Armature",
    "description": "Creates Car Rig",
    "location": "Armature properties",
    "wiki_url": "",
    "tracker_url": "",
    "category": "Rigging"}


import bpy
import math
import bpy_extras
import mathutils

ANIM_BONE_LAYER=0
GROUND_SENSOR_BONE_LAYER=1
DEF_BONE_LAYER=30
MCH_BONE_LAYER=31
WIDGETS_LAYER = 19

def apply_layer(bone):
    layers = [False] * 32

    if bone.name.startswith('DEF-'):
        layers[DEF_BONE_LAYER] = True
    elif "GroundSensor" in bone.name:
        layers[GROUND_SENSOR_BONE_LAYER] = True
    elif bone.name.startswith('MCH-'):
        layers[MCH_BONE_LAYER] = True
    else:
        layers[ANIM_BONE_LAYER] = True

    bone.layers = layers

def create_constraint_influence_driver(ob, cns, driver_data_path, base_influence = 1.0):
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

def generate_rig(context):
    ob = context.active_object
    ob['wheels_on_y_axis'] = True
    ob['damper_factor'] = .5
    ob['damper_rolling_factor'] = .5
    amt = ob.data
    amt['Car Rig'] = True

    create_widgets()

    bpy.ops.object.mode_set(mode='EDIT')

    wheelFtR = amt.edit_bones['DEF-Wheel.Ft.R']
    wheelFtL = amt.edit_bones['DEF-Wheel.Ft.L']
    wheelBkR = amt.edit_bones['DEF-Wheel.Bk.R']
    wheelBkL = amt.edit_bones['DEF-Wheel.Bk.L']
    body = amt.edit_bones['DEF-Body']

    pos_front = (wheelFtR.head + wheelFtL.head) / 2
    pos_back = (wheelBkR.head + wheelBkL.head) / 2
    pos_body = body.head

    root = amt.edit_bones.new('Root')
    root.head = (body.head.x, body.head.y, 0)
    root.tail = (body.tail.x, body.tail.y, 0)
    root.use_deform = False

    drift = amt.edit_bones.new('Drift')
    drift.head = pos_front
    drift.tail = pos_front
    drift.head.y -= body.length * 0.5
    drift.tail.y -= body.length * 0.8
    drift.head.z = body.envelope_distance * 1.2
    drift.tail.z = body.envelope_distance * 1.2
    drift.roll = math.pi
    drift.use_deform = False
    drift.parent = root

    generate_wheel_bones(amt, 'Ft.L', drift)
    generate_wheel_bones(amt, 'Ft.R', drift)
    generate_wheel_bones(amt, 'Bk.L', drift)
    generate_wheel_bones(amt, 'Bk.R', drift)

    wheels = amt.edit_bones.new('Front Wheels')
    wheels.head = wheelFtL.head
    wheels.tail = wheelFtL.tail
    wheels.head.x += math.copysign(wheelFtL.envelope_distance * 1.3, wheels.head.x)
    wheels.tail.x += math.copysign(wheelFtL.envelope_distance * 1.3, wheels.tail.x)
    wheels.use_deform = False
    wheels.parent = amt.edit_bones['WheelBumper.Ft.L']

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
    wheels.head.x += math.copysign(wheelBkL.envelope_distance * 1.3, wheels.head.x)
    wheels.tail.x += math.copysign(wheelBkL.envelope_distance * 1.3, wheels.tail.x)
    wheels.use_deform = False
    wheels.parent = amt.edit_bones['WheelBumper.Bk.L']

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

    damperBk = amt.edit_bones.new('MCH-Damper.Bk')
    damperBk.head = pos_back
    damperBk.tail = pos_back
    damperBk.tail.y += 2
    damperBk.use_deform = False
    damperBk.parent = drift

    damperFt = amt.edit_bones.new('MCH-Damper.Ft')
    damperFt.head = pos_front
    align_vector = damperBk.head - damperFt.head
    align_vector.magnitude = 2
    damperFt.tail = pos_front + align_vector
    damperFt.use_deform = False
    damperFt.parent = drift

    axis = amt.edit_bones.new('MCH-Axis')
    axis.head = damperFt.head
    axis.tail = damperBk.head
    axis.use_deform = False
    axis.parent = damperFt

    mchBody = amt.edit_bones.new('MCH-Body')
    mchBody.head = body.head
    mchBody.tail = body.tail
    mchBody.tail.y += 1
    mchBody.use_deform = False
    mchBody.parent = axis

    body.parent = mchBody

    damper = amt.edit_bones.new('Damper')
    damper.head = body.head
    damper.tail = body.head
    damper.tail.y += body.envelope_distance
    damper.head.z = body.envelope_distance * 1.5
    damper.tail.z = body.envelope_distance * 1.5
    damper.use_deform = False
    damper.parent = axis

    mchSteering = amt.edit_bones.new('MCH-Steering')
    mchSteering.head = pos_front
    mchSteering.tail = pos_front
    mchSteering.tail.y += body.length * 0.3
    mchSteering.use_deform = False
    mchSteering.parent = root

    steeringController = amt.edit_bones.new('MCH-Steering.controller')
    steeringController.head = mchSteering.head
    steeringController.tail = mchSteering.head
    steeringController.head.y -= 2
    steeringController.use_deform = False
    steeringController.parent = root

    steering = amt.edit_bones.new('Steering')
    steering.head = steeringController.head
    steering.tail = steeringController.tail
    steering.use_deform = False
    steering.parent = steeringController

    for b in amt.edit_bones:
        apply_layer(b)

def generate_wheel_bones(amt, name_suffix, parent_bone):
    def_wheel_bone = amt.edit_bones['DEF-Wheel.%s' % name_suffix]

    ground_sensor = amt.edit_bones.new('GroundSensor.%s' % name_suffix)
    ground_sensor.head = def_wheel_bone.head
    ground_sensor.tail = def_wheel_bone.tail
    ground_sensor.head.z = 0
    ground_sensor.tail.z = 0
    ground_sensor.use_deform = False
    ground_sensor.parent = parent_bone

    wheel_bumper = amt.edit_bones.new('WheelBumper.%s' % name_suffix)
    wheel_bumper.head = def_wheel_bone.head
    wheel_bumper.tail = def_wheel_bone.tail
    wheel_bumper.head.x += math.copysign(def_wheel_bone.envelope_distance * 1.5, wheel_bumper.head.x)
    wheel_bumper.tail.x += math.copysign(def_wheel_bone.envelope_distance * 1.5, wheel_bumper.tail.x)
    wheel_bumper.head.z *= 1.5
    wheel_bumper.tail.z *= 1.5
    wheel_bumper.use_deform = False
    wheel_bumper.parent = ground_sensor

    mch_wheel = amt.edit_bones.new('MCH-Wheel.%s' % name_suffix)
    mch_wheel.head = def_wheel_bone.head
    mch_wheel.tail = def_wheel_bone.tail
    mch_wheel.tail.y += 1
    mch_wheel.use_deform = False
    mch_wheel.parent = wheel_bumper

    def_wheel_bone.parent = mch_wheel


def edit_generated_rig(context):
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

    edit_wheel_bones(ob, 'Ft.L')
    edit_wheel_bones(ob, 'Ft.R')
    edit_wheel_bones(ob, 'Bk.L')
    edit_wheel_bones(ob, 'Bk.R')

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

    for damper_pos in ('Ft', 'Bk'):
        mch_damper = pose.bones['MCH-Damper.%s' % damper_pos]
        subtarget = 'MCH-Axis.%s' % damper_pos
        cns = mch_damper.constraints.new('COPY_LOCATION')
        cns.name = 'Location from %s' % subtarget
        cns.target = ob
        cns.subtarget = subtarget
        cns.head_tail = .5
        cns.use_x = False
        cns.use_y = False
        cns.use_z = True
        cns.owner_space = 'WORLD'
        cns.target_space = 'WORLD'
        create_constraint_influence_driver(ob, cns, '["damper_factor"]')

        if damper_pos == 'Ft':
            cns = mch_damper.constraints.new('DAMPED_TRACK')
            cns.name = 'Track damper back'
            cns.target = ob
            cns.subtarget = 'MCH-Damper.Bk'
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
        create_constraint_influence_driver(ob, cns, '["damper_rolling_factor"]', base_influence = influence)

    root = pose.bones['Root']
    root.lock_scale = (True, True, True)
    root.custom_shape = bpy.data.objects['WGT-CarRig.Root']

    drift = pose.bones['Drift']
    drift.lock_location = (True, True, True)
    drift.lock_rotation = (True, True, False)
    drift.lock_scale = (True, True, True)
    drift.rotation_mode = 'ZYX'
    drift.custom_shape = bpy.data.objects['WGT-CarRig.Drift']

    damper = pose.bones['Damper']
    damper.lock_rotation = (True, True, True)
    damper.lock_scale = (True, True, True)
    damper.lock_rotation_w = True
    damper.custom_shape = bpy.data.objects['WGT-CarRig.Damper']

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
    cns.name = 'Damper on rollover'
    cns.target = ob
    cns.subtarget = 'Damper'
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
    cns.name = 'Damper on vertical'
    cns.target = ob
    cns.subtarget = 'Damper'
    cns.map_from = 'LOCATION'
    cns.from_min_z = -0.5
    cns.from_max_z = 0.5
    cns.map_to_z_from = 'Z'
    cns.map_to = 'LOCATION'
    cns.to_min_z = -0.1
    cns.to_max_z = 0.1
    cns.owner_space = 'LOCAL'
    cns.target_space = 'LOCAL'

    create_bone_group(pose, 'Damper', color_set='THEME09', bone_names=('Damper', 'WheelBumper.Ft.L', 'WheelBumper.Ft.R', 'WheelBumper.Bk.L', 'WheelBumper.Bk.R'))
    create_bone_group(pose, 'Direction', color_set='THEME04', bone_names=('Root', 'Steering', 'Drift'))
    create_bone_group(pose, 'Wheel', color_set='THEME03', bone_names=('Front Wheels', 'Back Wheels'))
    create_bone_group(pose, 'GroundSensor', color_set='THEME02', bone_names=('GroundSensor.Ft.L', 'GroundSensor.Ft.R', 'GroundSensor.Bk.L', 'GroundSensor.Bk.R'))


def create_bone_group(pose, group_name, color_set, bone_names):
    group = pose.bone_groups.new(group_name)
    group.color_set = color_set
    for b in bone_names:
        pose.bones[b].bone_group = group


def edit_wheel_bones(ob, name_suffix):
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

    wheel_bumper = pose.bones['WheelBumper.%s' % name_suffix]
    wheel_bumper.lock_location = (True, True, False)
    wheel_bumper.lock_rotation = (True, True, True)
    wheel_bumper.lock_rotation_w = True
    wheel_bumper.lock_scale = (True, True, True)
    wheel_bumper.custom_shape = bpy.data.objects['WGT-CarRig.WheelDamper']

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
    cns.map_from ='LOCATION'
    cns.from_min_y = - math.pi * abs(mch_wheel.head.z if mch_wheel.head.z != 0 else 1)
    cns.from_max_y = - cns.from_min_y
    cns.map_to_x_from = 'Y'
    cns.map_to ='ROTATION'
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


class BaseCarRigPanel:
    @classmethod
    def poll(cls, context):
        return 'Car Rig' in context.object.data if context.object is not None and context.object.data is not None else False

    def draw(self, context):
        if context.object.data['Car Rig']:
            self.layout.prop(context.object, '["wheels_on_y_axis"]', text = "Wheels on Y axis")
            self.layout.prop(context.object, '["damper_factor"]', text = "Damper fact.")
            self.layout.prop(context.object, '["damper_rolling_factor"]', text = "Damper rolling fact.")
            self.layout.operator('car.bake_wheel_rotation', 'Bake wheels rotation', 'Automatically generates wheels animation based on Root bone animation.')
            self.layout.operator('car.bake_steering_wheel_rotation', 'Bake steering wheels', 'Automatically generates wheels animation based on Root bone animation.')
        elif context.object.mode in {"POSE", "OBJECT"}:
            self.layout.operator("car.rig_generate", text='Generate')


class UICarRigPropertiesPanel(bpy.types.Panel, BaseCarRigPanel):
    bl_label = "Car Rig"
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "data"


class UICarRigView3DPanel(bpy.types.Panel, BaseCarRigPanel):
    bl_label = "Car Rig"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"


class AddCarMetaRigOperator(bpy.types.Operator):
    """Operator to create Car Meta Rig"""

    bl_idname = "car.meta_rig"
    bl_label = "Add car meta rig"
    bl_options = {'REGISTER', 'UNDO'}

    def _compute_envelope_distance(self, obj, bound_box_co_index, default_offset):
        if obj.bound_box is None:
            return default_offset

        max_x = max([abs(bb[bound_box_co_index]) for bb in obj.bound_box])

        return max_x if max_x > 0 else default_offset


    def _create_bone(self, selected_objects, rig, name, head):
        b = rig.data.edit_bones.new('DEF-' + name)
        b.head = head
        b.tail = b.head
        b.tail.y += 1.0
        b.envelope_distance = 0.25 if b.name != 'DEF-Body' else 1

        for target_obj in selected_objects:
            if target_obj.name == name:
                b.head = target_obj.location
                b.tail = b.head
                b.tail.y += target_obj.dimensions[1] / 2 if target_obj.dimensions and target_obj.dimensions[0] != 0 else 1
                if b.name == 'DEF-Body':
                    b.envelope_distance = self._compute_envelope_distance(target_obj, bound_box_co_index = 2, default_offset = 1)
                else:
                    b.envelope_distance = self._compute_envelope_distance(target_obj, bound_box_co_index = 0, default_offset = .25)
                target_obj.parent = rig
                target_obj.parent_bone = b.name
                target_obj.parent_type = 'BONE'
                target_obj.location += rig.matrix_world.to_translation()
                target_obj.matrix_parent_inverse = (rig.matrix_world * mathutils.Matrix.Translation(b.tail)).inverted()
                break

    def execute(self, context):
        """Creates the meta rig with basic bones"""

        selected_objects = context.selected_objects
        amt = bpy.data.armatures.new('Car Rig Data')
        amt['Car Rig'] = False
        obj_data = bpy_extras.object_utils.object_data_add(context, amt, name='Car Rig')
        rig = obj_data.object

        bpy.ops.object.mode_set(mode='EDIT')

        self._create_bone(selected_objects, rig, 'Body',      (  0,  0, .8))
        self._create_bone(selected_objects, rig, 'Wheel.Ft.L', ( .9, -2,  1))
        self._create_bone(selected_objects, rig, 'Wheel.Ft.R', (-.9, -2,  1))
        self._create_bone(selected_objects, rig, 'Wheel.Bk.L', ( .9,  2,  1))
        self._create_bone(selected_objects, rig, 'Wheel.Bk.R', (-.9,  2,  1))

        bpy.ops.object.mode_set(mode='OBJECT')

        return{'FINISHED'}


class GenerateCarRigOperator(bpy.types.Operator):
    bl_idname = "car.rig_generate"
    bl_label = "Generate Car Rig"
    bl_options = {'UNDO'}

    def execute(self, context):
        generate_rig(context)
        edit_generated_rig(context)
        return {"FINISHED"}


class FCurvesEvaluator:
    """Encapsulates a bunch of FCurves for vector animations."""

    def __init__(self, fcurves, default_value):
        self.default_value = default_value
        self.fcurves = fcurves

    def evaluate(self, f):
        result = []
        for fcurve, value in zip(self.fcurves, self.default_value):
            if fcurve is not None:
                result.append(fcurve.evaluate(f))
            else:
                result.append(value)
        return result


class BakeWheelRotationOperator(bpy.types.Operator):
    bl_idname = 'car.bake_wheel_rotation'
    bl_label = 'Car Rig: bake wheels rotation'
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return (context.object is not None and
                context.object.animation_data is not None and
                context.object.animation_data.action is not None and
                'Car Rig' in context.object.data and
                context.object.data['Car Rig'])

    def execute(self, context):
        self._bake_wheel_rotation(context.object.animation_data.action, context.object.data.bones['Root'], context.object.data.bones['MCH-Wheels'])
        context.object['wheels_on_y_axis'] = False
        return {'FINISHED'}

    def _create_rotation_evaluator(self, action, source_bone):
        fcurve_name = 'pose.bones["%s"].rotation_quaternion' % source_bone.name
        fc_root_rot = [action.fcurves.find(fcurve_name , i) for i in range(0, 4)]
        return FCurvesEvaluator(fc_root_rot, default_value= (1.0, .0, .0, .0))

    def _create_location_evaluator(self, action, source_bone):
        fcurve_name = 'pose.bones["%s"].location' % source_bone.name
        fc_root_loc = [action.fcurves.find(fcurve_name , i) for i in range(0, 3)]
        return FCurvesEvaluator(fc_root_loc, default_value= (.0, .0, .0))

    def _evaluate_distance_per_frame(self, action, source_bone):
        locEvaluator = self._create_location_evaluator(action, source_bone)
        rotEvaluator = self._create_rotation_evaluator(action, source_bone)

        start, end = action.frame_range
        if end - start <= 0:
            return

        source_bone_init_vector = (source_bone.head_local - source_bone.tail_local).normalized()
        prev_pos = mathutils.Vector(locEvaluator.evaluate(start))
        distance = 0
        for f in range(int(start), int(end)+1):
            pos = mathutils.Vector(locEvaluator.evaluate(f))
            rotation_quaternion = mathutils.Quaternion(rotEvaluator.evaluate(f))
            speed_vector = pos - prev_pos
            root_orientation = rotation_quaternion * source_bone_init_vector
            distance += math.copysign(speed_vector.magnitude, root_orientation.dot(speed_vector))
            # TODO yield only if speed has changed (avoid unecessary keyframes)
            yield f, distance
            prev_pos = pos

    def _bake_wheel_rotation(self, action, source_bone, target_bone):
        fcurve_datapath = 'pose.bones["%s"].rotation_euler' % target_bone.name
        fc_rot = action.fcurves.find(fcurve_datapath, 0)

        if fc_rot is not None:
            action.fcurves.remove(fc_rot)

        fc_rot = action.fcurves.new(fcurve_datapath, 0, target_bone.name)

        for f, distance in self._evaluate_distance_per_frame(action, source_bone):
            rotation = (distance + math.pi) % (2 * math.pi) - math.pi
            fc_rot.keyframe_points.insert(f, rotation)


class BakeSteeringWheelRotationOperator(bpy.types.Operator):
    bl_idname = 'car.bake_steering_wheel_rotation'
    bl_label = 'Car Rig: bake steering wheel rotation'
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return (context.object is not None and
                context.object.animation_data is not None and
                context.object.animation_data.action is not None and
                'Car Rig' in context.object.data and
                context.object.data['Car Rig'])

    def execute(self, context):
        steering = context.object.data.bones['Steering']
        mch_steering = context.object.data.bones['MCH-Steering']
        distance = (steering.head - mch_steering.head).length
        self._bake_steering_wheel_rotation(context.object.animation_data.action, distance, context.object.data.bones['Root'], context.object.data.bones['MCH-Steering.controller'])
        return {'FINISHED'}

    def _create_rotation_evaluator(self, action, source_bone):
        fcurve_name = 'pose.bones["%s"].rotation_quaternion' % source_bone.name
        fc_root_rot = [action.fcurves.find(fcurve_name , i) for i in range(0, 4)]
        return FCurvesEvaluator(fc_root_rot, default_value= (1.0, .0, .0, .0))

    def _evaluate_rotation_per_frame(self, action, source_bone):
        rotEvaluator = self._create_rotation_evaluator(action, source_bone)

        start, end = action.frame_range
        if end - start <= 0:
            return

        current_rotation_quaternion = mathutils.Quaternion(rotEvaluator.evaluate(start))
        for f in range(int(start), int(end)+1):
            next_rotation_quaternion = mathutils.Quaternion(rotEvaluator.evaluate(f + 1))
            rot_axis, rot_angle = current_rotation_quaternion.rotation_difference(next_rotation_quaternion).to_axis_angle()
            yield f, math.copysign(rot_angle, rot_axis.z)
            current_rotation_quaternion = next_rotation_quaternion

    def _bake_steering_wheel_rotation(self, action, distance, source_bone, target_bone):
        fcurve_datapath = 'pose.bones["%s"].location' % target_bone.name

        fc_rot = action.fcurves.find(fcurve_datapath, 0)
        if fc_rot is not None:
            action.fcurves.remove(fc_rot)

        fc_rot = action.fcurves.new(fcurve_datapath, 0, target_bone.name)

        for f, rotation_angle in self._evaluate_rotation_per_frame(action, source_bone):
            # TODO use correct ratio and correct bone
            fc_rot.keyframe_points.insert(f, math.tan(rotation_angle * 10) * distance)


def create_widgets():
    layers = [False] * 20
    layers[WIDGETS_LAYER] = True

    for name, widget in get_widgets().items():
        object_name = 'WGT-CarRig.%s' % name
        if object_name not in bpy.data.objects:
            m = bpy.data.meshes.new(object_name)
            m.from_pydata(widget['vertices'], widget['edges'], [])
            o = bpy.data.objects.new(object_name, m)
        else:
            o = bpy.data.objects[object_name]

        if object_name not in bpy.context.scene.objects:
            ob = bpy.context.scene.objects.link(o)
            ob.layers = layers


def get_widgets():
    widgets = {}
    widgets['Drift'] = {
        'vertices': [(1.4319738149642944, 0.6372261047363281, 0.0), (0.0, 2.996018886566162, 0.0), (-1.4319738149642944, 0.6372257471084595, 0.0),
                     (1.3195747137069702, 0.47894078493118286, 0.0), (1.1310639381408691, 0.3195039629936218, 0.0), (0.9425532817840576, 0.20844024419784546, 0.0),
                     (0.7540426254272461, 0.1276901364326477, 0.0), (0.5655319094657898, 0.06957072019577026, 0.0), (0.37702125310897827, 0.03012484312057495, 0.0),
                     (0.18851056694984436, 0.00718766450881958, 0.0), (-9.53415693061288e-08, -0.0003445744514465332, 0.0), (-0.1885107457637787, 0.00718766450881958, 0.0),
                     (-0.3770214319229126, 0.03012484312057495, 0.0), (-0.5655321478843689, 0.06957072019577026, 0.0), (-0.7540427446365356, 0.12769001722335815, 0.0),
                     (-0.9425533413887024, 0.2084401249885559, 0.0), (-1.1310639381408691, 0.3195039629936218, 0.0), (-1.3195745944976807, 0.47894060611724854, 0.0)],
        'edges': [(0, 1), (17, 2), (1, 2), (0, 3), (3, 4), (4, 5), (5, 6), (6, 7), (7, 8), (8, 9), (9, 10), (10, 11), (11, 12), (12, 13), (13, 14), (14, 15), (15, 16), (16, 17)]
    }

    widgets['Root'] = {
        'vertices': [(-0.5, -0.8844379782676697, 0.0), (-0.3844379782676697, -1.0, 0.0), (-0.4912033677101135, -0.9286617040634155, 0.0),
                     (-0.4661526679992676, -0.9661527276039124, 0.0), (-0.42866164445877075, -0.9912034273147583, 0.0), (0.3844379782676697, -1.0, 0.0),
                     (0.5, -0.8844379782676697, 0.0), (0.42866164445877075, -0.9912034273147583, 0.0), (0.4661526679992676, -0.9661527276039124, 0.0),
                     (0.4912033677101135, -0.9286617040634155, 0.0), (-0.5, 0.8844379782676697, 0.0), (-0.3844379782676697, 1.0, 0.0),
                     (-0.4912033677101135, 0.9286617040634155, 0.0), (-0.4661526679992676, 0.9661527276039124, 0.0), (-0.42866164445877075, 0.9912034273147583, 0.0),
                     (0.5, 0.8844379782676697, 0.0), (0.3844379782676697, 1.0, 0.0), (0.4912033677101135, 0.9286617040634155, 0.0),
                     (0.4661526679992676, 0.9661527276039124, 0.0), (0.42866164445877075, 0.9912034273147583, 0.0), (0.1234154999256134, -1.0, 0.0),
                     (-0.1234154999256134, -1.0, 0.0), (0.1234154999256134, -1.0971899032592773, 0.0), (-0.1234154999256134, -1.0971899032592773, 0.0),
                     (0.3031257688999176, -1.0971899032592773, 0.0), (-0.3031257688999176, -1.0971899032592773, 0.0), (0.0, -1.25, 0.0)],
        'edges': [(0, 2), (2, 3), (3, 4), (4, 1), (5, 7), (7, 8), (8, 9), (9, 6), (10, 12), (12, 13), (13, 14), (14, 11), (15, 17), (17, 18), (18, 19),
                  (19, 16), (0, 10), (6, 15), (11, 16), (20, 5), (21, 1), (22, 20), (23, 21), (24, 22), (25, 23), (26, 24), (26, 25)]
    }

    widgets['GroundSensor'] = {
        'vertices': [(-0.5, -0.822191596031189, 0.0), (-0.32219159603118896, -1.0, 0.0), (-0.4761781692504883, -0.9110957980155945, 0.0),
                     (-0.4110957980155945, -0.9761781692504883, 0.0), (0.32219159603118896, -1.0, 0.0), (0.5, -0.822191596031189, 0.0),
                     (0.4110957980155945, -0.9761781692504883, 0.0), (0.47617819905281067, -0.9110957980155945, 0.0), (-0.5, 0.822191596031189, 0.0),
                     (-0.32219159603118896, 1.0, 0.0), (-0.4761781692504883, 0.9110957980155945, 0.0), (-0.4110957980155945, 0.9761781692504883, 0.0),
                     (0.5, 0.822191596031189, 0.0), (0.32219159603118896, 1.0, 0.0), (0.4761781692504883, 0.9110957980155945, 0.0),
                     (0.4110957980155945, 0.9761781692504883, 0.0)],
        'edges': [(0, 2), (2, 3), (3, 1), (4, 6), (6, 7), (7, 5), (8, 10), (10, 11), (11, 9), (12, 14), (14, 15), (15, 13), (0, 8), (1, 4), (5, 12), (9, 13)]
    }

    widgets['Wheel'] = {
        'vertices': [(-1.1874363536890087e-07, 0.9999999403953552, -1.1874362826347351e-07), (-1.043081283569336e-07, 0.9807851910591125, 0.19509020447731018),
                     (-8.940696716308594e-08, 0.9238794445991516, 0.38268333673477173), (-1.1920928955078125e-07, 0.8314695358276367, 0.555570125579834),
                     (-5.960464477539063e-08, 0.7071067094802856, 0.7071066498756409), (-5.960464477539063e-08, 0.555570125579834, 0.8314695358276367),
                     (-5.960464477539063e-08, 0.3826833963394165, 0.9238793849945068), (-5.960464477539063e-08, 0.19509033858776093, 0.9807851314544678),
                     (-5.960464477539063e-08, 7.549790836947068e-08, 0.9999998807907104), (-5.960464477539063e-08, -0.195090189576149, 0.9807851910591125),
                     (-5.960464477539063e-08, -0.38268324732780457, 0.9238794445991516), (-5.960464477539063e-08, -0.555570125579834, 0.8314695358276367),
                     (-5.960464477539063e-08, -0.7071067094802856, 0.7071066498756409), (-1.1920928955078125e-07, -0.8314695954322815, 0.5555700659751892),
                     (-8.940696716308594e-08, -0.9238795638084412, 0.3826831579208374), (-1.043081283569336e-07, -0.9807852506637573, 0.19508996605873108),
                     (-1.1874365668518294e-07, -0.9999998211860657, -4.445849981493666e-07), (-1.341104507446289e-07, -0.9807851314544678, -0.19509084522724152),
                     (-1.4901161193847656e-07, -0.9238792657852173, -0.38268399238586426), (-1.1920928955078125e-07, -0.8314692378044128, -0.5555708408355713),
                     (-1.7881393432617188e-07, -0.7071062922477722, -0.7071073651313782), (-1.7881393432617188e-07, -0.555569589138031, -0.8314701318740845),
                     (-1.7881393432617188e-07, -0.3826826512813568, -0.9238799810409546), (-1.7881393432617188e-07, -0.1950894445180893, -0.9807855486869812),
                     (-1.1920928955078125e-07, 9.655991561885457e-07, -1.0000001192092896), (-1.7881393432617188e-07, 0.1950913369655609, -0.9807851910591125),
                     (-1.7881393432617188e-07, 0.3826844394207001, -0.9238792061805725), (-1.7881393432617188e-07, 0.5555711984634399, -0.8314690589904785),
                     (-1.7881393432617188e-07, 0.7071076035499573, -0.7071059942245483), (-1.1920928955078125e-07, 0.8314703106880188, -0.5555692315101624),
                     (-1.4901161193847656e-07, 0.9238800406455994, -0.382682204246521), (-1.341104507446289e-07, 0.9807854890823364, -0.1950889378786087),
                     (-1.1866376325997408e-07, 0.9439931511878967, -1.099180622077256e-07), (-1.0503674019446407e-07, 0.9258545637130737, 0.18416382372379303),
                     (-9.097014697090344e-08, 0.8721359372138977, 0.36125046014785767), (-1.191033334180247e-07, 0.7849016189575195, 0.5244544148445129),
                     (-6.283696052378218e-08, 0.6675039529800415, 0.667503833770752), (-6.283696052378218e-08, 0.5244544148445129, 0.7849015593528748),
                     (-6.283696052378218e-08, 0.36125051975250244, 0.8721358180046082), (-6.283696052378218e-08, 0.18416395783424377, 0.9258544445037842),
                     (-6.283696052378218e-08, 8.652769167838414e-08, 0.9439930319786072), (-6.283696052378218e-08, -0.18416379392147064, 0.925854504108429),
                     (-6.283696052378218e-08, -0.3612503409385681, 0.8721358776092529), (-6.283696052378218e-08, -0.5244543552398682, 0.7849015593528748),
                     (-6.283696052378218e-08, -0.6675038933753967, 0.667503833770752), (-1.191033334180247e-07, -0.7849015593528748, 0.5244543552398682),
                     (-9.097014697090344e-08, -0.8721359372138977, 0.36125028133392334), (-1.0503674019446407e-07, -0.9258545637130737, 0.18416360020637512),
                     (-1.1866378457625615e-07, -0.9439929723739624, -4.1751010826374113e-07), (-1.331699337470127e-07, -0.9258544445037842, -0.18416441977024078),
                     (-1.4723653407600068e-07, -0.8721356987953186, -0.3612510859966278), (-1.191033334180247e-07, -0.7849012613296509, -0.5244550704956055),
                     (-1.7536972052312194e-07, -0.6675034761428833, -0.6675045490264893), (-1.7536972052312194e-07, -0.52445387840271, -0.7849021553993225),
                     (-1.7536972052312194e-07, -0.36124980449676514, -0.8721364140510559), (-1.7536972052312194e-07, -0.18416307866573334, -0.9258548617362976),
                     (-1.191033334180247e-07, 9.267772043131117e-07, -0.9439932703971863), (-1.7536972052312194e-07, 0.18416491150856018, -0.925854504108429),
                     (-1.7536972052312194e-07, 0.36125150322914124, -0.8721356987953186), (-1.7536972052312194e-07, 0.5244554281234741, -0.7849010825157166),
                     (-1.7536972052312194e-07, 0.6675047874450684, -0.6675032377243042), (-1.191033334180247e-07, 0.7849022746086121, -0.5244535803794861),
                     (-1.4723653407600068e-07, 0.8721364140510559, -0.3612493872642517), (-1.331699337470127e-07, 0.9258548021316528, -0.18416263163089752)],
        'edges': [(1, 0), (2, 1), (3, 2), (4, 3), (5, 4), (6, 5), (7, 6), (8, 7), (9, 8), (10, 9), (11, 10), (12, 11), (13, 12), (14, 13), (15, 14), (16, 15),
                  (17, 16), (18, 17), (19, 18), (20, 19), (21, 20), (22, 21), (23, 22), (24, 23), (25, 24), (26, 25), (27, 26), (28, 27), (29, 28), (30, 29),
                  (31, 30), (0, 31), (33, 32), (34, 33), (35, 34), (36, 35), (37, 36), (38, 37), (39, 38), (40, 39), (41, 40), (42, 41), (43, 42), (44, 43),
                  (45, 44), (46, 45), (47, 46), (48, 47), (49, 48), (50, 49), (51, 50), (52, 51), (53, 52), (54, 53), (55, 54), (56, 55), (57, 56), (58, 57),
                  (59, 58), (60, 59), (61, 60), (62, 61), (63, 62), (32, 63)]
    }

    widgets['Steering'] = {
        'vertices': [(0.7296777367591858, 0.07034172862768173, 0.0), (0.057004380971193314, -0.07034172862768173, 0.0), (0.7296777367591858, -0.07034172862768173, 0.0),
                     (0.057004380971193314, 0.07034172862768173, 0.0), (0.7296777367591858, 0.16664999723434448, 0.0), (0.7296777367591858, -0.16664999723434448, 0.0),
                     (0.9998999834060669, 0.0, 0.0), (-0.7296777367591858, 0.07034172862768173, 0.0), (-0.057004380971193314, -0.07034172862768173, 0.0),
                     (-0.7296777367591858, -0.07034172862768173, 0.0), (-0.057004380971193314, 0.07034172862768173, 0.0), (-0.7296777367591858, 0.16664999723434448, 0.0),
                     (-0.7296777367591858, -0.16664999723434448, 0.0), (-0.9998999834060669, 0.0, 0.0)],
        'edges': [(2, 1), (3, 0), (1, 3), (0, 4), (5, 2), (4, 6), (6, 5), (9, 8), (10, 7), (8, 10), (7, 11), (12, 9), (11, 13), (13, 12)]
    }

    widgets['Damper'] = {
        'vertices': [(-0.42728525400161743, -0.12928833067417145, 0.11859216541051865), (-0.06909304857254028, 0.2578587830066681, 0.16264206171035767),
                     (-0.13347753882408142, 0.23118986189365387, 0.16264206171035767), (-0.1887657195329666, 0.1887657195329666, 0.16264206171035767),
                     (-0.23118992149829865, 0.13347753882408142, 0.16264206171035767), (-0.2578587830066681, 0.06909307092428207, 0.16264206171035767),
                     (-0.42728525400161743, -0.06909316033124924, 0.1368037909269333), (-0.2578587830066681, -0.06909302622079849, 0.16264206171035767),
                     (-0.23118986189365387, -0.13347747921943665, 0.16264206171035767), (-0.18876579403877258, -0.18876568973064423, 0.16264206171035767),
                     (-0.1334775984287262, -0.23118983209133148, 0.16264206171035767), (-0.06909313797950745, -0.2578587532043457, 0.16264206171035767),
                     (-0.42728525400161743, 0.06909292191267014, 0.1368037909269333), (0.06909293681383133, -0.2578587830066681, 0.16264206171035767),
                     (0.13347743451595306, -0.23118995130062103, 0.16264206171035767), (0.18876565992832184, -0.18876583874225616, 0.16264206171035767),
                     (0.23118983209133148, -0.1334775984287262, 0.16264206171035767), (0.2578587532043457, -0.06909316033124924, 0.16264206171035767),
                     (0.42728525400161743, -0.06909316033124924, 0.1368037909269333), (0.2578587830066681, 0.06909292191267014, 0.16264206171035767),
                     (0.23118992149829865, 0.13347740471363068, 0.16264206171035767), (0.18876585364341736, 0.18876561522483826, 0.16264206171035767),
                     (0.13347768783569336, 0.2311897873878479, 0.16264206171035767), (0.0690932497382164, 0.2578587532043457, 0.16264206171035767),
                     (0.42728525400161743, 0.06909292191267014, 0.1368037909269333), (0.42728525400161743, -0.12928833067417145, 0.11859216541051865),
                     (0.42728525400161743, 0.12928791344165802, 0.11859223246574402), (0.6011841893196106, -2.227646831443053e-07, 0.04259913042187691),
                     (-0.42728525400161743, 0.12928791344165802, 0.11859223246574402), (-0.6011841893196106, -2.227646831443053e-07, 0.04259913042187691),
                     (-0.06909316033124924, -0.42728525400161743, 0.1368037909269333), (0.06909292191267014, -0.42728525400161743, 0.1368037909269333),
                     (-0.12928833067417145, -0.42728525400161743, 0.11859216541051865), (0.12928785383701324, -0.42728525400161743, 0.11859225481748581),
                     (-2.545882011872891e-07, -0.6011841893196106, 0.042599156498909), (-0.06909316033124924, 0.42728525400161743, 0.1368037909269333),
                     (0.06909292191267014, 0.42728525400161743, 0.1368037909269333), (-0.12928833067417145, 0.42728525400161743, 0.11859216541051865),
                     (0.12928785383701324, 0.42728525400161743, 0.11859223246574402), (-2.545882011872891e-07, 0.6011841893196106, 0.04259913042187691),
                     (0.0, 0.1595769226551056, 0.16264206171035767), (-0.0797884613275528, 0.13819767534732819, 0.16264206171035767),
                     (-0.13819767534732819, 0.079788438975811, 0.16264206171035767), (-0.1595769226551056, -6.975329203129377e-09, 0.16264206171035767),
                     (-0.13819767534732819, -0.0797884613275528, 0.16264206171035767), (-0.0797884613275528, -0.13819767534732819, 0.16264206171035767),
                     (-2.409544741510672e-08, -0.1595769226551056, 0.16264206171035767), (0.07978841662406921, -0.13819767534732819, 0.16264206171035767),
                     (0.1381976306438446, -0.07978851348161697, 0.16264206171035767), (0.1595769226551056, -7.418926628588451e-08, 0.16264206171035767),
                     (0.13819773495197296, 0.07978837937116623, 0.16264206171035767), (0.07978855073451996, 0.1381976157426834, 0.16264206171035767),
                     (0.3002154231071472, 0.06909292191267014, 0.16078221797943115), (0.34257203340530396, 0.06909292191267014, 0.15552930533885956),
                     (0.3849286139011383, 0.06909292191267014, 0.1473732590675354), (0.3849286139011383, -0.06909316033124924, 0.14737321436405182),
                     (0.34257200360298157, -0.06909316033124924, 0.15552933514118195), (0.30021539330482483, -0.06909316033124924, 0.16078221797943115),
                     (-0.3002154231071472, 0.06909303367137909, 0.16078221797943115), (-0.34257203340530396, 0.0690929964184761, 0.15552930533885956),
                     (-0.3849286139011383, 0.06909295171499252, 0.1473732590675354), (-0.3849286139011383, -0.06909313052892685, 0.1473732590675354),
                     (-0.34257200360298157, -0.06909309327602386, 0.15552935004234314), (-0.30021539330482483, -0.06909305602312088, 0.16078220307826996),
                     (-0.06909314543008804, -0.30021539330482483, 0.16078221797943115), (-0.06909314543008804, -0.34257200360298157, 0.15552933514118195),
                     (-0.06909315288066864, -0.3849286139011383, 0.14737321436405182), (0.06909293681383133, -0.3002154231071472, 0.16078221797943115),
                     (0.06909292936325073, -0.34257203340530396, 0.15552930533885956), (0.06909292191267014, -0.3849286139011383, 0.1473732590675354),
                     (-0.06909307837486267, 0.3002154231071472, 0.16078221797943115), (-0.06909310817718506, 0.34257203340530396, 0.15552930533885956),
                     (-0.06909313052892685, 0.3849286139011383, 0.1473732590675354), (0.06909316033124924, 0.30021539330482483, 0.16078221797943115),
                     (0.06909307837486267, 0.34257200360298157, 0.15552933514118195), (0.0690930038690567, 0.3849286139011383, 0.14737321436405182)],
        'edges': [(2, 1), (3, 2), (4, 3), (5, 4), (12, 28), (0, 6), (8, 7), (9, 8), (10, 9), (11, 10), (28, 29), (29, 0), (14, 13), (15, 14), (16, 15), (17, 16),
                  (54, 24), (57, 17), (20, 19), (21, 20), (22, 21), (23, 22), (60, 12), (25, 18), (24, 26), (27, 25), (26, 27), (63, 7), (32, 30), (31, 33),
                  (34, 32), (33, 34), (66, 30), (69, 31), (37, 35), (36, 38), (39, 37), (38, 39), (72, 35), (75, 36), (41, 40), (42, 41), (43, 42), (44, 43),
                  (45, 44), (46, 45), (47, 46), (48, 47), (49, 48), (50, 49), (51, 50), (40, 51), (19, 52), (52, 53), (53, 54), (18, 55), (55, 56), (56, 57),
                  (5, 58), (58, 59), (59, 60), (6, 61), (61, 62), (62, 63), (11, 64), (64, 65), (65, 66), (13, 67), (67, 68), (68, 69), (1, 70), (70, 71),
                  (71, 72), (23, 73), (73, 74), (74, 75)]
    }

    widgets['WheelDamper'] = {
        'vertices': [(-0.17770397663116455, -0.09192684292793274, 0.14030149579048157), (-0.17770397663116455, -0.09192684292793274, 0.23383590579032898),
        (-0.10793274641036987, -0.16846297681331635, 0.13250692188739777), (-0.10793278366327286, -0.16846297681331635, 0.22604137659072876),
        (-0.009241040796041489, -0.1998595893383026, 0.12471239268779755), (-0.009241056628525257, -0.19985966384410858, 0.21824686229228973),
        (0.09192679077386856, -0.17770402133464813, 0.11691785603761673), (0.09192682057619095, -0.17770402133464813, 0.21045231819152832),
        (0.16846293210983276, -0.10793281346559525, 0.1091233491897583), (0.16846293210983276, -0.10793281346559525, 0.2026577889919281),
        (0.1998595893383026, -0.009241072461009026, 0.10132881999015808), (0.1998595893383026, -0.009241089224815369, 0.19486328959465027),
        (0.17770397663116455, 0.09192679077386856, 0.09353431314229965), (0.1777040809392929, 0.09192677587270737, 0.18706879019737244),
        (0.10793274641036987, 0.16846294701099396, 0.08573977649211884), (0.10793281346559525, 0.16846294701099396, 0.17927424609661102),
        (0.009240993298590183, 0.19985954463481903, 0.0779452696442604), (0.009241056628525257, 0.1998595893383026, 0.1714797168970108),
        (-0.09192684292793274, 0.17770391702651978, 0.07015073299407959), (-0.09192682057619095, 0.17770402133464813, 0.16368518769741058),
        (-0.16846294701099396, 0.10793264955282211, 0.06235618144273758), (-0.16846297681331635, 0.10793274641036987, 0.15589064359664917),
        (-0.19985954463481903, 0.009240960702300072, 0.05456162989139557), (-0.1998595893383026, 0.00924102496355772, 0.14809606969356537),
        (-0.17770391702651978, -0.09192687273025513, 0.04676707834005356), (-0.17770402133464813, -0.09192684292793274, 0.14030154049396515),
        (-0.1079326793551445, -0.16846294701099396, 0.03897252678871155), (-0.10793278366327286, -0.16846302151679993, 0.13250696659088135),
        (-0.00924097653478384, -0.19985954463481903, 0.03117799013853073), (-0.009241056628525257, -0.19985966384410858, 0.12471242249011993),
        (0.09192684292793274, -0.17770391702651978, 0.02338346280157566), (0.09192682057619095, -0.1777040809392929, 0.11691789329051971),
        (0.16846293210983276, -0.1079326942563057, 0.015588936395943165), (0.16846293210983276, -0.10793283581733704, 0.10912337899208069),
        (0.19985954463481903, -0.009240993298590183, 0.0077944230288267136), (0.1998595893383026, -0.009241105057299137, 0.10132886469364166),
        (0.17770391702651978, 0.09192683547735214, -9.42906623890849e-08), (0.1777040809392929, 0.09192677587270737, 0.09353433549404144),
        (0.1079326793551445, 0.16846290230751038, -0.007794610224664211), (0.10793281346559525, 0.16846294701099396, 0.08573982864618301),
        (0.00924097653478384, 0.19985945522785187, -0.015589137561619282), (0.009241056628525257, 0.1998595893383026, 0.0779452919960022),
        (-0.09192682057619095, 0.177703857421875, -0.023383673280477524), (-0.09192682057619095, 0.17770402133464813, 0.07015074789524078),
        (-0.16846290230751038, 0.10793262720108032, -0.031178224831819534), (-0.16846294701099396, 0.10793274641036987, 0.06235621124505997),
        (-0.19985945522785187, 0.009240944869816303, -0.03897276148200035), (-0.1998595893383026, 0.00924102496355772, 0.05456166714429855),
        (-0.177703857421875, -0.09192684292793274, -0.046767283231019974), (-0.17770402133464813, -0.09192684292793274, 0.04676711559295654),
        (-0.10793262720108032, -0.16846290230751038, -0.054561812430620193), (-0.10793278366327286, -0.16846302151679993, 0.038972560316324234),
        (-0.009240960702300072, -0.19985948503017426, -0.062356337904930115), (-0.00924102496355772, -0.19985966384410858, 0.031178021803498268),
        (0.09192684292793274, -0.177703857421875, -0.07015086710453033), (0.09192683547735214, -0.17770402133464813, 0.023383498191833496),
        (0.16846293210983276, -0.10793262720108032, -0.07794538885354996), (0.16846302151679993, -0.10793278366327286, 0.015588970854878426),
        (0.19985945522785187, -0.009240944869816303, -0.08573991060256958), (0.19985966384410858, -0.009241040796041489, 0.007794454228132963),
        (0.1777038276195526, 0.09192683547735214, -0.09353446215391159), (0.1777040809392929, 0.09192684292793274, -6.286042264491698e-08),
        (0.10793259739875793, 0.1684628427028656, -0.10132896900177002), (0.10793281346559525, 0.16846302151679993, -0.007794579025357962),
        (0.00924092996865511, 0.1998593956232071, -0.10912348330020905), (0.009241040796041489, 0.19985966384410858, -0.015589105896651745),
        (-0.09192682057619095, 0.17770376801490784, -0.11691801995038986), (-0.09192684292793274, 0.17770405113697052, -0.023383641615509987),
        (-0.1684628427028656, 0.10793255269527435, -0.12471257150173187), (-0.16846302151679993, 0.10793274641036987, -0.031178191304206848),
        (-0.1998593509197235, 0.009240913204848766, -0.1325071007013321), (-0.19985966384410858, 0.009240993298590183, -0.03897274285554886),
        (-0.17770376801490784, -0.09192683547735214, -0.1403016299009323), (-0.17770402133464813, -0.09192688763141632, -0.046767283231019974),
        (-0.10793255269527435, -0.1684628427028656, -0.14809612929821014), (-0.10793274641036987, -0.16846303641796112, -0.054561812430620193),
        (-0.009240896441042423, -0.1998593509197235, -0.15589067339897156), (-0.009240993298590183, -0.19985966384410858, -0.062356337904930115),
        (0.09192683547735214, -0.17770375311374664, -0.16368518769741058), (0.09192688763141632, -0.17770402133464813, -0.07015086710453033),
        (0.1684628129005432, -0.10793253034353256, -0.1714797168970108), (0.16846302151679993, -0.10793274641036987, -0.07794538885354996),
        (0.1998593509197235, -0.009240913204848766, -0.17927424609661102), (0.1998595893383026, -0.009241009131073952, -0.08573991060256958),
        (0.17770375311374664, 0.09192679077386856, -0.18706879019737244), (0.17770397663116455, 0.09192684292793274, -0.09353446215391159),
        (0.10793255269527435, 0.1684628278017044, -0.19486330449581146), (0.10793274641036987, 0.16846294701099396, -0.10132896900177002),
        (0.00924092996865511, 0.1998593509197235, -0.20265783369541168), (0.00924102496355772, 0.19985954463481903, -0.10912348330020905),
        (-0.09192678332328796, 0.17770370841026306, -0.2104523628950119), (-0.09192679077386856, 0.17770394682884216, -0.11691801995038986),
        (-0.1684628129005432, 0.10793255269527435, -0.21824689209461212), (-0.16846290230751038, 0.10793272405862808, -0.12471257150173187),
        (-0.19985929131507874, 0.00924092996865511, -0.22604137659072876), (-0.19985954463481903, 0.00924102496355772, -0.1325071007013321),
        (-0.17770370841026306, -0.09192678332328796, -0.23383590579032898), (-0.17770391702651978, -0.09192680567502975, -0.1403016299009323),
        (-0.21371114253997803, -0.21371114253997803, 0.24772925674915314), (0.21371114253997803, -0.21371114253997803, 0.24772925674915314),
        (-0.21371114253997803, 0.21371114253997803, 0.24772925674915314), (0.21371114253997803, 0.21371114253997803, 0.24772925674915314),
        (-0.21371114253997803, -0.21371114253997803, -0.24772925674915314), (0.21371114253997803, -0.21371114253997803, -0.24772925674915314),
        (-0.21371114253997803, 0.21371114253997803, -0.24772925674915314), (0.21371114253997803, 0.21371114253997803, -0.24772925674915314)],
        'edges': [(1, 3), (2, 0), (3, 5), (4, 2), (5, 7), (6, 4), (7, 9), (8, 6), (9, 11), (10, 8), (11, 13), (12, 10), (13, 15), (14, 12),
                  (15, 17), (16, 14), (17, 19), (18, 16), (19, 21), (20, 18), (21, 23), (22, 20), (23, 25), (24, 22), (25, 27), (26, 24),
                  (27, 29), (28, 26), (29, 31), (30, 28), (31, 33), (32, 30), (33, 35), (34, 32), (35, 37), (36, 34), (37, 39), (38, 36),
                  (39, 41), (40, 38), (41, 43), (42, 40), (43, 45), (44, 42), (45, 47), (46, 44), (47, 49), (48, 46), (49, 51), (50, 48),
                  (51, 53), (52, 50), (53, 55), (54, 52), (55, 57), (56, 54), (57, 59), (58, 56), (59, 61), (60, 58), (61, 63), (62, 60),
                  (63, 65), (64, 62), (65, 67), (66, 64), (67, 69), (68, 66), (69, 71), (70, 68), (71, 73), (72, 70), (73, 75), (74, 72),
                  (75, 77), (76, 74), (77, 79), (78, 76), (79, 81), (80, 78), (81, 83), (82, 80), (83, 85), (84, 82), (85, 87), (86, 84),
                  (87, 89), (88, 86), (89, 91), (90, 88), (91, 93), (92, 90), (93, 95), (94, 92), (95, 97), (96, 94), (100, 98), (98, 99),
                  (99, 101), (101, 100), (104, 102), (102, 103), (103, 105), (105, 104)]
    }
    return widgets


def menu_entries(menu, context):
    menu.layout.operator("car.meta_rig",text="Car (Meta-Rig)",icon='AUTO')


def register():
    bpy.types.INFO_MT_armature_add.append(menu_entries)
    bpy.utils.register_class(UICarRigPropertiesPanel)
    bpy.utils.register_class(GenerateCarRigOperator)
    bpy.utils.register_class(BakeWheelRotationOperator)
    bpy.utils.register_class(BakeSteeringWheelRotationOperator)
    bpy.utils.register_class(AddCarMetaRigOperator)
    bpy.utils.register_class(UICarRigView3DPanel)


def unregister():
    bpy.types.INFO_MT_armature_add.remove(menu_func)
    bpy.utils.unregister_class(UICarRigPropertiesPanel)
    bpy.utils.unregister_class(GenerateCarRigOperator)
    bpy.utils.unregister_class(BakeWheelRotationOperator)
    bpy.utils.unregister_class(BakeSteeringWheelRotationOperator)
    bpy.utils.unregister_class(AddCarMetaRigOperator)
    bpy.utils.unregister_class(UICarRigView3DPanel)


if __name__ == "__main__":
    register()
