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

def create_constraint_influence_driver(armature, cns, driver_data_path, base_influence = 1.0):
    fcurve = cns.driver_add('influence')
    drv = fcurve.driver
    drv.type = 'AVERAGE'
    var = drv.variables.new()
    var.name = 'influence'
    var.type = 'SINGLE_PROP'

    targ = var.targets[0]
    targ.id_type = 'ARMATURE'
    targ.id = armature
    targ.data_path = driver_data_path

    if base_influence != 1.0:
        fmod = fcurve.modifiers[0]
        fmod.mode = 'POLYNOMIAL'
        fmod.poly_order = 1
        fmod.coefficients = (0, base_influence)

def generate_rig(context):
    ob = context.active_object
    ob["Car Rig"] = True
    amt = ob.data
    amt['wheels_on_y_axis'] = True
    amt['damper_factor'] = .5
    amt['damper_rolling_factor'] = .5

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
    root.head = (pos_body.x, pos_body.y, 0)
    root.tail = (pos_body.x, pos_body.y + 3, 0)
    root.use_deform = False

    drift = amt.edit_bones.new('Drift')
    drift.head = (pos_front.x, pos_front.y, pos_front.z * 3)
    drift.tail = (pos_front.x, pos_front.y - 3, pos_front.z * 3)
    drift.roll = math.pi
    drift.use_deform = False
    drift.parent = root

    generate_wheel_bones(amt, 'Ft.L', drift)
    generate_wheel_bones(amt, 'Ft.R', drift)
    generate_wheel_bones(amt, 'Bk.L', drift)
    generate_wheel_bones(amt, 'Bk.R', drift)

    wheels = amt.edit_bones.new('Wheels')
    wheels.head = wheelFtL.head
    wheels.tail = wheelFtL.tail
    wheels.tail.y = wheels.tail.z * 1.2
    wheels.use_deform = False
    wheels.parent = amt.edit_bones['WheelBumper.Ft.L']

    mch_wheels = amt.edit_bones.new('MCH-Wheels')
    mch_wheels.head = wheelFtL.head
    mch_wheels.tail = wheelFtL.tail
    mch_wheels.head.x /= 2
    mch_wheels.tail.x /= 2
    mch_wheels.use_deform = False
    mch_wheels.parent = root

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

    damperFt = amt.edit_bones.new('MCH-Damper.Ft')
    damperFt.head = pos_front
    damperFt.tail = pos_front
    damperFt.tail.y += 2
    damperFt.use_deform = False
    damperFt.parent = drift

    damperBk = amt.edit_bones.new('MCH-Damper.Bk')
    damperBk.head = pos_back
    damperBk.tail = pos_back
    damperBk.tail.y += 2
    damperBk.use_deform = False
    damperBk.parent = drift

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
    damper.tail = body.tail
    damper.head.z *= 4
    damper.head.z += 2
    damper.tail.z *= 4
    damper.tail.z += 2
    damper.use_deform = False
    damper.parent = axis

    mchSteering = amt.edit_bones.new('MCH-Steering')
    mchSteering.head = pos_front
    mchSteering.tail = pos_front
    mchSteering.head.y -= 5
    mchSteering.tail.y *= 3
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
    wheel_bumper.head.z *= .2
    wheel_bumper.tail.z *= .2
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

    wheels = pose.bones['Wheels']
    wheels.lock_location = (True, True, True)
    wheels.lock_rotation = (False, True, True)
    wheels.lock_scale = (True, True, True)
    cns = wheels.constraints.new('COPY_ROTATION')
    cns.name = 'Steering rotation'
    cns.target = ob
    cns.subtarget = 'MCH-Steering'
    cns.use_x = False
    cns.use_y = False
    cns.use_z = True
    cns.owner_space = 'LOCAL'
    cns.target_space = 'LOCAL'

    mch_wheels = pose.bones['MCH-Wheels']
    mch_wheels.rotation_mode = "XYZ"
    cns = mch_wheels.constraints.new('COPY_ROTATION')
    cns.name = 'Animation wheels'
    cns.target = ob
    cns.subtarget = 'Wheels'
    cns.use_x = True
    cns.use_y = False
    cns.use_z = False
    cns.use_offset = True
    cns.owner_space = 'LOCAL'
    cns.target_space = 'LOCAL'

    cns = mch_wheels.constraints.new('TRANSFORM')
    cns.name = 'Wheel rotation along Y axis'
    cns.target = ob
    cns.subtarget = 'Root'
    cns.use_motion_extrapolate = True
    cns.map_from ='LOCATION'
    cns.from_min_y = 0
    cns.from_max_y = 10
    cns.map_to_x_from = 'Y'
    cns.map_to ='ROTATION'
    cns.to_min_x_rot = 0
    cns.to_max_x_rot = -5
    cns.owner_space = 'LOCAL'
    cns.target_space = 'LOCAL'

    create_constraint_influence_driver(ob.data, cns, '["wheels_on_y_axis"]')

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
        create_constraint_influence_driver(ob.data, cns, '["damper_factor"]')

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
        cns.from_min_x_rot = math.radians(-360)
        cns.from_max_x_rot = math.radians(360)
        cns.map_to_y_from = 'X'
        cns.map_to = 'ROTATION'
        cns.to_min_y_rot = math.radians(360)
        cns.to_max_y_rot = math.radians(-360)
        cns.owner_space = 'LOCAL'
        cns.target_space = 'LOCAL'
        create_constraint_influence_driver(ob.data, cns, '["damper_rolling_factor"]', base_influence = influence)

    root = pose.bones['Root']
    root.lock_scale = (True, True, True)

    drift = pose.bones['Drift']
    drift.lock_location = (True, True, True)
    drift.lock_rotation = (True, True, False)
    drift.lock_scale = (True, True, True)
    drift.rotation_mode = 'ZYX'

    damper = pose.bones['Damper']
    damper.lock_rotation = (True, True, True)
    damper.lock_scale = (True, True, True)
    damper.lock_rotation_w = True

    steering = pose.bones['Steering']
    steering.lock_location = (False, True, True)
    steering.lock_rotation = (True, True, True)
    steering.lock_scale = (True, True, True)
    steering.lock_rotation_w = True

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


def edit_wheel_bones(ob, name_suffix):
    pose = ob.pose

    ground_sensor = pose.bones['GroundSensor.%s' % name_suffix]
    ground_sensor.lock_location = (True, True, False)
    ground_sensor.lock_rotation = (True, True, True)
    ground_sensor.lock_rotation_w = True
    ground_sensor.lock_scale = (True, True, True)
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

    cns = mch_wheel.constraints.new('TRANSFORM')
    cns.name = 'Wheel rotation'
    cns.target = ob
    cns.subtarget = 'MCH-Wheels'
    cns.use_motion_extrapolate = True
    cns.map_from ='ROTATION'
    cns.from_min_x_rot = 0
    cns.from_max_x_rot = 2 * math.pi
    cns.map_to ='ROTATION'
    cns.to_min_x_rot = 0
    cns.to_max_x_rot = 2 * math.pi / mch_wheel.head.z
    cns.owner_space = 'LOCAL'
    cns.target_space = 'LOCAL'

    def_wheel = pose.bones['DEF-Wheel.%s' % name_suffix]
    def_wheel.rotation_mode = "XYZ"

class BaseCarRigPanel:
    @classmethod
    def poll(cls, context):
        return context.object is not None and "Car Rig" in context.object

    def draw(self, context):
        if not context.object["Car Rig"] and context.object.mode in {"POSE", "OBJECT"}:
            self.layout.operator("car.rig_generate", text='Generate')
        if context.object["Car Rig"]:
            self.layout.prop(context.object.data, '["wheels_on_y_axis"]', text = "Wheels on Y axis")
            self.layout.prop(context.object.data, '["damper_factor"]', text = "Damper fact.")
            self.layout.prop(context.object.data, '["damper_rolling_factor"]', text = "Damper rolling fact.")
            self.layout.operator('car.bake_wheel_rotation', 'Bake wheels rotation', 'Automatically generates wheels animation based on Root bone animation.')
            self.layout.operator('car.bake_steering_wheel_rotation', 'Bake steering wheels', 'Automatically generates wheels animation based on Root bone animation.')


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

    def _create_bone(self, selected_objects, rig, name, head):
        b = rig.data.edit_bones.new('DEF-' + name)
        b.head = head
        b.tail = b.head
        b.tail.y += 1.0

        for target_obj in selected_objects:
            if target_obj.name == name:
                b.head = target_obj.location
                b.tail = b.head
                b.tail.y += 1.0
                target_obj.parent = rig
                target_obj.parent_bone = b.name
                target_obj.parent_type = 'BONE'
                target_obj.location += rig.matrix_world.to_translation()
                target_obj.matrix_parent_inverse = (rig.matrix_world * mathutils.Matrix.Translation(b.tail)).inverted()

    def execute(self, context):
        """Creates the meta rig with basic bones"""

        selected_objects = context.selected_objects
        amt = bpy.data.armatures.new('Car Rig Data')
        obj_data = bpy_extras.object_utils.object_data_add(context, amt, name='Car Rig')
        rig = obj_data.object
        rig["Car Rig"] = False

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
                "Car Rig" in context.object and
                context.object["Car Rig"])

    def execute(self, context):
        self._bake_wheel_rotation(context.object.animation_data.action, context.object.data.bones['Root'], context.object.data.bones['MCH-Wheels'])
        context.object.data['wheels_on_y_axis'] = False
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

        fc_speed = action.fcurves.find(fcurve_datapath, 0)
        if fc_speed is not None:
            action.fcurves.remove(fc_speed)

        fc_speed = action.fcurves.new(fcurve_datapath, 0, 'Wheel rotation baking')

        for f, distance in self._evaluate_distance_per_frame(action, source_bone):
            fc_speed.keyframe_points.insert(f, distance)


class BakeSteeringWheelRotationOperator(bpy.types.Operator):
    bl_idname = 'car.bake_steering_wheel_rotation'
    bl_label = 'Car Rig: bake steering wheel rotation'
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return (context.object is not None and
                context.object.animation_data is not None and
                context.object.animation_data.action is not None and
                "Car Rig" in context.object and
                context.object["Car Rig"])

    def execute(self, context):
        self._bake_steering_wheel_rotation(context.object.animation_data.action, context.object.data.bones['Root'], context.object.data.bones['MCH-Steering.controller'])
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

    def _bake_steering_wheel_rotation(self, action, source_bone, target_bone):
        fcurve_datapath = 'pose.bones["%s"].location' % target_bone.name

        fc_rot = action.fcurves.find(fcurve_datapath, 0)
        if fc_rot is not None:
            action.fcurves.remove(fc_rot)

        fc_rot = action.fcurves.new(fcurve_datapath, 0, 'Wheel rotation baking')

        for f, rotation_angle in self._evaluate_rotation_per_frame(action, source_bone):
            # TODO use correct ratio and correct bone
            fc_rot.keyframe_points.insert(f, math.tan(rotation_angle) * target_bone.length * 10)


def menu_func(self, context):
    self.layout.operator("car.meta_rig",text="Car (Meta-Rig)",icon='AUTO')

def register():
    bpy.types.INFO_MT_armature_add.append(menu_func)
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
