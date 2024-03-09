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
import bpy_extras.anim_utils
import mathutils
import math
import itertools
import re


def cursor(cursor_mode):
    def cursor_decorator(func):
        def wrapper(self, context, *args, **kwargs):
            context.window.cursor_modal_set(cursor_mode)
            try:
                return func(self, context, *args, **kwargs)
            finally:
                context.window.cursor_modal_restore()

        return wrapper

    return cursor_decorator


def bone_name(prefix, position, side, index=0):
    if index == 0:
        return '%s.%s.%s' % (prefix, position, side)
    else:
        return '%s.%s.%s.%03d' % (prefix, position, side, index)


def bone_range(bones, name_prefix, position, side):
    for index in itertools.count():
        name = bone_name(name_prefix, position, side, index)
        if name in bones:
            yield bones[name]
        else:
            break


def find_wheelbrake_bone(bones, position, side, index):
    other_side = 'R' if side == 'L' else 'L'
    name_prefix = 'WheelBrake'
    bone = bones.get(bone_name(name_prefix, position, side, index))
    if bone:
        return bone
    bone = bones.get(bone_name(name_prefix, position, other_side, index))
    if bone:
        return bone
    if index > 0:
        bone = bones.get(bone_name(name_prefix, position, side))
        if bone:
            return bone
        bone = bones.get(bone_name(name_prefix, position, other_side))
        if bone:
            return bone
    backward_compatible_bone_name = '%s Wheels' % ('Front' if position == 'Ft' else 'Back')
    return bones.get(backward_compatible_bone_name)


def clear_property_animation(context, property_name, remove_keyframes=True):
    if remove_keyframes and context.object.animation_data and context.object.animation_data.action:
        fcurve_datapath = '["%s"]' % property_name
        action = context.object.animation_data.action
        fcurve = action.fcurves.find(fcurve_datapath)
        if fcurve is not None:
            action.fcurves.remove(fcurve)
    context.object[property_name] = .0


def create_property_animation(context, property_name):
    action = context.object.animation_data.action
    fcurve_datapath = '["%s"]' % property_name
    return action.fcurves.new(fcurve_datapath, index=0, action_group='Wheels rotation')


class FCurvesEvaluator(object):
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


class VectorFCurvesEvaluator(object):

    def __init__(self, fcurves_evaluator):
        self.fcurves_evaluator = fcurves_evaluator

    def evaluate(self, f):
        return mathutils.Vector(self.fcurves_evaluator.evaluate(f))


class EulerToQuaternionFCurvesEvaluator(object):

    def __init__(self, fcurves_evaluator):
        self.fcurves_evaluator = fcurves_evaluator

    def evaluate(self, f):
        return mathutils.Euler(self.fcurves_evaluator.evaluate(f)).to_quaternion()


class QuaternionFCurvesEvaluator(object):

    def __init__(self, fcurves_evaluator):
        self.fcurves_evaluator = fcurves_evaluator

    def evaluate(self, f):
        return mathutils.Quaternion(self.fcurves_evaluator.evaluate(f))


def fix_old_steering_rotation(rig_object):
    """
    Fix  armature generated with Rigacar version < 6.0
    """
    if rig_object.pose and rig_object.pose.bones:
        if 'MCH-Steering.rotation' in rig_object.pose.bones:
            rig_object.pose.bones['MCH-Steering.rotation'].rotation_mode = 'QUATERNION'


class BakingOperator(object):
    frame_start: bpy.props.IntProperty(name='Start Frame', min=1)
    frame_end: bpy.props.IntProperty(name='End Frame', min=1)
    keyframe_tolerance: bpy.props.FloatProperty(name='Keyframe tolerance', min=0, default=.01)

    @classmethod
    def poll(cls, context):
        return ('Car Rig' in context.object.data and
                context.object.data['Car Rig'] and
                context.object.mode in ('POSE', 'OBJECT'))

    def invoke(self, context, event):
        if context.object.animation_data is None:
            context.object.animation_data_create()
        if context.object.animation_data.action is None:
            context.object.animation_data.action = bpy.data.actions.new("%sAction" % context.object.name)

        action = context.object.animation_data.action
        self.frame_start = int(action.frame_range[0])
        self.frame_end = int(action.frame_range[1])

        return context.window_manager.invoke_props_dialog(self)

    def draw(self, context):
        self.layout.use_property_split = True
        self.layout.use_property_decorate = False
        self.layout.prop(self, 'frame_start')
        self.layout.prop(self, 'frame_end')
        self.layout.prop(self, 'keyframe_tolerance')

    def _create_euler_evaluator(self, action, source_bone):
        fcurve_name = 'pose.bones["%s"].rotation_euler' % source_bone.name
        fc_root_rot = [action.fcurves.find(fcurve_name, index=i) for i in range(3)]
        return EulerToQuaternionFCurvesEvaluator(FCurvesEvaluator(fc_root_rot, default_value=(.0, .0, .0)))

    def _create_quaternion_evaluator(self, action, source_bone):
        fcurve_name = 'pose.bones["%s"].rotation_quaternion' % source_bone.name
        fc_root_rot = [action.fcurves.find(fcurve_name, index=i) for i in range(4)]
        return QuaternionFCurvesEvaluator(FCurvesEvaluator(fc_root_rot, default_value=(1.0, .0, .0, .0)))

    def _create_location_evaluator(self, action, source_bone):
        fcurve_name = 'pose.bones["%s"].location' % source_bone.name
        fc_root_loc = [action.fcurves.find(fcurve_name, index=i) for i in range(3)]
        return VectorFCurvesEvaluator(FCurvesEvaluator(fc_root_loc, default_value=(.0, .0, .0)))

    def _create_scale_evaluator(self, action, source_bone):
        fcurve_name = 'pose.bones["%s"].scale' % source_bone.name
        fc_root_loc = [action.fcurves.find(fcurve_name, index=i) for i in range(3)]
        return VectorFCurvesEvaluator(FCurvesEvaluator(fc_root_loc, default_value=(1.0, 1.0, 1.0)))

    def _bake_action(self, context, *source_bones):
        action = context.object.animation_data.action
        nla_tweak_mode = context.object.animation_data.use_tweak_mode if hasattr(context.object.animation_data,
                                                                                 'use_tweak_mode') else False

        # saving context
        selected_bones = [b for b in context.object.data.bones if b.select]
        mode = context.object.mode
        for b in selected_bones:
            b.select = False

        bpy.ops.object.mode_set(mode='OBJECT')
        source_bones_matrix_basis = []
        for source_bone in source_bones:
            source_bones_matrix_basis.append(context.object.pose.bones[source_bone.name].matrix_basis.copy())
            source_bone.select = True

        baked_action = bpy_extras.anim_utils.bake_action(
            context.object,
            action=None,
            frames=range(self.frame_start, self.frame_end + 1),
            only_selected=True,
            do_pose=True,
            do_object=False,
            do_visual_keying=True,
        )

        # restoring context
        for source_bone, matrix_basis in zip(source_bones, source_bones_matrix_basis):
            context.object.pose.bones[source_bone.name].matrix_basis = matrix_basis
            source_bone.select = False
        for b in selected_bones:
            b.select = True

        bpy.ops.object.mode_set(mode=mode)

        if nla_tweak_mode:
            context.object.animation_data.use_tweak_mode = nla_tweak_mode
        else:
            context.object.animation_data.action = action

        return baked_action


class ANIM_OT_carWheelsRotationBake(bpy.types.Operator, BakingOperator):
    bl_idname = 'anim.car_wheels_rotation_bake'
    bl_label = 'Bake wheels rotation'
    bl_description = 'Automatically generates wheels animation based on Root bone animation.'
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        context.object['wheels_on_y_axis'] = False
        self._bake_wheels_rotation(context)
        return {'FINISHED'}

    @cursor('WAIT')
    def _bake_wheels_rotation(self, context):
        bones = context.object.data.bones

        wheel_bones = []
        brake_bones = []
        for position, side in itertools.product(('Ft', 'Bk'), ('L', 'R')):
            for index, wheel_bone in enumerate(bone_range(bones, 'MCH-Wheel.rotation', position, side)):
                wheel_bones.append(wheel_bone)
                brake_bones.append(find_wheelbrake_bone(bones, position, side, index) or wheel_bone)

        for property_name in map(lambda wheel_bone: wheel_bone.name.replace('MCH-', ''), wheel_bones):
            clear_property_animation(context, property_name)

        bones = set(wheel_bones + brake_bones)
        baked_action = self._bake_action(context, *bones)

        if baked_action is None:
            self.report({'WARNING'}, "Existing action failed to bake. Won't bake wheel rotation")
            return

        try:
            for wheel_bone, brake_bone in zip(wheel_bones, brake_bones):
                self._bake_wheel_rotation(context, baked_action, wheel_bone, brake_bone)
        finally:
            bpy.data.actions.remove(baked_action)

    def _evaluate_distance_per_frame(self, action, bone, brake_bone):
        loc_evaluator = self._create_location_evaluator(action, bone)
        rot_evaluator = self._create_euler_evaluator(action, bone)
        brake_evaluator = self._create_scale_evaluator(action, brake_bone)

        radius = bone.length if bone.length > .0 else 1.0
        bone_init_vector = (bone.head_local - bone.tail_local).normalized()
        prev_pos = loc_evaluator.evaluate(self.frame_start)
        prev_speed = 0
        distance = 0
        yield self.frame_start, distance
        for f in range(self.frame_start + 1, self.frame_end):
            pos = loc_evaluator.evaluate(f)
            speed_vector = pos - prev_pos
            speed_vector *= 2 * brake_evaluator.evaluate(f).y - 1
            rotation_quaternion = rot_evaluator.evaluate(f)
            bone_orientation = rotation_quaternion @ bone_init_vector
            speed = math.copysign(speed_vector.magnitude, bone_orientation.dot(speed_vector))
            speed /= radius
            drop_keyframe = False
            if speed == .0:
                drop_keyframe = prev_speed == speed
            elif prev_speed != .0:
                drop_keyframe = abs(1 - prev_speed / speed) < self.keyframe_tolerance / 10
            if not drop_keyframe:
                prev_speed = speed
                yield f - 1, distance
            distance += speed
            prev_pos = pos
        yield self.frame_end, distance

    def _bake_wheel_rotation(self, context, baked_action, bone, brake_bone):
        fc_rot = create_property_animation(context, bone.name.replace('MCH-', ''))

        # Reset the transform of the wheel bone, otherwise baking yields wrong results
        pb: bpy.types.PoseBone = context.object.pose.bones[bone.name]
        pb.matrix_basis.identity()

        for f, distance in self._evaluate_distance_per_frame(baked_action, bone, brake_bone):
            kf = fc_rot.keyframe_points.insert(f, distance)
            kf.interpolation = 'LINEAR'
            kf.type = 'JITTER'


class ANIM_OT_carSteeringBake(bpy.types.Operator, BakingOperator):
    bl_idname = 'anim.car_steering_bake'
    bl_label = 'Bake car steering'
    bl_description = 'Automatically generates steering animation based on Root bone animation.'
    bl_options = {'REGISTER', 'UNDO'}

    rotation_factor: bpy.props.FloatProperty(name='Rotation factor', min=.1, default=1)

    def draw(self, context):
        self.layout.use_property_split = True
        self.layout.use_property_decorate = False
        self.layout.prop(self, 'frame_start')
        self.layout.prop(self, 'frame_end')
        self.layout.prop(self, 'rotation_factor')
        self.layout.prop(self, 'keyframe_tolerance')

    def execute(self, context):
        if self.frame_end > self.frame_start:
            if 'Steering' in context.object.data.bones and 'MCH-Steering.rotation' in context.object.data.bones:
                steering = context.object.data.bones['Steering']
                mch_steering_rotation = context.object.data.bones['MCH-Steering.rotation']
                bone_offset = abs(steering.head_local.y - mch_steering_rotation.head_local.y)
                self._bake_steering_rotation(context, bone_offset, mch_steering_rotation)
        return {'FINISHED'}

    def _evaluate_rotation_per_frame(self, action, bone_offset, bone):
        loc_evaluator = self._create_location_evaluator(action, bone)
        rot_evaluator = self._create_quaternion_evaluator(action, bone)

        distance_threshold = pow(bone_offset * max(self.keyframe_tolerance, .001), 2)
        steering_threshold = bone_offset * self.keyframe_tolerance * .1
        bone_direction_vector = (bone.head_local - bone.tail_local).normalized()
        bone_normal_vector = mathutils.Vector((1, 0, 0))

        current_pos = loc_evaluator.evaluate(self.frame_start)
        previous_steering_position = None
        for f in range(self.frame_start, self.frame_end - 1):
            next_pos = loc_evaluator.evaluate(f + 1)
            steering_direction_vector = next_pos - current_pos

            if steering_direction_vector.length_squared < distance_threshold:
                continue

            rotation_quaternion = rot_evaluator.evaluate(f)
            world_space_bone_direction_vector = rotation_quaternion @ bone_direction_vector
            world_space_bone_normal_vector = rotation_quaternion @ bone_normal_vector

            projected_steering_direction = steering_direction_vector.dot(world_space_bone_direction_vector)
            if projected_steering_direction == 0:
                continue

            length_ratio = bone_offset * self.rotation_factor / projected_steering_direction
            steering_direction_vector *= length_ratio

            steering_position = mathutils.geometry.distance_point_to_plane(steering_direction_vector,
                                                                           world_space_bone_direction_vector,
                                                                           world_space_bone_normal_vector)

            if previous_steering_position is not None \
                    and abs(steering_position - previous_steering_position) < steering_threshold:
                continue

            yield f, steering_position
            current_pos = next_pos
            previous_steering_position = steering_position

    @cursor('WAIT')
    def _bake_steering_rotation(self, context, bone_offset, bone):
        clear_property_animation(context, 'Steering.rotation')
        fix_old_steering_rotation(context.object)
        fc_rot = create_property_animation(context, 'Steering.rotation')

        baked_action = self._bake_action(context, bone)
        if baked_action is None:
            self.report({'WARNING'}, "Existing action failed to bake. Won't bake steering rotation")
            return

        try:
            # Reset the transform of the steering bone, because baking action manipulates the transform
            # and evaluate_rotation_frame expects it at it's default position
            pb: bpy.types.PoseBone = context.object.pose.bones[bone.name]
            pb.matrix_basis.identity()

            for f, steering_pos in self._evaluate_rotation_per_frame(baked_action, bone_offset, bone):
                kf = fc_rot.keyframe_points.insert(f, steering_pos)
                kf.type = 'JITTER'
                kf.interpolation = 'LINEAR'
        finally:
            bpy.data.actions.remove(baked_action)


class ANIM_OT_carClearSteeringWheelsRotation(bpy.types.Operator):
    bl_idname = "anim.car_clear_steering_wheels_rotation"
    bl_label = "Clear baked animation"
    bl_description = "Clear generated rotation for steering and wheels"
    bl_options = {'REGISTER', 'UNDO'}

    clear_steering: bpy.props.BoolProperty(name="Steering", description="Clear generated animation for steering",
                                           default=True)
    clear_wheels: bpy.props.BoolProperty(name="Wheels", description="Clear generated animation for wheels",
                                         default=True)

    def draw(self, context):
        self.layout.use_property_decorate = False
        self.layout.label(text='Clear generated keyframes for')
        self.layout.prop(self, property='clear_steering')
        self.layout.prop(self, property='clear_wheels')

    @classmethod
    def poll(cls, context):
        return context.object is not None and context.object.data is not None and context.object.data.get('Car Rig')

    def execute(self, context):
        re_wheel_propname = re.compile(r'^Wheel\.rotation\.(Ft|Bk)\.[LR](\.\d+)?$')
        for prop in context.object.keys():
            if prop == 'Steering.rotation':
                clear_property_animation(context, prop, remove_keyframes=self.clear_steering)
            elif re_wheel_propname.match(prop):
                clear_property_animation(context, prop, remove_keyframes=self.clear_wheels)
        # this is a hack to force Blender to take into account the modification
        # of the properties by changing the object mode.
        # Don't know yet if it is specific to blender 2.80
        mode = context.object.mode
        bpy.ops.object.mode_set(mode='OBJECT' if mode == 'POSE' else 'POSE')
        bpy.ops.object.mode_set(mode=mode)
        return {'FINISHED'}


def register():
    bpy.utils.register_class(ANIM_OT_carWheelsRotationBake)
    bpy.utils.register_class(ANIM_OT_carSteeringBake)
    bpy.utils.register_class(ANIM_OT_carClearSteeringWheelsRotation)


def unregister():
    bpy.utils.unregister_class(ANIM_OT_carClearSteeringWheelsRotation)
    bpy.utils.unregister_class(ANIM_OT_carSteeringBake)
    bpy.utils.unregister_class(ANIM_OT_carWheelsRotationBake)


if __name__ == "__main__":
    register()
