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
import mathutils
import math
import itertools


def name_range(prefix):
    yield prefix
    for i in itertools.count(1):
        yield '%s.%03d' % (prefix, i)


def bone_range(bones, name_prefix):
    return map(lambda n: bones[n], itertools.takewhile(lambda n: n in bones, name_range(name_prefix)))


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


class VectorFCurvesEvaluator:

    def __init__(self, fcurves_evaluator):
        self.fcurves_evaluator = fcurves_evaluator

    def evaluate(self, f):
        return mathutils.Vector(self.fcurves_evaluator.evaluate(f))


class EulerToQuaternionFCurvesEvaluator:

    def __init__(self, fcurves_evaluator):
        self.fcurves_evaluator = fcurves_evaluator

    def evaluate(self, f):
        return mathutils.Euler(self.fcurves_evaluator.evaluate(f)).to_quaternion()


class BakingOperator:
    frame_start = bpy.props.IntProperty(name='Start Frame', min=1)
    frame_end = bpy.props.IntProperty(name='End Frame', min=1)
    keyframe_tolerance = bpy.props.FloatProperty(name='Keyframe tolerance', min=0, default=.4)

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
        self.frame_start, self.frame_end = action.frame_range

        return context.window_manager.invoke_props_dialog(self)

    def draw(self, context):
        self.layout.prop(self, 'frame_start')
        self.layout.prop(self, 'frame_end')
        self.layout.prop(self, 'keyframe_tolerance')

    def _create_rotation_evaluator(self, action, source_bone):
        fcurve_name = 'pose.bones["%s"].rotation_euler' % source_bone.name
        fc_root_rot = [action.fcurves.find(fcurve_name, i) for i in range(0, 3)]
        return EulerToQuaternionFCurvesEvaluator(FCurvesEvaluator(fc_root_rot, default_value=(.0, .0, .0)))

    def _create_location_evaluator(self, action, source_bone):
        fcurve_name = 'pose.bones["%s"].location' % source_bone.name
        fc_root_loc = [action.fcurves.find(fcurve_name, i) for i in range(0, 3)]
        return VectorFCurvesEvaluator(FCurvesEvaluator(fc_root_loc, default_value=(.0, .0, .0)))

    def _create_scale_evaluator(self, action, source_bone):
        fcurve_name = 'pose.bones["%s"].scale' % source_bone.name
        fc_root_loc = [action.fcurves.find(fcurve_name, i) for i in range(0, 3)]
        return VectorFCurvesEvaluator(FCurvesEvaluator(fc_root_loc, default_value=(1.0, 1.0, 1.0)))

    def _bake_action(self, context, *source_bones):
        action = context.object.animation_data.action
        nla_tweak_mode = context.object.animation_data.use_tweak_mode

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

        bpy.ops.nla.bake(frame_start=self.frame_start, frame_end=self.frame_end, only_selected=True, bake_types={'POSE'}, visual_keying=True)
        bpy.context.scene.update()
        baked_action = context.object.animation_data.action

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

    def _clear_property_fcurve(self, context, property_name):
        fcurve_datapath = '["%s"]' % property_name
        action = context.object.animation_data.action
        fc_rot = action.fcurves.find(fcurve_datapath)
        if fc_rot is not None:
            action.fcurves.remove(fc_rot)
        context.object[property_name] = .0

    def _create_property_fcurve(self, context, property_name):
        action = context.object.animation_data.action
        fcurve_datapath = '["%s"]' % property_name
        return action.fcurves.new(fcurve_datapath, 0, 'Wheels rotation')


class BakeWheelRotationOperator(bpy.types.Operator, BakingOperator):
    bl_idname = 'anim.car_wheels_rotation_bake'
    bl_label = 'Bake car wheels rotation'
    bl_description = 'Automatically generates wheels animation based on Root bone animation.'
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        self._bake_wheels_rotation(context)
        return {'FINISHED'}

    @cursor('WAIT')
    def _bake_wheels_rotation(self, context):
        bones = context.object.data.bones
        wheel_bones = tuple(bone_range(bones, 'MCH-Wheel.rotation.Ft.L'))
        wheel_bones += tuple(bone_range(bones, 'MCH-Wheel.rotation.Ft.R'))
        wheel_bones += tuple(bone_range(bones, 'MCH-Wheel.rotation.Bk.L'))
        wheel_bones += tuple(bone_range(bones, 'MCH-Wheel.rotation.Bk.R'))
        brake_bones = tuple(bones[name] for name in ('Front Wheels', 'Back Wheels') if name in bones)

        for property_name in map(lambda wheel_bone: wheel_bone.name.replace('MCH-', ''), wheel_bones):
            self._clear_property_fcurve(context, property_name)

        baked_action = self._bake_action(context, *wheel_bones + brake_bones)

        try:
            for wheel_bone in wheel_bones:
                brake_name = 'Front Wheels' if wheel_bone.name.startswith('MCH-Wheel.rotation.Ft.') else 'Back Wheels'
                self._bake_wheel_rotation(context, baked_action, wheel_bone, bones[brake_name])
        finally:
            bpy.data.actions.remove(baked_action)

    def _evaluate_distance_per_frame(self, action, bone, brake_bone):
        loc_evaluator = self._create_location_evaluator(action, bone)
        rot_evaluator = self._create_rotation_evaluator(action, bone)
        break_evaluator = self._create_scale_evaluator(action, brake_bone)

        radius = bone.length if bone.length > .0 else 1.0
        bone_init_vector = (bone.head_local - bone.tail_local).normalized()
        prev_pos = loc_evaluator.evaluate(self.frame_start)
        prev_speed = 0
        distance = 0
        yield self.frame_start, distance
        for f in range(self.frame_start + 1, self.frame_end):
            pos = loc_evaluator.evaluate(f)
            speed_vector = pos - prev_pos
            speed_vector *= 2 * break_evaluator.evaluate(f).y - 1
            rotation_quaternion = rot_evaluator.evaluate(f)
            bone_orientation = rotation_quaternion * bone_init_vector
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
        fc_rot = self._create_property_fcurve(context, bone.name.replace('MCH-', ''))

        for f, distance in self._evaluate_distance_per_frame(baked_action, bone, brake_bone):
            kf = fc_rot.keyframe_points.insert(f, distance)
            kf.interpolation = 'LINEAR'


class BakeSteeringOperator(bpy.types.Operator, BakingOperator):
    bl_idname = 'anim.car_steering_bake'
    bl_label = 'Bake car steering'
    bl_description = 'Automatically generates steering animation based on Root bone animation.'
    bl_options = {'REGISTER', 'UNDO'}

    rotation_factor = bpy.props.FloatProperty(name='Rotation factor', min=0.1, default=1.0)

    def draw(self, context):
        self.layout.prop(self, 'frame_start')
        self.layout.prop(self, 'frame_end')
        self.layout.prop(self, 'keyframe_tolerance')
        self.layout.prop(self, 'rotation_factor')

    def execute(self, context):
        if self.frame_end > self.frame_start:
            if 'Steering' in context.object.data.bones and 'MCH-Steering.rotation' in context.object.data.bones:
                steering = context.object.data.bones['Steering']
                mch_steering_rotation = context.object.data.bones['MCH-Steering.rotation']
                distance = (steering.head - mch_steering_rotation.head).length
                self._bake_steering_rotation(context, distance, mch_steering_rotation)
        return {'FINISHED'}

    def _evaluate_rotation_per_frame(self, action, bone):
        loc_evaluator = self._create_location_evaluator(action, bone)
        rot_evaluator = self._create_rotation_evaluator(action, bone)

        init_vector = bone.head - bone.tail
        current_pos = loc_evaluator.evaluate(self.frame_start)
        prev_rotation = .0
        last_frame = self.frame_start
        for f in range(self.frame_start, self.frame_end - 1):
            next_pos = loc_evaluator.evaluate(f + 1)
            # TODO better guess the minimun length and remove some wrong rotation
            if (next_pos - current_pos).length < .02:
                continue
            world_space_tangent_vector = next_pos - current_pos
            local_space_tangent_vector = rot_evaluator.evaluate(f).inverted() * world_space_tangent_vector
            current_rotation = local_space_tangent_vector.xy.angle_signed(init_vector.xy, prev_rotation)
            drop_keyframe = abs(prev_rotation - current_rotation) < self.keyframe_tolerance / 50
            if drop_keyframe and f > self.frame_start:
                continue
            # TODO should also take speed into account
            anticipation = abs(prev_rotation - current_rotation) * 7
            if f - last_frame > anticipation:
                yield f - anticipation, prev_rotation
            yield f, current_rotation
            last_frame = f
            prev_rotation = current_rotation
            current_pos = next_pos

        yield self.frame_end, prev_rotation

    @cursor('WAIT')
    def _bake_steering_rotation(self, context, distance, bone):
        self._clear_property_fcurve(context, 'Steering.rotation')
        fc_rot = self._create_property_fcurve(context, 'Steering.rotation')
        action = self._bake_action(context, bone)

        try:
            for f, rotation_angle in self._evaluate_rotation_per_frame(action, bone):
                kf = fc_rot.keyframe_points.insert(f, math.tan(rotation_angle * self.rotation_factor) * distance)
                kf.interpolation = 'LINEAR'
        finally:
            bpy.data.actions.remove(action)


def register():
    bpy.utils.register_class(BakeWheelRotationOperator)
    bpy.utils.register_class(BakeSteeringOperator)


def unregister():
    bpy.utils.unregister_class(BakeWheelRotationOperator)
    bpy.utils.unregister_class(BakeSteeringOperator)


if __name__ == "__main__":
    register()
