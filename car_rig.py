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
    "author": "Ondrej Raha(lokhorn), David Gayerie",
    "version": (0, 3),
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
from bpy.props import *

ANIM_BONE_LAYER=0
GROUND_SENSOR_BONE_LAYER=1
DEF_BONE_LAYER=30
MCH_BONE_LAYER=31

def apply_layer(bone):
    layers = [False] * 32

    if bone.name.startswith('DEF-'):
        layers[DEF_BONE_LAYER] = True
    elif bone.name.startswith('MCH-'):
        if "GroundSensor" in bone.name:
            layers[GROUND_SENSOR_BONE_LAYER] = True
        else:
            layers[MCH_BONE_LAYER] = True
    else:
        layers[ANIM_BONE_LAYER] = True

    bone.layers = layers

class BakeWheelRotationOperator(bpy.types.Operator):
    bl_idname = 'car.bake_wheel_rotation'
    bl_label = 'Car Rig: bake wheels rotation'
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return context.object is not None and "Car Rig" in context.object

    def execute(self, context):
        self.bake_wheel_rotation(context.object)
        return {'FINISHED'}

    def evaluate_distance_per_frame(self, action):
        fc_root = []
        for i in range(0, 3):
            fc_root.append(action.fcurves.find('pose.bones["Root"].location', i))

        start = int(min([f.range()[0] for f in fc_root]))
        end = int(max([f.range()[1] for f in fc_root])) + 1

        yield start, 0

        prev_pos = mathutils.Vector((fc_root[0].evaluate(start), fc_root[1].evaluate(start), fc_root[2].evaluate(start)))
        distance = 0
        for f in range(start + 1, end):
            pos = mathutils.Vector((fc_root[0].evaluate(f), fc_root[1].evaluate(f), fc_root[2].evaluate(f)))
            distance += (pos - prev_pos).magnitude
            # TODO yield only if speed has changed (avoid unecessary keyframes)
            yield f, distance
            prev_pos = pos

    def bake_wheel_rotation(self, rig_ob):
        fcurve_datapath = 'pose.bones["Wheel rotation"].rotation_euler'
        action = rig_ob.animation_data.action

        # TODO fcurve should be recreated once we had remove unecessary keyframes
        fc_speed = action.fcurves.find(fcurve_datapath, 0)
        if fc_speed is None:
            fc_speed = action.fcurves.new(fcurve_datapath, 0, 'Wheel rotation')

        for f, distance in self.evaluate_distance_per_frame(action):
            # TODO compute real rotation on definitive bones
            fc_speed.keyframe_points.insert(f, distance)


def Generate():
    print("Starting car rig generation...")

    ob = bpy.context.active_object
    ob.name = "Car Rig"
    amt = ob.data
    amt['turning_wheels'] = True

    bpy.ops.object.mode_set(mode='EDIT')

    #####################################Computing Average Positions#################################
    pos_front = (amt.bones['DEF-Wheel.Ft.R'].head_local + amt.bones['DEF-Wheel.Ft.L'].head_local) /2
    pos_back = (amt.bones['DEF-Wheel.Bk.R'].head_local + amt.bones['DEF-Wheel.Bk.L'].head_local) /2
    pos_body = amt.bones['DEF-Body'].head_local

    #####################################Create Bones#################################
    Root = amt.edit_bones.new('Root')
    Root.head = (pos_body.x, pos_body.y, 0)
    Root.tail = (pos_body.x, pos_body.y + 3, 0)
    Root.use_deform = False

    wheelEngine = amt.edit_bones.new('MCH-Wheel.engine')
    wheelEngine.head = (pos_front.x, pos_front.y - 1, pos_front.z)
    wheelEngine.tail = (pos_front.x, pos_front.y - .5, pos_front.z)
    wheelEngine.parent = Root
    wheelEngine.use_deform = False

    axis = amt.edit_bones.new('MCH-axis')
    axis.head = pos_front
    axis.tail = pos_back
    axis.use_deform = False

    damperCenter = amt.edit_bones.new('MCH-Damper.center')
    damperCenter.head = amt.bones['DEF-Body'].head_local
    damperCenter.tail = (pos_body.x, pos_body.y + .5, pos_body.z)
    damperCenter.parent = Root
    damperCenter.use_deform = False

    body = amt.edit_bones['DEF-Body']
    body.parent = axis

    steeringWheel = amt.edit_bones.new('Steering')
    steeringWheel.head = (pos_front.x, pos_front.y - 2, pos_front.z)
    steeringWheel.tail = (pos_front.x, pos_front.y - 1.5, pos_front.z)
    steeringWheel.parent = Root
    steeringWheel.use_deform = False

    damperFront = amt.edit_bones.new('MCH-Damper.Ft')
    damperFront.head = amt.bones['DEF-Wheel.Ft.R'].head_local
    damperFront.tail = amt.bones['DEF-Wheel.Ft.L'].head_local
    damperFront.use_deform = False

    damperBack = amt.edit_bones.new('MCH-Damper.Bk')
    damperBack.head = amt.bones['DEF-Wheel.Bk.R'].head_local
    damperBack.tail = amt.bones['DEF-Wheel.Bk.L'].head_local
    damperBack.use_deform = False

    damper = amt.edit_bones.new('Damper')
    damper.head = (pos_body.x, pos_body.y, pos_body.z + 2)
    damper.tail = (pos_body.x, pos_body.y + 1, pos_body.z + 2)
    damper.parent = damperCenter
    damper.use_deform = False

    for w in ('DEF-Wheel.Ft.L', 'DEF-Wheel.Ft.R', 'DEF-Wheel.Bk.L', 'DEF-Wheel.Bk.R'):
        amt.edit_bones[w].parent = damperCenter

        sensor = amt.edit_bones.new(w.replace('DEF-Wheel', 'MCH-GroundSensor'))
        sensor.head = amt.bones[w].head_local
        sensor.tail = amt.bones[w].head_local
        sensor.tail.y += 0.3
        sensor.parent = damperCenter
        sensor.use_deform = False

    WheelRot = amt.edit_bones.new('Wheel rotation')
    WheelRot.head = amt.bones['DEF-Wheel.Ft.L'].head_local
    WheelRot.tail = amt.bones['DEF-Wheel.Ft.L'].tail_local
    WheelRot.tail.y += 0.3
    WheelRot.parent = damperCenter
    WheelRot.use_deform = False

    for b in amt.edit_bones:
        apply_layer(b)

    #####################################Pose Constraints#################################
    bpy.ops.object.mode_set(mode='POSE')

    # Lock transformation on steering wheel
    steeringWheel = ob.pose.bones['Steering']
    steeringWheel.lock_location = (False, True, True)
    steeringWheel.lock_rotation = (True, True, True)
    steeringWheel.lock_rotation_w = True

    # Constaints on wheels
    add_wheel_constraints(ob, 'DEF-Wheel.Ft.L', 'MCH-GroundSensor.Ft.L')
    add_wheel_constraints(ob, 'DEF-Wheel.Ft.R', 'MCH-GroundSensor.Ft.R')
    add_wheel_constraints(ob, 'DEF-Wheel.Bk.L', 'MCH-GroundSensor.Bk.L')
    add_wheel_constraints(ob, 'DEF-Wheel.Bk.R', 'MCH-GroundSensor.Bk.R')

    # Transformation constraint Body -> damper
    damperCenter = ob.pose.bones['DEF-Body']
    cns4 = damperCenter.constraints.new('TRANSFORM')
    cns4.target = ob
    cns4.subtarget = 'Damper'
    cns4.from_min_x = -0.3
    cns4.from_max_x = 0.3
    cns4.from_min_y = -0.3
    cns4.from_max_y = 0.3
    cns4.map_to_x_from = "Y"
    cns4.map_to_y_from = "X"
    cns4.map_to = "ROTATION"
    cns4.to_min_x_rot = math.radians(6)
    cns4.to_max_x_rot = math.radians(-6)
    cns4.to_min_y_rot = math.radians(-7)
    cns4.to_max_y_rot = math.radians(7)
    cns4.owner_space = 'LOCAL'
    cns4.target_space = 'LOCAL'
     # Transformation constraint Body -> damper
    damperCenter = ob.pose.bones['DEF-Body']
    cns4 = damperCenter.constraints.new('TRANSFORM')
    cns4.target = ob
    cns4.subtarget = 'Damper'
    cns4.from_min_z = -0.5
    cns4.from_max_z = 0.5
    cns4.to_min_z = -0.1
    cns4.to_max_z = 0.1
    cns4.owner_space = 'LOCAL'
    cns4.target_space = 'LOCAL'


    # Copy Location constraint axis -> damperBack
    axis = ob.pose.bones['MCH-axis']
    cns5 = axis.constraints.new('COPY_LOCATION')
    cns5.target = ob
    cns5.subtarget = 'MCH-Damper.Ft'
    cns5.head_tail = 0.5
    # Tract To constraint axis -> damperFront
    cns6 = axis.constraints.new('TRACK_TO')
    cns6.target = ob
    cns6.subtarget = 'MCH-Damper.Bk'
    cns6.head_tail = 0.5
    cns6.track_axis = 'TRACK_Y'
    cns6.use_target_z = True
    cns6.owner_space = 'POSE'
    cns6.target_space = 'POSE'
    # Damped Track constraint axis -> damperBack
    cns7 = axis.constraints.new('DAMPED_TRACK')
    cns7.influence = 0.5
    cns7.target = ob
    cns7.subtarget = 'MCH-Damper.Ft'
    cns7.track_axis = "TRACK_NEGATIVE_X"
    cns7.influence = 0.5

    # Copy Location constraint damperBack -> BRWheel
    damperBack = ob.pose.bones['MCH-Damper.Bk']
    cns8 = damperBack.constraints.new('COPY_LOCATION')
    cns8.target = ob
    cns8.subtarget = 'MCH-GroundSensor.Bk.R'
    # Track To constraint damperBack -> BLWheel
    cns9 = damperBack.constraints.new('TRACK_TO')
    cns9.target = ob
    cns9.subtarget = 'MCH-GroundSensor.Bk.L'

    # Copy Location constraint damperFront -> FRWheel
    damperFront = ob.pose.bones['MCH-Damper.Ft']
    cns10 = damperFront.constraints.new('COPY_LOCATION')
    cns10.target = ob
    cns10.subtarget = 'MCH-GroundSensor.Ft.R'
    # Track To constraint damperFront -> FLWheel
    cns11 = damperFront.constraints.new('TRACK_TO')
    cns11.target = ob
    cns11.subtarget = 'MCH-GroundSensor.Ft.L'

    # Copy Location constraint Sensors ->
    for sensor_name in ('MCH-GroundSensor.Ft.L', 'MCH-GroundSensor.Ft.R', 'MCH-GroundSensor.Bk.L', 'MCH-GroundSensor.Bk.R'):
        Sensor = ob.pose.bones[sensor_name]
        Sensor.lock_location = (True,True,False)
        cns = Sensor.constraints.new('SHRINKWRAP')
        cns.distance = Sensor.head.z

    # Copy Location constraint WheelRot -> FLSensor
    WheelRot = ob.pose.bones['Wheel rotation']
    WheelRot.rotation_mode = "XYZ"
    WheelRot.lock_location = (True,True,True)
    WheelRot.lock_rotation = (False,True,True)

    cns = WheelRot.constraints.new('COPY_LOCATION')
    cns.target = ob
    cns.subtarget = 'MCH-GroundSensor.Ft.L'
    cns.use_x = False
    cns.use_y = False

    ob.select = True
    bpy.context.scene.objects.active = ob

    print("Generate Finished")


def add_wheel_constraints(ob, wheel_name, sensor_name):
    wheel = ob.pose.bones[wheel_name]
    wheel.rotation_mode = "XYZ"

    # Copy Location constraint from ground sensor
    cns = wheel.constraints.new('COPY_LOCATION')
    cns.target = ob
    cns.subtarget = sensor_name
    cns.use_x = False
    cns.use_y = False

    if wheel_name in ('DEF-Wheel.Ft.L', 'DEF-Wheel.Ft.R'):
        cns = wheel.constraints.new('TRANSFORM')
        cns.target = ob
        cns.subtarget = 'Steering'
        cns.from_min_x = -1
        cns.from_max_x = 1
        cns.map_to_z_from = "X"
        cns.map_to = "ROTATION"
        cns.to_min_z_rot = math.radians(-25)
        cns.to_max_z_rot = math.radians(25)
        cns.owner_space = 'LOCAL'
        cns.target_space = 'LOCAL'

    # Copy Rotation constraint XXWheel -> WheelRot
    cns = wheel.constraints.new('COPY_ROTATION')
    cns.target = ob
    cns.subtarget = 'Wheel rotation'
    cns.use_y = False
    cns.use_z = False
    cns.owner_space = 'LOCAL'
    cns.target_space = 'LOCAL'

    # Transformation constraint XXWheel -> wheelEngine
    cns = wheel.constraints.new('TRANSFORM')
    cns.target = ob
    cns.subtarget = 'MCH-Wheel.engine'
    cns.use_motion_extrapolate = True
    cns.from_min_y = 0
    cns.from_max_y = wheel.head.z * math.pi
    cns.map_to_x_from = "Y"
    cns.map_to = "ROTATION"
    cns.to_min_x_rot = 0
    cns.to_max_x_rot = -math.pi
    cns.owner_space = 'LOCAL'
    cns.target_space = 'LOCAL_WITH_PARENT'

    # Driver on influence for Transformation constraint
    fcurve = cns.driver_add('influence')
    drv = fcurve.driver
    drv.type = 'AVERAGE'
    var = drv.variables.new()
    var.name = 'influence'
    var.type = 'SINGLE_PROP'

    targ = var.targets[0]
    targ.id_type = 'ARMATURE'
    targ.id = ob.data
    targ.data_path = '["turning_wheels"]'


#generate button
class UImetaRigGenerate(bpy.types.Panel):
    bl_label = "Car Rig"
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "data"

    @classmethod
    def poll(cls, context):
        return context.object is not None and "Car Rig" in context.object

    def draw(self, context):
        obj = bpy.context.active_object
        if obj.mode in {"POSE", "OBJECT"}:
            self.layout.operator("car.rig_generate", text='Generate')


### Add panel to properties to adjust wheel size
class UIPanel(bpy.types.Panel):
    bl_label = "Car Rig"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"

    @classmethod
    def poll(cls, context):
        return context.object is not None and "Car Rig" in context.object

    def draw(self, context):
        if 'turning_wheels' in context.object.data:
            self.layout.prop(context.object.data, '["turning_wheels"]', text = "Wheels")
            self.layout.operator('car.bake_wheel_rotation', 'Bake wheels animation', 'Automatically generates wheels animation based on Root bone animation.')

class AddCarMetaRig(bpy.types.Operator):
    """Operator to create Car Meta Rig"""

    bl_idname = "car.meta_rig"
    bl_label = "Add car meta rig"
    bl_options = {'REGISTER', 'UNDO'}

    def _create_bone(self, selected_objects, rig, name, head):
        target_obj = None
        for o in selected_objects:
            if o.name == name:
                target_obj = o
                break

        b = rig.data.edit_bones.new('DEF-' + name)
        if target_obj is None:
            b.head = head
        else:
            b.head = target_obj.location
            target_obj.parent = rig
            target_obj.parent_bone = b.name
            target_obj.parent_type = 'BONE'
            # BUG : target_obj is not located at the proper place (seems to be at the tail pos)
            target_obj.matrix_local = rig.matrix_world.inverted()

        b.tail = b.head
        b.tail.y += 1.0


    def execute(self, context):
        """Creates the meta rig with basic bones"""

        selected_objects = context.selected_objects
        #create meta rig
        amt = bpy.data.armatures.new('CarMetaRigData')
        obj_data = bpy_extras.object_utils.object_data_add(context, amt, name='CarMetaRig')
        rig = obj_data.object
        rig["Car Rig"] = True

        #create meta rig bones
        bpy.ops.object.mode_set(mode='EDIT')

        self._create_bone(selected_objects, rig, 'Body',      (  0,  0, .8))
        self._create_bone(selected_objects, rig, 'Wheel.Ft.L', ( .9, -2,  1))
        self._create_bone(selected_objects, rig, 'Wheel.Ft.R', (-.9, -2,  1))
        self._create_bone(selected_objects, rig, 'Wheel.Bk.L', ( .9,  2,  1))
        self._create_bone(selected_objects, rig, 'Wheel.Bk.R', (-.9,  2,  1))

        bpy.ops.object.mode_set(mode='OBJECT')
        return{'FINISHED'}


class GenerateRig(bpy.types.Operator):
    # Generates a rig from metarig

    bl_idname = "car.rig_generate"
    bl_label = "Generate Car Rig"
    bl_options = {'UNDO'}

    def execute(self, context):
        Generate()
        return {"FINISHED"}

# Add to menu
def menu_func(self, context):
    self.layout.operator("car.meta_rig",text="Car (Meta-Rig)",icon='AUTO')

def register():
    bpy.types.INFO_MT_armature_add.append(menu_func)
    bpy.utils.register_class(UImetaRigGenerate)
    bpy.utils.register_class(GenerateRig)
    bpy.utils.register_class(BakeWheelRotationOperator)
    bpy.utils.register_class(AddCarMetaRig)
    bpy.utils.register_class(UIPanel)

def unregister():
    bpy.types.INFO_MT_armature_add.remove(menu_func)
    bpy.utils.unregister_class(UImetaRigGenerate)
    bpy.utils.unregister_class(GenerateRig)
    bpy.utils.unregister_class(BakeWheelRotationOperator)
    bpy.utils.unregister_class(AddCarMetaRig)
    bpy.utils.unregister_class(UIPanel)

if __name__ == "__main__":
    register()
