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

def Generate():
    print("Starting car rig generation...")

    ob = bpy.context.active_object
    ob.show_x_ray = True
    ob.name = "Car Rig"
    amt = ob.data
    amt['turning_wheels'] = True

    bpy.ops.object.mode_set(mode='EDIT')
    #####################################Computing Average Positions#################################
    posx = (ob.data.bones['DEF-Wheel.F.R'].head_local[0] + ob.data.bones['DEF-Wheel.F.L'].head_local[0]) /2
    posy = (ob.data.bones['DEF-Wheel.F.R'].head_local[1] + ob.data.bones['DEF-Wheel.F.L'].head_local[1]) /2
    posz = (ob.data.bones['DEF-Wheel.F.R'].head_local[2] + ob.data.bones['DEF-Wheel.F.L'].head_local[2]) /2

    pos2x = (ob.data.bones['DEF-Wheel.B.R'].head_local[0] + ob.data.bones['DEF-Wheel.B.L'].head_local[0]) /2
    pos2y = (ob.data.bones['DEF-Wheel.B.R'].head_local[1] + ob.data.bones['DEF-Wheel.B.L'].head_local[1]) /2
    pos2z = (ob.data.bones['DEF-Wheel.B.R'].head_local[2] + ob.data.bones['DEF-Wheel.B.L'].head_local[2]) /2

    pos3x = ob.data.bones['DEF-Body'].head_local[0]
    pos3y = ob.data.bones['DEF-Body'].head_local[1]
    pos3z = ob.data.bones['DEF-Body'].head_local[2]


    #####################################Create Bones#################################
    Root = amt.edit_bones.new('Root')
    Root.head = (pos3x,pos3y,0)
    Root.tail = (pos3x,pos3y+3,0)

    wheelEngine = amt.edit_bones.new('MCH-Wheel.engine')
    wheelEngine.head = (posx,posy-1,posz)
    wheelEngine.tail = (posx,posy-.5,posz)
    wheelEngine.parent = Root

    axis = amt.edit_bones.new('MCH-axis')
    axis.head = (posx,posy,posz)
    axis.tail = (pos2x,pos2y,pos2z)

    damperCenter = amt.edit_bones.new('MCH-Damper.center')
    damperCenter.head = ob.data.bones['DEF-Body'].head_local
    damperCenter.tail = (pos3x,pos3y+.5,pos3z)
    damperCenter.parent = Root

    body = amt.edit_bones['DEF-Body']
    body.parent = axis

    FRWheel = amt.edit_bones['DEF-Wheel.F.R']
    FRWheel.parent = damperCenter

    FLWheel = amt.edit_bones['DEF-Wheel.F.L']
    FLWheel.parent = damperCenter

    BRWheel = amt.edit_bones['DEF-Wheel.B.R']
    BRWheel.parent = damperCenter

    BLWheel = amt.edit_bones['DEF-Wheel.B.L']
    BLWheel.parent = damperCenter

    steeringWheel = amt.edit_bones.new('Steering')
    steeringWheel.head = (posx,posy-2,posz)
    steeringWheel.tail = (posx,posy-1.5,posz)
    steeringWheel.parent = Root

    damperFront = amt.edit_bones.new('MCH-Damper.front')
    damperFront.head = ob.data.bones['DEF-Wheel.F.R'].head_local
    damperFront.tail = ob.data.bones['DEF-Wheel.F.L'].head_local

    damperBack = amt.edit_bones.new('MCH-Damper.back')
    damperBack.head = ob.data.bones['DEF-Wheel.B.R'].head_local
    damperBack.tail = ob.data.bones['DEF-Wheel.B.L'].head_local

    damper = amt.edit_bones.new('Damper')
    damper.head = (pos3x,pos3y,pos3z+2)
    damper.tail = (pos3x,pos3y+1,pos3z+2)
    damper.parent = damperCenter

    FRSensor = amt.edit_bones.new('MCH-GroundSensor.F.R')
    FRSensor.head = ob.data.bones['DEF-Wheel.F.R'].head_local
    FRSensor.tail = ob.data.bones['DEF-Wheel.F.R'].head_local
    FRSensor.tail[1] = FRSensor.tail.y+0.3
    FRSensor.parent = damperCenter

    FLSensor = amt.edit_bones.new('MCH-GroundSensor.F.L')
    FLSensor.head = ob.data.bones['DEF-Wheel.F.L'].head_local
    FLSensor.tail = ob.data.bones['DEF-Wheel.F.L'].head_local
    FLSensor.tail[1] = FLSensor.tail.y+0.3
    FLSensor.parent = damperCenter

    BRSensor = amt.edit_bones.new('MCH-GroundSensor.B.R')
    BRSensor.head = ob.data.bones['DEF-Wheel.B.R'].head_local
    BRSensor.tail = ob.data.bones['DEF-Wheel.B.R'].head_local
    BRSensor.tail[1] = BRSensor.tail.y+0.3
    BRSensor.parent = damperCenter

    BLSensor = amt.edit_bones.new('MCH-GroundSensor.B.L')
    BLSensor.head = ob.data.bones['DEF-Wheel.B.L'].head_local
    BLSensor.tail = ob.data.bones['DEF-Wheel.B.L'].head_local
    BLSensor.tail[1] = BLSensor.tail.y+0.3
    BLSensor.parent = damperCenter

    WheelRot = amt.edit_bones.new('Wheel rotation')
    WheelRot.head = ob.data.bones['DEF-Wheel.F.L'].head_local
    WheelRot.tail = ob.data.bones['DEF-Wheel.F.L'].head_local
    WheelRot.tail[1] = FLSensor.tail.y+0.3
    WheelRot.parent = damperCenter

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
    add_wheel_constraints(ob, 'DEF-Wheel.F.L', 'MCH-GroundSensor.F.L')
    add_wheel_constraints(ob, 'DEF-Wheel.F.R', 'MCH-GroundSensor.F.R')
    add_wheel_constraints(ob, 'DEF-Wheel.B.L', 'MCH-GroundSensor.B.L')
    add_wheel_constraints(ob, 'DEF-Wheel.B.R', 'MCH-GroundSensor.B.R')

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
    cns5.subtarget = 'MCH-Damper.front'
    cns5.head_tail = 0.5
    # Tract To constraint axis -> damperFront
    cns6 = axis.constraints.new('TRACK_TO')
    cns6.target = ob
    cns6.subtarget = 'MCH-Damper.back'
    cns6.head_tail = 0.5
    cns6.track_axis = 'TRACK_Y'
    cns6.use_target_z = True
    cns6.owner_space = 'POSE'
    cns6.target_space = 'POSE'
    # Damped Track constraint axis -> damperBack
    cns7 = axis.constraints.new('DAMPED_TRACK')
    cns7.influence = 0.5
    cns7.target = ob
    cns7.subtarget = 'MCH-Damper.front'
    cns7.track_axis = "TRACK_NEGATIVE_X"
    cns7.influence = 0.5

    # Copy Location constraint damperBack -> BRWheel
    damperBack = ob.pose.bones['MCH-Damper.back']
    cns8 = damperBack.constraints.new('COPY_LOCATION')
    cns8.target = ob
    cns8.subtarget = 'MCH-GroundSensor.B.R'
    # Track To constraint damperBack -> BLWheel
    cns9 = damperBack.constraints.new('TRACK_TO')
    cns9.target = ob
    cns9.subtarget = 'MCH-GroundSensor.B.L'

    # Copy Location constraint damperFront -> FRWheel
    damperFront = ob.pose.bones['MCH-Damper.front']
    cns10 = damperFront.constraints.new('COPY_LOCATION')
    cns10.target = ob
    cns10.subtarget = 'MCH-GroundSensor.F.R'
    # Track To constraint damperFront -> FLWheel
    cns11 = damperFront.constraints.new('TRACK_TO')
    cns11.target = ob
    cns11.subtarget = 'MCH-GroundSensor.F.L'

    # Copy Location constraint Sensors ->
    for sensor_name in ('MCH-GroundSensor.F.L', 'MCH-GroundSensor.F.R', 'MCH-GroundSensor.B.L', 'MCH-GroundSensor.B.R'):
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
    cns.subtarget = 'MCH-GroundSensor.F.L'
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

    if wheel_name in ('DEF-Wheel.F.L', 'DEF-Wheel.F.R'):
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

class AddCarMetaRig(bpy.types.Operator):
    """Operator to create Car Meta Rig"""

    bl_idname = "car.meta_rig"
    bl_label = "Add car meta rig"
    bl_options = {'REGISTER', 'UNDO'}

    def _create_bone(self, armature, name, head, tail):
        b = armature.edit_bones.new(name)
        b.head = head
        b.tail = tail

    def execute(self, context):
        """Creates the meta rig with basic bones"""

        #create meta rig
        amt = bpy.data.armatures.new('CarMetaRigData')
        obj = bpy_extras.object_utils.object_data_add(context, amt, name='CarMetaRig')
        rig = obj.object
        rig["Car Rig"] = True

        #create meta rig bones
        bpy.ops.object.mode_set(mode='EDIT')

        self._create_bone(amt, 'DEF-Body',      (  0,  0, .8), (  0,    2, .8))
        self._create_bone(amt, 'DEF-Wheel.F.L', ( .9, -2,  1), ( .9, -1.5,  1))
        self._create_bone(amt, 'DEF-Wheel.F.R', (-.9, -2,  1), (-.9, -1.5,  1))
        self._create_bone(amt, 'DEF-Wheel.B.L', ( .9,  2,  1), ( .9,  2.5,  1))
        self._create_bone(amt, 'DEF-Wheel.B.R', (-.9,  2,  1), (-.9,  2.5,  1))

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
    bpy.types.INFO_MT_armature_add.prepend(menu_func)
    bpy.utils.register_class(UImetaRigGenerate)
    bpy.utils.register_class(GenerateRig)
    bpy.utils.register_class(AddCarMetaRig)
    bpy.utils.register_class(UIPanel)

def unregister():
    bpy.types.INFO_MT_armature_add.remove(menu_func)
    bpy.utils.unregister_class(UImetaRigGenerate)
    bpy.utils.unregister_class(GenerateRig)
    bpy.utils.unregister_class(AddCarMetaRig)
    bpy.utils.unregister_class(UIPanel)

if __name__ == "__main__":
    register()
