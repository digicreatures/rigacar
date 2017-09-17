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

ANIM_BONE_LAYER=1
DEF_BONE_LAYER=30
MECA_BONE_LAYER=31

def Generate():
    print("Starting car rig generation...")

    ob = bpy.context.active_object
    ob.show_x_ray = True
    ob.name = "Car Rig"
    amt = ob.data
    amt['turning_wheels'] = True

    bpy.ops.object.mode_set(mode='EDIT')
    #####################################Computing Average Positions#################################
    posx = (ob.data.bones['FRWheel'].head_local[0] + ob.data.bones['FLWheel'].head_local[0]) /2
    posy = (ob.data.bones['FRWheel'].head_local[1] + ob.data.bones['FLWheel'].head_local[1]) /2
    posz = (ob.data.bones['FRWheel'].head_local[2] + ob.data.bones['FLWheel'].head_local[2]) /2

    pos2x = (ob.data.bones['BRWheel'].head_local[0] + ob.data.bones['BLWheel'].head_local[0]) /2
    pos2y = (ob.data.bones['BRWheel'].head_local[1] + ob.data.bones['BLWheel'].head_local[1]) /2
    pos2z = (ob.data.bones['BRWheel'].head_local[2] + ob.data.bones['BLWheel'].head_local[2]) /2

    pos3x = ob.data.bones['Body'].head_local[0]
    pos3y = ob.data.bones['Body'].head_local[1]
    pos3z = ob.data.bones['Body'].head_local[2]


    #####################################Create Bones#################################
    Root = amt.edit_bones.new("Root")
    Root.head = (pos3x,pos3y,0)
    Root.tail = (pos3x,pos3y,pos3z+5)

    wheelEngine = amt.edit_bones.new("wheelEngine")
    wheelEngine.head = (posx,posy-1,posz)
    wheelEngine.tail = (posx,posy-3,posz)
    wheelEngine.parent = Root

    wheelEngine.layers[30] = True
    wheelEngine.layers[0] = False

    axis = amt.edit_bones.new("axis")
    axis.head = (pos2x,pos2y,pos2z)
    axis.tail = (posx,posy,posz)
    axis.roll = math.pi

    axis.layers[30] = True
    axis.layers[0] = False

    damperCenter = amt.edit_bones.new("damperCenter")
    damperCenter.head = ob.data.bones['Body'].head_local
    damperCenter.tail = (pos3x,pos3y-1,pos3z)
    damperCenter.layers[30] = True
    damperCenter.layers[0] = False
    damperCenter.parent = Root

    body = amt.edit_bones["Body"]
    body.parent = axis
    body.layers[31] = True
    body.layers[0] = False

    FRWheel = amt.edit_bones["FRWheel"]
    FRWheel.layers[29] = True
    FRWheel.layers[0] = False
    FRWheel.parent = damperCenter

    FLWheel = amt.edit_bones["FLWheel"]
    FLWheel.layers[29] = True
    FLWheel.layers[0] = False
    FLWheel.parent = damperCenter

    BRWheel = amt.edit_bones["BRWheel"]
    BRWheel.layers[29] = True
    BRWheel.layers[0] = False
    BRWheel.parent = damperCenter

    BLWheel = amt.edit_bones["BLWheel"]
    BLWheel.layers[29] = True
    BLWheel.layers[0] = False
    BLWheel.parent = damperCenter

    wheelFront = amt.edit_bones.new("wheelFront")
    wheelFront.head = (posx,posy,posz)
    wheelFront.tail = (posx,posy-0.8,posz)
    wheelFront.parent = damperCenter
    wheelFront.layers[31] = True
    wheelFront.layers[0] = False

    steeringWheel = amt.edit_bones.new("steeringWheel")
    steeringWheel.head = (posx,posy-2,posz)
    steeringWheel.tail = (posx,posy-2.5,posz)
    steeringWheel.parent = Root

    damperFront = amt.edit_bones.new("damperFront")
    damperFront.head = ob.data.bones['FRWheel'].head_local
    damperFront.tail = ob.data.bones['FLWheel'].head_local
    damperFront.layers[30] = True
    damperFront.layers[0] = False

    damperBack = amt.edit_bones.new("damperBack")
    damperBack.head = ob.data.bones['BRWheel'].head_local
    damperBack.tail = ob.data.bones['BLWheel'].head_local
    damperBack.layers[30] = True
    damperBack.layers[0] = False

    damper = amt.edit_bones.new("damper")
    damper.head = (pos3x,pos3y,pos3z+2)
    damper.tail = (pos3x,pos3y-1,pos3z+2)
    damper.parent = damperCenter

    FRSensor = amt.edit_bones.new("FRSensor")
    FRSensor.head = ob.data.bones['FRWheel'].head_local
    FRSensor.tail = ob.data.bones['FRWheel'].head_local
    FRSensor.tail[2] = FRSensor.tail.z+0.3
    FRSensor.parent = damperCenter

    FLSensor = amt.edit_bones.new("FLSensor")
    FLSensor.head = ob.data.bones['FLWheel'].head_local
    FLSensor.tail = ob.data.bones['FLWheel'].head_local
    FLSensor.tail[2] = FLSensor.tail.z+0.3
    FLSensor.parent = damperCenter

    BRSensor = amt.edit_bones.new("BRSensor")
    BRSensor.head = ob.data.bones['BRWheel'].head_local
    BRSensor.tail = ob.data.bones['BRWheel'].head_local
    BRSensor.tail[2] = BRSensor.tail.z+0.3
    BRSensor.parent = damperCenter

    BLSensor = amt.edit_bones.new("BLSensor")
    BLSensor.head = ob.data.bones['BLWheel'].head_local
    BLSensor.tail = ob.data.bones['BLWheel'].head_local
    BLSensor.tail[2] = BLSensor.tail.z+0.3
    BLSensor.parent = damperCenter

    WheelRot = amt.edit_bones.new("WheelRot")
    WheelRot.head = ob.data.bones['FLWheel'].head_local
    WheelRot.tail = ob.data.bones['FLWheel'].head_local
    WheelRot.tail[1] = FLSensor.tail.y+0.3
    WheelRot.parent = damperCenter
    WheelRot.roll = math.pi


    #####################################Pose Constraints#################################
    bpy.ops.object.mode_set(mode='POSE')

    # Lock transformation on steering wheel
    steeringWheel = ob.pose.bones['steeringWheel']
    steeringWheel.lock_location = (False, True, True)
    steeringWheel.lock_rotation = (True, True, True)
    steeringWheel.lock_rotation_w = True


    # Locked Track constraint wheelFront -> steeringWheel
    wheelFront = ob.pose.bones['wheelFront']
    cns1 = wheelFront.constraints.new('LOCKED_TRACK')
    cns1.target = ob
    cns1.subtarget = 'steeringWheel'

    # Constaints on wheels
    add_wheel_constraints(ob, 'FLWheel', 'FLSensor')
    add_wheel_constraints(ob, 'FRWheel', 'FRSensor')
    add_wheel_constraints(ob, 'BLWheel', 'BLSensor')
    add_wheel_constraints(ob, 'BRWheel', 'BRSensor')

    # Transformation constraint Body -> damper
    damperCenter = ob.pose.bones['Body']
    cns4 = damperCenter.constraints.new('TRANSFORM')
    cns4.target = ob
    cns4.subtarget = 'damper'
    cns4.from_min_x = -0.3
    cns4.from_max_x = 0.3
    cns4.from_min_y = -0.3
    cns4.from_max_y = 0.3
    cns4.map_to_x_from = "Y"
    cns4.map_to_z_from = "X"
    cns4.map_to = "ROTATION"
    cns4.to_min_x_rot = math.radians(-6)
    cns4.to_max_x_rot = math.radians(6)
    cns4.to_min_z_rot = math.radians(-7)
    cns4.to_max_z_rot = math.radians(7)
    cns4.owner_space = 'LOCAL'
    cns4.target_space = 'LOCAL'
     # Transformation constraint Body -> damper
    damperCenter = ob.pose.bones['Body']
    cns4 = damperCenter.constraints.new('TRANSFORM')
    cns4.target = ob
    cns4.subtarget = 'damper'
    cns4.from_min_z = -0.1
    cns4.from_max_z = 0.1
    cns4.map_to_y_from = "Z"
    cns4.to_min_y = -0.1
    cns4.to_max_y = 0.1
    cns4.owner_space = 'LOCAL'
    cns4.target_space = 'LOCAL'


    # Copy Location constraint axis -> damperBack
    axis = ob.pose.bones['axis']
    cns5 = axis.constraints.new('COPY_LOCATION')
    cns5.target = ob
    cns5.subtarget = 'damperBack'
    cns5.head_tail = 0.5
    # Tract To constraint axis -> damperFront
    cns6 = axis.constraints.new('TRACK_TO')
    cns6.target = ob
    cns6.subtarget = 'damperFront'
    cns6.head_tail = 0.5
    cns6.use_target_z = True
    cns6.owner_space = 'POSE'
    cns6.target_space = 'POSE'
    # Damped Track constraint axis -> damperBack
    cns7 = axis.constraints.new('DAMPED_TRACK')
    cns7.influence = 0.5
    cns7.target = ob
    cns7.subtarget = 'damperBack'
    cns7.track_axis = "TRACK_X"
    cns7.influence = 0.5

    # Copy Location constraint damperBack -> BRWheel
    damperBack = ob.pose.bones['damperBack']
    cns8 = damperBack.constraints.new('COPY_LOCATION')
    cns8.target = ob
    cns8.subtarget = 'BRSensor'
    # Track To constraint damperBack -> BLWheel
    cns9 = damperBack.constraints.new('TRACK_TO')
    cns9.target = ob
    cns9.subtarget = 'BLSensor'

    # Copy Location constraint damperFront -> FRWheel
    damperFront = ob.pose.bones['damperFront']
    cns10 = damperFront.constraints.new('COPY_LOCATION')
    cns10.target = ob
    cns10.subtarget = 'FRSensor'
    # Track To constraint damperFront -> FLWheel
    cns11 = damperFront.constraints.new('TRACK_TO')
    cns11.target = ob
    cns11.subtarget = 'FLSensor'

    # Copy Location constraint Sensors ->
    for sensor_name in ('FLSensor', 'FRSensor', 'BLSensor', 'BRSensor'):
        Sensor = ob.pose.bones[sensor_name]
        Sensor.lock_location = (True,False,True)
        cns = Sensor.constraints.new('SHRINKWRAP')
        cns.distance = Sensor.head.z

    # Copy Location constraint WheelRot -> FLSensor
    WheelRot = ob.pose.bones['WheelRot']
    WheelRot.rotation_mode = "XYZ"
    WheelRot.lock_location = (True,True,True)
    WheelRot.lock_rotation = (False,True,True)

    cns = WheelRot.constraints.new('COPY_LOCATION')
    cns.target = ob
    cns.subtarget = 'FLSensor'
    cns.use_x = False
    cns.use_y = False


    #############################################Add Driver#########################

    # fcurve = FLWheel.driver_add('rotation_euler', 0)
    # drv = fcurve.driver
    # drv.type = 'AVERAGE'
    # var = drv.variables.new()
    # var.name = 'x'
    # var.type = 'TRANSFORMS'
    #
    # targ = var.targets[0]
    # targ.id = empty
    # targ.transform_type = 'LOC_Y'
    # targ.transform_space = "LOCAL_SPACE"
    #
    # fmod = fcurve.modifiers[0]
    # fmod.mode = 'POLYNOMIAL'
    # fmod.poly_order = 1
    # if FLWheel.head.z <= 0:
    #     fmod.coefficients = (0, 1)
    # else:
    #     fmod.coefficients = (0, 1/FLWheel.head.z)

    # bpy.ops.object.select_all(action="TOGGLE")
    ob.select = True
    bpy.context.scene.objects.active = ob

    print("Generate Finished")


def add_wheel_constraints(ob, wheel_name, sensor_name):
    # Copy Location constraint XXWheel -> XXSensor
    wheel = ob.pose.bones[wheel_name]
    wheel.rotation_mode = "XYZ"
    cns = wheel.constraints.new('COPY_LOCATION')
    cns.target = ob
    cns.subtarget = sensor_name
    cns.use_x = False
    cns.use_y = False

    # # Damped Track constraint XXWheel -> damper
    # cns = wheel.constraints.new('DAMPED_TRACK')
    # cns.track_axis = 'TRACK_X' if wheel_name in ('FLWheel', 'BLWheel') else 'TRACK_NEGATIVE_X'
    # cns.target = ob
    # cns.subtarget = 'damperFront' if wheel_name in ('FLWheel', 'FRWheel') else 'damperBack'
    #
    # # Copy Location constraint XXWheel -> damper
    # cns = wheel.constraints.new('COPY_LOCATION')
    # cns.target = ob
    # cns.subtarget = 'damperFront' if wheel_name in ('FLWheel', 'FRWheel') else 'damperBack'
    # cns.head_tail = 1
    # cns.use_y = False
    # cns.use_z = False

    if wheel_name in ('FLWheel', 'FRWheel'):
        # Copy Rotation constraint XXWheel -> wheelFront
        cns = wheel.constraints.new('COPY_ROTATION')
        cns.target = ob
        cns.subtarget = 'wheelFront'
        cns.use_x = False
        cns.use_y = False

    # Copy Rotation constraint XXWheel -> WheelRot
    cns = wheel.constraints.new('COPY_ROTATION')
    cns.target = ob
    cns.subtarget = 'WheelRot'
    cns.use_y = False
    cns.use_z = False

    # Transformation constraint XXWheel -> wheelEngine
    cns = wheel.constraints.new('TRANSFORM')
    cns.target = ob
    cns.subtarget = 'wheelEngine'
    cns.use_motion_extrapolate = True
    cns.from_min_y = wheel.head.z
    cns.from_max_y = 0
    cns.map_to_x_from = "Y"
    cns.map_to = "ROTATION"
    cns.to_min_x_rot = 0
    cns.to_max_x_rot = math.pi
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

    def execute(self, context):
        """Creates the meta rig with basic bones"""

        #create meta rig
        amt = bpy.data.armatures.new('CarMetaRigData')
        obj = bpy_extras.object_utils.object_data_add(context, amt, name='CarMetaRig')
        rig = obj.object
        rig["Car Rig"] = True

        #create meta rig bones
        bpy.ops.object.mode_set(mode='EDIT')
        body = amt.edit_bones.new('Body')
        body.head = (0,0,0.8)
        body.tail = (0,0,1.8)

        FRW = amt.edit_bones.new('FLWheel')
        FRW.head = (0.9,-2,1)
        FRW.tail = (0.9,-2.5,1)

        FLW = amt.edit_bones.new('FRWheel')
        FLW.head = (-0.9,-2,1)
        FLW.tail = (-0.9,-2.5,1)

        BRW = amt.edit_bones.new('BLWheel')
        BRW.head = (0.9,2,1)
        BRW.tail = (0.9,1.5,1)

        BLW = amt.edit_bones.new('BRWheel')
        BLW.head = (-0.9,2,1)
        BLW.tail = (-0.9,1.5,1)
        return{'FINISHED'}


class GenerateRig(bpy.types.Operator):
    # Generates a rig from metarig

    bl_idname = "car.rig_generate"
    bl_label = "Generate Car Rig"

    # I must study this
    #bl_register = True
    #bl_undo = True
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
