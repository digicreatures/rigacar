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
from bpy.props import *

def Generate(origin):
    print("Starting car rig generation...")

    ob = bpy.context.active_object
    ob.show_x_ray = True
    ob.name = "Car Rig"
    amt = ob.data


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

    FLWheel.parent = WheelRot

    #####################################Pose Constraints#################################
    bpy.ops.object.mode_set(mode='POSE')

    # Locked Track constraint wheelFront -> steeringWheel
    wheelFront = ob.pose.bones['wheelFront']
    cns1 = wheelFront.constraints.new('LOCKED_TRACK')
    cns1.target = ob
    cns1.subtarget = 'steeringWheel'


    # Copy Location constraint FLWheel -> FLSensor
    FLWheel = ob.pose.bones['FLWheel']
    cns3b = FLWheel.constraints.new('COPY_LOCATION')
    cns3b.target = ob
    cns3b.subtarget = 'FLSensor'
    cns3b.use_x = False
    cns3b.use_y = False
    # Damped Track constraint FLWheel -> damperFront
    cns3 = FLWheel.constraints.new('DAMPED_TRACK')
    cns3.track_axis = "TRACK_X"
    cns3.target = ob
    cns3.subtarget = 'damperFront'
    # Copy Location constraint FLWheel -> damperFront
    cns3b = FLWheel.constraints.new('COPY_LOCATION')
    cns3b.target = ob
    cns3b.subtarget = 'damperFront'
    cns3b.head_tail = 1
    cns3b.use_y = False
    cns3b.use_z = False
    # Copy Rotation constraint FLWheel -> wheelFront
    cns2 = FLWheel.constraints.new('COPY_ROTATION')
    cns2.target = ob
    cns2.subtarget = 'wheelFront'
    cns2.use_x = False
    cns2.use_y = False



    # Copy Location constraint FRWheel -> FRSensor
    FRWheel = ob.pose.bones['FRWheel']
    cns3b = FRWheel.constraints.new('COPY_LOCATION')
    cns3b.target = ob
    cns3b.subtarget = 'FRSensor'
    cns3b.use_x = False
    cns3b.use_y = False
    # Damped Track constraint FRWheel -> damperFront
    cns3 = FRWheel.constraints.new('DAMPED_TRACK')
    cns3.track_axis = "TRACK_NEGATIVE_X"
    cns3.target = ob
    cns3.subtarget = 'damperFront'
    cns3.head_tail = 1
    # Copy Rotation constraint FRWheel -> wheelFront
    cns3a = FRWheel.constraints.new('COPY_ROTATION')
    cns3a.target = ob
    cns3a.subtarget = 'wheelFront'
    cns3a.use_x = False
    cns3a.use_y = False
    # Copy Rotation constraint RRWheel -> BLWHeel
    cns3c = FRWheel.constraints.new('COPY_ROTATION')
    cns3c.target = ob
    cns3c.subtarget = 'FLWheel'
    cns3c.use_y = False
    cns3c.use_z = False

    # Copy Location constraint BRWheel -> BRSensor
    BRWheel = ob.pose.bones['BRWheel']
    cns3b = BRWheel.constraints.new('COPY_LOCATION')
    cns3b.target = ob
    cns3b.subtarget = 'BRSensor'
    cns3b.use_x = False
    cns3b.use_y = False
    # Damped Track constraint BRWheel -> damperBack
    cns3 = BRWheel.constraints.new('DAMPED_TRACK')
    cns3.track_axis = "TRACK_NEGATIVE_X"
    cns3.head_tail = 1
    cns3.target = ob
    cns3.subtarget = 'damperBack'
    # Copy Rotation constraint BRWheel -> BLWHeel
    cns3c = BRWheel.constraints.new('COPY_ROTATION')
    cns3c.target = ob
    cns3c.subtarget = 'FLWheel'
    cns3c.use_y = False
    cns3c.use_z = False


    # Copy Location constraint BLWheel -> BLSensor
    BLWheel = ob.pose.bones['BLWheel']
    cns3b = BLWheel.constraints.new('COPY_LOCATION')
    cns3b.target = ob
    cns3b.subtarget = 'BLSensor'
    cns3b.use_x = False
    cns3b.use_y = False
    # Damped Track constraint BLWheel -> damperBack
    cns3 = BLWheel.constraints.new('DAMPED_TRACK')
    cns3.track_axis = "TRACK_X"
    cns3.target = ob
    cns3.subtarget = 'damperBack'
    # Copy Location constraint BLWheel -> damperBack
    cns3b = BLWheel.constraints.new('COPY_LOCATION')
    cns3b.target = ob
    cns3b.subtarget = 'damperBack'
    cns3b.head_tail = 1
    cns3b.use_y = False
    cns3b.use_z = False
    # Copy Rotation constraint BLWheel -> BLWHeel
    cns3c = BLWheel.constraints.new('COPY_ROTATION')
    cns3c.target = ob
    cns3c.subtarget = 'FLWheel'
    cns3c.use_y = False
    cns3c.use_z = False


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
    cns4.to_min_x = -6
    cns4.to_max_x = 6
    cns4.to_min_z = -7
    cns4.to_max_z = 7
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
    cns8.subtarget = 'BRWheel'
    # Tract To constraint damperBack -> BLWheel
    cns9 = damperBack.constraints.new('TRACK_TO')
    cns9.target = ob
    cns9.subtarget = 'BLWheel'

    # Copy Location constraint damperFront -> FRWheel
    damperFront = ob.pose.bones['damperFront']
    cns10 = damperFront.constraints.new('COPY_LOCATION')
    cns10.target = ob
    cns10.subtarget = 'FRWheel'
    # Tract To constraint damperFront -> FLWheel
    cns11 = damperFront.constraints.new('TRACK_TO')
    cns11.target = ob
    cns11.subtarget = 'FLWheel'

    # Copy Location constraint FLSensor ->
    FLSensor = ob.pose.bones['FLSensor']
    FLSensor.lock_location = (True,False,True)
    cns = FLSensor.constraints.new('SHRINKWRAP')
    cns.distance = FLSensor.head.z

    # Copy Location constraint FRSensor ->
    FRSensor = ob.pose.bones['FRSensor']
    FRSensor.lock_location = (True,False,True)
    cns = FRSensor.constraints.new('SHRINKWRAP')
    cns.distance = FRSensor.head.z

    # Copy Location constraint BLSensor ->
    BLSensor = ob.pose.bones['BLSensor']
    BLSensor.lock_location = (True,False,True)
    cns = BLSensor.constraints.new('SHRINKWRAP')
    cns.distance = BLSensor.head.z

    # Copy Location constraint BRSensor ->
    BRSensor = ob.pose.bones['BRSensor']
    BRSensor.lock_location = (True,False,True)
    cns = BRSensor.constraints.new('SHRINKWRAP')
    cns.distance = BRSensor.head.z

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
    # add empty
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.add(type="EMPTY", location=(0,0,0))
    empty = bpy.context.active_object
    empty.name = "carDriver"
    empty.empty_draw_size = 2
    empty.show_x_ray = True
    empty.empty_draw_type = "ARROWS"

    FLWheel.rotation_mode = "XYZ"

    fcurve = FLWheel.driver_add('rotation_euler', 0)
    drv = fcurve.driver
    drv.type = 'AVERAGE'
    var = drv.variables.new()
    var.name = 'x'
    var.type = 'TRANSFORMS'

    targ = var.targets[0]
    targ.id = empty
    targ.transform_type = 'LOC_Y'
    targ.transform_space = "LOCAL_SPACE"

    fmod = fcurve.modifiers[0]
    fmod.mode = 'POLYNOMIAL'
    fmod.poly_order = 1
    if FLWheel.head.z <= 0:
        fmod.coefficients = (0, 1)
    else:
        fmod.coefficients = (0, 1/(FLWheel.head.z * math.pi))

    # parent body bone
    ob.parent = empty

    bpy.ops.object.select_all(action="TOGGLE")
    ob.select = True

    print("Generate Finished")



def CreateCarMetaRig(origin):       #create Car meta rig
    #create meta rig
    amt = bpy.data.armatures.new('CarMetaRigData')
    rig = bpy.data.objects.new('CarMetaRig', amt)
    rig["metaCarRig"] = True
    rig.location = origin
    rig.show_x_ray = True
    #link to scene
    scn = bpy.context.scene
    scn.objects.link(rig)
    scn.objects.active = rig
    scn.update()

    #create meta rig bones
    bpy.ops.object.mode_set(mode='EDIT')
    body = amt.edit_bones.new('Body')
    body.head = (0,0,0)
    body.tail = (0,0,0.8)

    FRW = amt.edit_bones.new('FLWheel')
    FRW.head = (0.9,-2,0)
    FRW.tail = (0.9,-2.5,0)

    FLW = amt.edit_bones.new('FRWheel')
    FLW.head = (-0.9,-2,0)
    FLW.tail = (-0.9,-2.5,0)

    BRW = amt.edit_bones.new('BLWheel')
    BRW.head = (0.9,2,0)
    BRW.tail = (0.9,1.5,0)

    BLW = amt.edit_bones.new('BRWheel')
    BLW.head = (-0.9,2,0)
    BLW.tail = (-0.9,1.5,0)

    #switch to object mode
    bpy.ops.object.mode_set(mode='OBJECT')


#generate button
class UImetaRigGenerate(bpy.types.Panel):
    bl_label = "Car Rig"
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "data"

    @classmethod
    def poll(cls, context):
        return context.object is not None and "metaCarRig" in context.object

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
        return context.object is not None and "metaCarRig" in context.object

    def draw(self, context):
        if context.object.animation_data is not None:
            if context.object.animation_data.drivers is not None:
                driver = context.object.animation_data.drivers.find('pose.bones["FLWheel"].rotation_euler')
                if driver is not None:
                    self.layout.prop(driver.modifiers[0], 'coefficients', text = "size of wheel")


### Add menu create car meta rig
class AddCarMetaRig(bpy.types.Operator):
    bl_idname = "car.meta_rig"
    bl_label = "Add car meta rig"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        CreateCarMetaRig((0,0,0))
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
        Generate((0,0,0))
        return {"FINISHED"}

# Add to menu
def menu_func(self, context):
    self.layout.operator("car.meta_rig",text="Car(Meta-Rig)",icon='MESH_CUBE')

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
