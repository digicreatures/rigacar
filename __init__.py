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
from . import bake_operators
from . import car_rig


bl_info = {
    "name": "Generate Car Rig",
    "author": "David Gayerie",
    "version": (0, 9),
    "blender": (2, 7, 9),
    "location": "View3D > Add > Armature",
    "description": "Adds a deformation rig for 4 wheels vehicules, generates animation rig and bake wheels animation.",
    "wiki_url": "",
    "tracker_url": "https://github.com/digicreatures/rigacar/issues",
    "category": "Rigging"}

#import importlib
#importlib.reload(rigacar.bake_operators)
#importlib.reload(rigacar.car_rig)

class BaseCarRigPanel:
    @classmethod
    def poll(cls, context):
        return 'Car Rig' in context.object.data if context.object is not None and context.object.data is not None else False

    def draw(self, context):
        if context.object.data['Car Rig']:
            self.layout.prop(context.object, '["wheels_on_y_axis"]', text="Wheels on Y axis")
            self.layout.prop(context.object, '["suspension_factor"]', text="Suspension fact.")
            self.layout.prop(context.object, '["suspension_rolling_factor"]', text="Suspension rolling fact.")
            self.layout.operator(bake_operators.BakeWheelRotationOperator.bl_idname, 'Bake wheels rotation', 'Automatically generates wheels animation based on Root bone animation.')
            self.layout.operator(bake_operators.BakeSteeringOperator.bl_idname, 'Bake steering', 'Automatically steering animation based on Root bone animation.')
        elif context.object.mode in {"POSE", "OBJECT"}:
            self.layout.operator(car_rig.GenerateCarAnimationRigOperator.bl_idname, text='Generate')


class UICarRigPropertiesPanel(bpy.types.Panel, BaseCarRigPanel):
    bl_label = "Car Rig"
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "data"


class UICarRigView3DPanel(bpy.types.Panel, BaseCarRigPanel):
    bl_label = "Car Rig"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"


def menu_entries(menu, context):
    menu.layout.operator(car_rig.AddCarDeformationRigOperator.bl_idname, text="Car (deformation rig)", icon='AUTO')


def register():
    bpy.types.INFO_MT_armature_add.append(menu_entries)
    bpy.utils.register_class(UICarRigPropertiesPanel)
    bpy.utils.register_class(UICarRigView3DPanel)
    car_rig.register()
    bake_operators.register()


def unregister():
    bpy.types.INFO_MT_armature_add.remove(menu_entries)
    bpy.utils.unregister_class(UICarRigPropertiesPanel)
    bpy.utils.unregister_class(UICarRigView3DPanel)
    car_rig.unregister()
    bake_operators.unregister()


if __name__ == "__main__":
    register()
