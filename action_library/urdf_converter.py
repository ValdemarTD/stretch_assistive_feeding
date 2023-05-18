import urdfpy
import numpy as np

import ikpy.urdf.utils
import ikpy.chain
import pathlib
from IPython import display
import ipywidgets as widgets
import ikpy.utils.plot as plot_utils
import matplotlib.pyplot as plt

import time

class URDFConverter():


    def __init__(self, path, links_to_remove, joints_to_remove):
        self.path = ""
        self.links_to_remove = links_to_remove
        self.joints_to_remove = joints_to_remove
        self.urdf_path = '~/catkin_ws/src/stretch_ros/stretch_description/urdf/exported_urdf/stretch.urdf'

    def convertUrdf(self):
        """
        Converts URDF into ikpy ready state 
        """
 
        self.current_urdf()
        self.simplify_urdf()
        if self.modified_urdf is not None:
            self.save(self.modified_urdf)
        
        #Double check current tree
        tree = ikpy.urdf.utils.get_urdf_tree(self.path, "base_link")[0]
        display.display_png(tree)
        return True 

    def get_path(self):
        """
        Displays and grabs currently URDF paths
        """
        urdf_path = str((pathlib.Path(self.path).absolute()))
        tree = ikpy.urdf.utils.get_urdf_tree(urdf_path, "base_link")[0]
        display.display_svg(tree)
        return urdf_path

    def current_urdf(self):
        """"
        Loads the currently stored URDF file
        """
        loaded_urdf = urdfpy.URDF.load(self.urdf_path)
        print(f"name: {loaded_urdf.name}")
        print(f"num links: {len(loaded_urdf.links)}")
        print(f"num joints: {len(loaded_urdf.joints)}")
        self.original_urdf = loaded_urdf

    def simplify_urdf(self):
        """
        Returns a simplified URDF chain
        """
        modified_urdf = self.original_urdf.copy()
        names_of_links_to_remove = self.links_to_remove 
        links_to_remove = [l for l in modified_urdf._links if l.name in names_of_links_to_remove]
        for lr in links_to_remove:
            modified_urdf._links.remove(lr)
        names_of_joints_to_remove = self.joints_to_remove
        joints_to_remove = [l for l in modified_urdf._joints if l.name in names_of_joints_to_remove]
        for jr in joints_to_remove:
            modified_urdf._joints.remove(jr)
        print(f"name: {modified_urdf.name}")
        print(f"num links: {len(modified_urdf.links)}")
        print(f"num joints: {len(modified_urdf.joints)}")
        self.modified_urdf = modified_urdf


    def save(modified_urdf):
        """
        Saves
        """
        iktuturdf_path = "/tmp/iktutorial/stretch.urdf"
        modified_urdf.save(iktuturdf_path)
        return iktuturdf_path