# -*- coding: utf-8 -*-
"""
Created on Thu Apr 20 11:10:21 2023

@title: Lab Session 6 Task 1: Kinematic Control of Robot Arm
@author: Alejandro Pascual San Roman (bdh532)
@organisation: School of Physics, Engineering and Technology. University of York

"""

from evasdk import Eva
from json import dumps

# Details to connect to the robot arm, more info here: 
# https://wiki.york.ac.uk/display/TSS/Network+Details+and+Credentials+for+the+EVA+Arms+and+Network+Cameras 
arm_hostname = "evatrendylimashifterpt410"
arm_ip = "144.32.152.105"
token = "1462980d67d58cb7eaabe8790d866609eb97fd2c"

# Create an “eva” object with these parameters to connect to the arm itself on the network
eva = Eva(arm_ip, token)

# Send the EVA arm to its home position
with eva.lock():
  eva.control_wait_for_ready()
  eva.control_go_to([0, 1, -2.5, 0, -1.6, 0])

# The EVA arm also has the ability to follow “toolpaths” that are pre-programmed.
toolpaths = eva.toolpaths_list()
outToolpaths = []

# Write toolpaths from the arm to custom array
for toolpathItem in toolpaths:
  toolpath = eva.toolpaths_retrieve(toolpathItem['id'])
  outToolpaths.append(toolpath)

# Print the toolpaths stored in the EVA arm
print(dumps(outToolpaths))

# To create a toolpath, you need to create a Python dictionary with the 
# parameters for the toolpath included
toolpath = {
  "metadata": {
      "version": 2,
      "payload": 0,
      "default_max_speed": 1.05,
      "next_label_id": 5,
      "analog_modes": {"i0": "voltage", "i1": "voltage", "o0": "voltage", "o1": "voltage"},
  },
  "waypoints": [
      {"joints": [-0.68147224, 0.3648368, -1.0703622, 9.354615e-05, -2.4358354, -0.6813218], "label_id": 3},
      {"joints": [-0.6350288, 0.25192022, -1.0664424, 0.030407501, -2.2955494, -0.615318], "label_id": 2},
      {"joints": [-0.13414459, 0.5361486, -1.280493, -6.992453e-08, -2.3972468, -0.13414553], "label_id": 1},
      {"joints": [-0.4103904, 0.33332264, -1.5417944, -5.380291e-06, -1.9328799, -0.41031334], "label_id": 4},
  ],
  "timeline": [
      {"type": "home", "waypoint_id": 2},
      {"type": "trajectory", "trajectory": "joint_space", "waypoint_id": 1},
      {"type": "trajectory", "trajectory": "joint_space", "waypoint_id": 0},
      {"type": "trajectory", "trajectory": "joint_space", "waypoint_id": 2},
  ],
}
  
# Defining and executing the example toolpath provided
with eva.lock():
  eva.control_wait_for_ready()
  eva.toolpaths_use(toolpath)
  eva.control_home()
  eva.control_run(loop=1)

# If there is an error (robot lights turn red), execute this before anything else:
#with eva.lock():
#  eva.control_reset_errors()

