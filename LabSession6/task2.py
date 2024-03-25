#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 20 14:08:56 2023

@title: Lab Session 6 Task 2: Control of Your Robot Gripper
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

# To close your gripper (or open it, depending on the design you have made), 
# you need to set ee_d1 to a low logic state and ee_d2 to a high logic state
with eva.lock():
   eva.control_wait_for_ready()
   eva.gpio_set('ee_d1', False)
   eva.gpio_set('ee_d0', True)
   
# To open (or close) the gripper, you need to do the opposite
with eva.lock():
   eva.control_wait_for_ready()
   eva.gpio_set('ee_d0', False)
   eva.gpio_set('ee_d1', True)
   
# If you need to check the current states of the pins, you can also get the 
# current values of these pins using the gpio_get() method in the API
with eva.lock():
   ee_d0_state = eva.gpio_get('ee_d0', 'output')
   ee_d1_state = eva.gpio_get('ee_d1', 'output')


# If there is an error (robot lights turn red), execute this before anything else:
#with eva.lock():
#  eva.control_reset_errors()
