#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 20 14:47:52 2023

@title: Lab Session 7 Task 1: Structure Your Program for Pick-and-Place Arm Control
@author: Alejandro Pascual San Roman (bdh532)
@organisation: School of Physics, Engineering and Technology. University of York

"""

# =============================================================================
#
# Your assessment is focused on using the gripper that you have designed to 
# perform what is known as “pick-and-place” automation: identifying and picking 
# up an object from a known or approximate location, moving it to a new location, 
# and dropping it there.  Pick-and-place systems are used in industry 
# for automated sorting, for material movement, and for packaging processes a
# mong many other applications.  It is one of the simplest and most fundamental 
# operations for robots in industry.
#
# To implement pick-and-place operations, first create a new Python program file 
# ending in .py if you have not already done so in the previous lab.  
# You will need to put in code based on the labs you have completed previously 
# this term.  Make sure to comment your program well for your own reference as 
# well as to make your code more readable for the assessment.  Your program can 
# be designed in any way you wish but the following structure is suggested to 
# get you started:
#
# =============================================================================

# 1. import all the modules/libraries for Python that are needed for your code, including NumPy, SymPy, MatPlotLib 
#    (if you want to use plotting of your kinematics as before), the evasdk module, and the aravis module.
import numpy
import matplotlib.pyplot
from kinematics import Kinematics

from mpl_toolkits.mplot3d import Axes3D
from sympy import Matrix, Symbol, symbols, solveset, solve, simplify, S, diff, det, erf, log, sqrt, pi, sin, cos, tan, atan2, init_printing

from evasdk import Eva
from json import dumps
 
import time
import cv2
from aravis import Camera

# 2. Initialize your kinematics functions and variables including the constants (dimensions, etc.) for the EVA robot arm.

# Initializes main kinematics class to assist with robot movement
def initialize_kinematics():
    # Dimensions of each link of the robot in metres
    arm_dimensions = [0.187, 0.096, 0.205, 0.124, 0.167, 0.104];
    
    # Robot initial pose (in joint angles)
    initial_pose = [0, 1, -2.5, 0, -1.6, 0] # [0, 1, -2.5, 0, -1.6, 0] # or [0,0,0,0,0,0]; # or [-pi/2,-pi/4,3*pi/4,0,pi/2,0];
    simulate = True # Whether the calculations should be displayed as a graphical simulation  
    
    # Variables to control calculation speed and accuracy
    dp_threshold = 0.01 # Calculations will stop when dp is lower than the threshold
    step_size = 0.01 # In each step, the robot will move this amount
    theta_max_step = 0.5 # Each joint won't rotate more than this angle in radians
    pause = 0.0000001 # Time the plot will be displayed
    
    # Variables to control how the plot is displayed
    plot_dimensions = [[-0.5, 0.5], # x axis limits in metres (min, max)
                       [-0.5, 0.5], # y axis limits in metres (min, max)
                       [0, 0.5]]    # z axis limits in metres (min, max)
    camera_view = [30, 45] # elev, azim (camera position and rotation)
    
    # All values are written to the Kinematics class
    kn = Kinematics(arm_dimensions, initial_pose, simulate, dp_threshold, step_size, theta_max_step, pause, plot_dimensions, camera_view);
    return kn

# 3. Initialize the arm and the camera as in the previous labs and start the camera capturing frames and showing them 
#    in an OpenCV window.

# Initializes robot arm module with specified conexion details
def initialize_arm():
    print("\nInitialising EVA robot arm...")
    
    # Details to connect to the robot arm, more info here: 
    # https://wiki.york.ac.uk/display/TSS/Network+Details+and+Credentials+for+the+EVA+Arms+and+Network+Cameras 
    arm_hostname = "evatrendylimashifterpt410"
    arm_ip = "144.32.152.105"
    token = "1462980d67d58cb7eaabe8790d866609eb97fd2c"
  
    # Create an “eva” object with these parameters to connect to the arm itself on the network
    eva = Eva(arm_ip, token)
    print("Done.\n")
    return eva

# Initializes camera module by connecting using the specified details
def initialize_camera():
    print("\nInitialising camera...")
    
    # Conexion details for the camera
    camera_hostname = "evacctv01"
    camera_ip = "144.32.152.102"
    camera_id = 'S1188411'

    # You can initialize a Camera object and set its parameters with:
    cam = Camera(camera_id)
    cam.set_feature("Width", 1936)
    cam.set_feature("Height", 1216)
    cam.set_frame_rate(10)
    cam.set_exposure_time(100000)
    cam.set_pixel_format_from_string('BayerRG8')

    # Print out the camera parameters in use
    print("\nCamera model: ", cam.get_model_name())
    print("Vendor Name: ", cam.get_vendor_name())
    print("Device id: ", cam.get_device_id())
    print("Region: ", cam.get_region(), end="")
    print("\n")
    
    return cam

def main():
    # Connect to the arm, camera and kinematics modules
    kn = initialize_kinematics();
    eva = initialize_arm();
    cam = initialize_camera();
    
    # Start camera acquisition block
    try:
        # Start the camera
        cam.start_acquisition_continuous()
        print("Camera On")
        
        # Open an OpenCV window to view the image
        cv2.namedWindow('capture', flags=0)
        
        # Capture an individual frame
        frame = cam.pop_frame()
        print("[", time.time(), "] initial frame. ""Shape: ", frame.shape)
        
        if not 0 in frame.shape:
            # Convert to standard RGB format
            image = cv2.cvtColor(frame, cv2.COLOR_BayerRG2RGB)
    
            # Show the image and wait a short time with:
            cv2.imshow("capture", image)
            cv2.waitKey(500)
            
        # 4. Move the arm to a starting position, ideally near where your target is located so that your camera can see it, 
        #    and initialize the state of the robot arm to have no object in its gripper and to move to pick up.
        target_position = [0, -0.2, 0.3];
        current_pose = kn.calculate_joint_angles(target_position, position_type = "absolute")
        
        with eva.lock():
          eva.control_wait_for_ready()
          eva.control_go_to(current_pose)
        
        # Open the gripper (to get rid of whatever is being gripped and test the gripper)
        with eva.lock():
           eva.control_wait_for_ready()
           eva.gpio_set('ee_d0', False)
           eva.gpio_set('ee_d1', True)
           
        time.sleep(0.2)
        
        # Close your gripper (or open it, depending on the design you have made), 
        # you need to set ee_d1 to a low logic state and ee_d2 to a high logic state
        with eva.lock():
           eva.control_wait_for_ready()
           eva.gpio_set('ee_d1', False)
           eva.gpio_set('ee_d0', True)
           
        # 5. Start your main program loop, in which you will probably need to:
        i = 0
        maximum_i = 50000
        while i < maximum_i:
            i += 1
        
            # a. Capture a frame from the camera and convert it to OpenCV format.
            frame = cam.pop_frame()
            print("[", time.time(), "] frame nb: ", i, " shape: ", frame.shape)
            
            if not 0 in frame.shape:
                # Convert to standard RGB format
                image = cv2.cvtColor(frame, cv2.COLOR_BayerRG2RGB)
                
                # b. Perform shape detection or blob detection using OpenCV to identify the target object to pick up.
                
                
                # c. Show the frame in the OpenCV window with detection annotations.
                # Show the image and wait a short time with:
                cv2.imshow("capture", image)
                cv2.waitKey(1)
            
            
            # d. Identify the location of the target object or the target drop location and convert this target location
            #    into the coordinate system that you are using for kinematics.
            
            # e. Calculate inverse kinematics for the robot arm to move to the next step in direction towards the target 
            #    location - it is usually better to move as a series of small steps rather than a single large motion 
            #    but it depends on how your inverse kinematics are implemented.
            
            # f. Detect when the gripper on the robot arm is located around the target object, and close the gripper, 
            #    or else detect that the arm has reached the drop location and open the gripper.
            
            # g. Repeat and change the current status of the arm if it has completed picking up or dropping an object.
    
        # 6. Complete the main program loop and return the arm to home position.
        
    # 7. Handle exceptions with “except KeyboardInterrupt:'' and “finally:” so that if problems occur the camera is shut down 
    #    and the arm stopped safely.  It is possible to use a “try” and “except” block to automatically reset the arm with 
    #    eva.control_reset_errors() if collisions occur, but this is rarely done in practice because for safety a human operator 
    #    should inspect the arm after an error before restarting automated motion.
    except KeyboardInterrupt:
       print("Exiting...")
       
    finally:
        # To automatically reset the arm. A human operator should inspect the arm after an error before restarting automated motion.
        eva.control_reset_errors()
        
        # Stop acquisition and shut down the camera with:
        cam.stop_acquisition()
        cam.shutdown()
        print("Camera Off")

if __name__ == "__main__":
    print("test")
    main();
