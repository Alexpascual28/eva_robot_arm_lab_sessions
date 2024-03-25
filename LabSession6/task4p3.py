# -*- coding: utf-8 -*-
"""
Created on Mon May  1 14:18:57 2023

@title: Lab Session 6 Task 4 Part 3: Integrating Control of the Robot Arm (using kinematics class)
@author: Alejandro Pascual San Roman (bdh532)
@organisation: School of Physics, Engineering and Technology. University of York

"""

# Integrate the inverse kinematics calculations from Session 5 with the 
# robot and camera control from this session. The system should: 
# 1. connect to the arm
# 2. move to the home position
# 3. capture an image
# 4. move to a new point defined by your inverse kinematics and capture another image

# Calibrating the camera is optional but useful. 
# See more info here: https://docs.opencv.org/4.5.1/d4/d94/tutorial_camera_calibration.html

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

# Initializes main kinematics class to assist with robot movement
def initialize_kinematics():
    # Dimensions of each link of the robot in metres
    arm_dimensions = [0.187, 0.096, 0.205, 0.124, 0.167, 0.104];
    
    # Robot initial pose (in joint angles)
    initial_pose = [0, 1, -2.5, 0, -1.6, 0] # [0, 1, -2.5, 0, -1.6, 0] # or [0,0,0,0,0,0]; # or [-pi/2,-pi/4,3*pi/4,0,pi/2,0];
    simulate = False # Whether the calculations should be displayed as a graphical simulation  
    
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

def main():
    # 1. connect to the arm
    kn = initialize_kinematics();
    eva = initialize_arm();
    cam = initialize_camera();
    
    # 2. move to the home position
    target_position = [0, -0.2, 0.3];
    current_pose = kn.calculate_joint_angles(target_position, position_type = "absolute")
    
    with eva.lock():
      eva.control_wait_for_ready()
      eva.control_go_to(current_pose)
    
    # Start camera acquisition block
    try:
        # 3. capture an image
        
        # Start the camera
        cam.start_acquisition_continuous()
        print("Camera On")
        
        # Open an OpenCV window to view the image
        cv2.namedWindow('capture', flags=0)
        
        # Capture an individual frame
        frame = cam.pop_frame()
        print("[", time.time(), "] frame nb: ", 1, " shape: ", frame.shape)
        
        if not 0 in frame.shape:
            # Convert to standard RGB format
            image = cv2.cvtColor(frame, cv2.COLOR_BayerRG2RGB)
            
            # Save the image to a picture file
            path = "task4_image1.png"
            print("Saving image to", path)
            cv2.imwrite(path, image)
    
            # Show the image and wait a short time with:
            cv2.imshow("capture", image)
            cv2.waitKey(1)
            
        # 4. move to a new point defined by your inverse kinematics and capture another image
        target_position = [0.1, -0.05, -0.1];
        current_pose = kn.calculate_joint_angles(target_position, position_type = "relative")
        
        with eva.lock():
          eva.control_wait_for_ready()
          eva.control_go_to(current_pose)
          
        # Capture an individual frame
        frame = cam.pop_frame()
        print("[", time.time(), "] frame nb: ", 2, " shape: ", frame.shape)
        
        if not 0 in frame.shape:
            # Convert to standard RGB format
            image = cv2.cvtColor(frame, cv2.COLOR_BayerRG2RGB)
            
            # Save the image to a picture file
            path = "task4_image2.png"
            print("Saving image to", path)
            cv2.imwrite(path, image)
    
            # Show the image and wait a short time with:
            cv2.imshow("capture", image)
            cv2.waitKey(1)

    except KeyboardInterrupt:
       print("Exiting...")
       
    finally:
        # Stop acquisition and shut down the camera with:
        cam.stop_acquisition()
        cam.shutdown()
        print("Camera Off")
        
    return;
   
if __name__ == "__main__":
    main();

