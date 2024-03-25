#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 20 14:18:45 2023

@title: Lab Session 6 Task 3: Viewing Targets with Arm Camera
@author: Alejandro Pascual San Roman (bdh532)
@organisation: School of Physics, Engineering and Technology. University of York

"""

import time
import cv2
from aravis import Camera

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
print("Camera model: ", cam.get_model_name())
print("Vendor Name: ", cam.get_vendor_name())
print("Device id: ", cam.get_device_id())
print("Region: ", cam.get_region())

try:
    # Start the camera
    cam.start_acquisition_continuous()
    print("Camera On")
    
    # Open an OpenCV window to view the image
    cv2.namedWindow('capture', flags=0)
    
    i = 0
    maximum_i = 500
    while i < maximum_i:
        i += 1
        
        # Capture an individual frame
        frame = cam.pop_frame()
        print("[", time.time(), "] frame nb: ", i, " shape: ", frame.shape)
        
        if not 0 in frame.shape:
            # Convert to standard RGB format
            image = cv2.cvtColor(frame, cv2.COLOR_BayerRG2RGB)
            
            # Save the image to a picture file
            path = "image.png"
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

