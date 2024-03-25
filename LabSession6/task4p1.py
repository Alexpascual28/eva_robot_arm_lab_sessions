#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 20 14:41:07 2023

@title: Lab Session 6 Task 4: Integrating Control of the Robot Arm
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
from mpl_toolkits.mplot3d import Axes3D
from sympy import Matrix, Symbol, symbols, solveset, solve, simplify, S, diff, det, erf, log, sqrt, pi, sin, cos, tan, atan2, init_printing

# =============================================================================
# from evasdk import Eva
# from json import dumps
# 
# import time
# import cv2
# from aravis import Camera
# =============================================================================

def main():
    final_efector_point = [-0.5, -0.2, 0.2];
    simulate_arm(final_efector_point)

def simulate_arm(final_efector_point):
    # Get initial values
    # elements = define_initial_elements(final_efector_point);
    # dp = elements[-1]
    
    # Define your joint angles as SymPy symbols
    theta1, theta2, theta3, theta4, theta5, theta6 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6')
    theta = Matrix([theta1,theta2,theta3,theta4,theta5,theta6])
     
    # Define transforms to each joint
    T1 = Ry(-pi/2) * T(0.187, 0, 0) * Rx(theta1)
    T2 = T1 * T(0.096, 0, 0) * Rz(theta2)
    T3 = T2 * T(0.205, 0, 0) * Rz(theta3)
    T4 = T3 * T(0.124, 0, 0) * Rx(theta4)
    T5 = T4 * T(0.167, 0, 0) * Rz(theta5)
    T6 = T5 * T(0.104, 0, 0) * Rx(theta6)
     
    # Find joint positions in space
    # p0, p1, p2, and p3 are points in 3D space defined as symbolic Matrices that 
    # can be transformed mathematically – note that they are actually four-dimensional 
    # so they can be used with homogeneous transforms with the last ‘W’ coordinate 
    # always 1 (to facilitate translation as well as rotation).
    p0 = Matrix([0,0,0,1])
    p1 = T1 * p0
    p2 = T2 * p0
    p3 = T3 * p0
    p4 = T4 * p0
    p5 = T5 * p0
    p6 = T6 * p0
     
    p = Matrix([p6[0], p6[1], p6[2]]) # coordinates of arm tip
     
    # Initial position θi to start movement from (straight up position, all joint angles=0)
    theta_i = Matrix([0,0,0,0,0,0])
     
    # Or you could use the “home” position from lab 3, 
    # though you need to change the variable names to use for substitution:
    # theta_i = Matrix([-pi/2,-pi/4,3*pi/4,0,pi/2,0])
     
    # Calculate the initial (current) point of the end effector directly 
    # from the joint angles by substituting theta in the forward kinematics of point p
    p_i = p.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2],
                  theta4:theta_i[3], theta5:theta_i[4],theta6:theta_i[5]}).evalf()
     
    # Define the final (target) point of the end effector also as simply 
    # a relative movement from your initial position, which you can do 
    # as follows for the example of moving the arm down in the z axis by 1cm
    # p_f = p_i + Matrix([0, 0, -0.01]);
     
    # Or define a final point as set coordinates
    p_f = Matrix(final_efector_point);
     
    # You can model the movement of the end of the arm with the 
    # difference dp = pf-pi and this must be caused by the 
    # joint angles θ moving by a difference dθ
    dp = p_f - p_i;
    
    # Threshold value to exit the loop
    dp_threshold = 0.01
    
    # Variables used to move in small increments, to reduce the vector dp into incremental steps.
    # Step the end effector position by 0.5mm each iteration
    # and change angle by no more than 0.2 radians
    step_size = 0.01
    theta_max_step = 0.2
    
    # Loop to iterate through a number of points from the start position pi to the end position pf
    while dp.norm() > dp_threshold:
        
        # theta_i, p_i, dp, X, Y, Z, W = calculate_next_position(elements, step_size, theta_max_step)
        
        # theta1, theta2, theta3, theta4, theta5, theta6, theta, theta_i, p0, p1, p2, p3, p4, p5, p6, p, p_f, dp = elements;
      
        # dp represents the distance between where the end effector is now and
        # our goal position. We can define a step size for x,y, and z coordinates
        # dp_step as follows
        dp_step = dp * step_size / dp.norm()
          
        # To calculate the numerical value of J (jacobian) at each point,
        # you can produce the Jacobian as follows
        J = p.jacobian(theta)
        J_i = J.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2],
                      theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
          
        # Solve the expression to find values of θ for the new point (solve for dθ)
        # To do this we calculate the inverse of J, and multiply this by dp_step, since J*dθ=dp => dθ=dp*J^-1
        J_inv = J_i.pinv()
        dtheta = J_inv * dp_step
          
        # Update your theta_i joint values, limiting the joint movement to theta_max_step, 
        # which can be done with numpy.clip()
        theta_i = theta_i + numpy.clip(dtheta,-1*theta_max_step,theta_max_step)
          
        # Update the new p_i from your forward kinematics
        p_i = p.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2],
                      theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
          
        # Update dp to reflect the current vector to the goal position
        dp = p_f - p_i
          
        # Plot the actual position of the arm in each loop
        p0sub = p0.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], 
                         theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
        p1sub = p1.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], 
                         theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
        p2sub = p2.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], 
                         theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
        p3sub = p3.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], 
                         theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
        p4sub = p4.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], 
                         theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
        p5sub = p5.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], 
                         theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
        p6sub = p6.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], 
                         theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
        
        soa = numpy.array([p0sub,p1sub,p2sub,p3sub,p4sub,p5sub,p6sub])
        X, Y, Z, W = zip(*soa)
          
        X = numpy.array(X)
        Y = numpy.array(Y)
        Z = numpy.array(Z)
        W = numpy.array(W)
          
        X = numpy.ndarray.flatten(X)
        Y = numpy.ndarray.flatten(Y)
        Z = numpy.ndarray.flatten(Z)
        W = numpy.ndarray.flatten(W)
        
        # Draw current position in plot
        ax = create_plot();
        draw_elements_in_plot(ax, X, Y, Z);

        # Print current step position and rotation
        print("step “,step,”:\n θ[",theta_i,"]\n p[",p_i,"]")
    
    # Print final joint angles
    print('\n\nFinal Joint Angles in Radians:\n', theta_i.evalf())

    return;

def define_initial_elements(final_efector_point):
    # Define your joint angles as SymPy symbols
    theta1, theta2, theta3, theta4, theta5, theta6 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6')
    theta = Matrix([theta1,theta2,theta3,theta4,theta5,theta6])
    
    # Define transforms to each joint
    T1 = Ry(-pi/2) * T(0.187, 0, 0) * Rx(theta1)
    T2 = T1 * T(0.096, 0, 0) * Rz(theta2)
    T3 = T2 * T(0.205, 0, 0) * Rz(theta3)
    T4 = T3 * T(0.124, 0, 0) * Rx(theta4)
    T5 = T4 * T(0.167, 0, 0) * Rz(theta5)
    T6 = T5 * T(0.104, 0, 0) * Rx(theta6)
    
    # Find joint positions in space
    # p0, p1, p2, and p3 are points in 3D space defined as symbolic Matrices that 
    # can be transformed mathematically – note that they are actually four-dimensional 
    # so they can be used with homogeneous transforms with the last ‘W’ coordinate 
    # always 1 (to facilitate translation as well as rotation).
    p0 = Matrix([0,0,0,1])
    p1 = T1 * p0
    p2 = T2 * p0
    p3 = T3 * p0
    p4 = T4 * p0
    p5 = T5 * p0
    p6 = T6 * p0
    
    p = Matrix([p6[0], p6[1], p6[2]]) # coordinates of arm tip
    
    # Initial position θi to start movement from (straight up position, all joint angles=0)
    theta_i = Matrix([0,0,0,0,0,0])
    
    # Or you could use the “home” position from lab 3, 
    # though you need to change the variable names to use for substitution:
    # theta_i = Matrix([-pi/2,-pi/4,3*pi/4,0,pi/2,0])
    
    # Calculate the initial (current) point of the end effector directly 
    # from the joint angles by substituting theta in the forward kinematics of point p
    p_i = p.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2],
                  theta4:theta_i[3], theta5:theta_i[4],theta6:theta_i[5]}).evalf()
    
    # Define the final (target) point of the end effector also as simply 
    # a relative movement from your initial position, which you can do 
    # as follows for the example of moving the arm down in the z axis by 1cm
    # p_f = p_i + Matrix([0, 0, -0.01]);
    
    # Or define a final point as set coordinates
    p_f = Matrix(final_efector_point);
    
    # You can model the movement of the end of the arm with the 
    # difference dp = pf-pi and this must be caused by the 
    # joint angles θ moving by a difference dθ
    dp = p_f - p_i;
    
    return [theta1, theta2, theta3, theta4, theta5,theta6, theta, 
            theta_i, p0, p1, p2, p3, p4, p5, p6, p, p_f, dp]

def calculate_next_position(elements, step_size, theta_max_step):
    
    [theta1, theta2, theta3, theta4, theta5, theta6, theta, theta_i,
     p0, p1, p2, p3, p4, p5, p6, p, p_f, dp] = elements;
    
    # dp represents the distance between where the end effector is now and
    # our goal position. We can define a step size for x,y, and z coordinates
    # dp_step as follows
    dp_step = dp * step_size / dp.norm()
    
    # To calculate the numerical value of J (jacobian) at each point,
    # you can produce the Jacobian as follows
    J = p.jacobian(theta)
    J_i = J.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2],
                  theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
    
    # Solve the expression to find values of θ for the new point (solve for dθ)
    # To do this we calculate the inverse of J, and multiply this by dp_step, since J*dθ=dp => dθ=dp*J^-1
    J_inv = J_i.pinv()
    dtheta = J_inv * dp_step
    
    # Update your theta_i joint values, limiting the joint movement to theta_max_step, 
    # which can be done with numpy.clip()
    theta_i = theta_i + numpy.clip(dtheta,-1*theta_max_step,theta_max_step)
    
    # Update the new p_i from your forward kinematics
    p_i = p.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2],
                  theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
    
    # Update dp to reflect the current vector to the goal position
    dp = p_f - p_i

    # Plot the actual position of the arm in each loop
    p0sub = p0.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], 
                     theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
    p1sub = p1.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], 
                     theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
    p2sub = p2.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], 
                     theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
    p3sub = p3.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], 
                     theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
    p4sub = p4.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], 
                     theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
    p5sub = p5.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], 
                     theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
    p6sub = p6.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], 
                     theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
    
    soa = numpy.array([p0sub,p1sub,p2sub,p3sub,p4sub,p5sub,p6sub])
    X, Y, Z, W = zip(*soa)
    
    X = numpy.array(X)
    Y = numpy.array(Y)
    Z = numpy.array(Z)
    W = numpy.array(W)
    
    X = numpy.ndarray.flatten(X)
    Y = numpy.ndarray.flatten(Y)
    Z = numpy.ndarray.flatten(Z)
    W = numpy.ndarray.flatten(W)
    
    return theta_i, p_i, dp, X, Y, Z, W

def create_plot():
    # fig.add_subplot(111, projection='3d') creates MatPlotLib axes that you 
    # can plot vectors and points on to
    fig = matplotlib.pyplot.figure(1)
    ax = fig.add_subplot(111, projection='3d')

    # set_xlabel() sets the label to a string argument on a parent axis
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    
    ax.set_xlim([-0.5, 0.5])
    ax.set_ylim([-0.5, 0.5])
    ax.set_zlim([0, 0.5])
    
    # Change initial orientation of the plot view perspective
    ax.view_init(elev=0, azim=-90)
    
    return ax;

def draw_elements_in_plot(ax, X, Y, Z):
    # ax.plot3D(X,Y,Z, 'blue', marker="o") plots lines connecting the points specified
    # in X, Y, and Z in the colour green (you can change this colour as you wish) and 
    # with the marker style specified (we use “o” for circles at joints)
    ax.plot3D(X,Y,Z, 'blue', marker="o")

    # matplotlib.pyplot.draw() draws the plot, matplotlib.pyplot.show() shows the 
    # plot, and matplotlib.pyplot.pause() runs the plotting event loop for a time.
    matplotlib.pyplot.draw()
    matplotlib.pyplot.show()
    matplotlib.pyplot.pause(0.0000001)
    
    return;

# Translation homogeneous matrix
def T(x, y, z):
   T_xyz = Matrix([[1,         0,          0,          x],
                   [0,         1,          0,          y],
                   [0,         0,          1,          z],
                   [0,         0,          0,          1]])
   return T_xyz

# Rotation along x axis homogeneous matrix
def Rx(roll):
   R_x = Matrix([[1,         0,          0, 0],
                 [0, cos(roll), -sin(roll), 0],
                 [0, sin(roll),  cos(roll), 0],
                 [0,         0,          0, 1]])
   return R_x

# Rotation along y axis homogeneous matrix
def Ry(pitch):
   R_y = Matrix([[ cos(pitch), 0, sin(pitch), 0],
                 [          0, 1,          0, 0],
                 [-sin(pitch), 0, cos(pitch), 0],
                 [          0, 0,          0, 1]])
   return R_y

# Rotation along z axis homogeneous matrix
def Rz(yaw):
   R_z = Matrix([[cos(yaw),-sin(yaw), 0, 0],
                 [sin(yaw), cos(yaw), 0, 0],
                 [       0,        0, 1, 0],
                 [       0,        0, 0, 1]])
   return R_z

# Compound rotation homogeneous matrix. 
# The order of rotations must be consistent.
def R(roll, pitch, yaw):
   R_x = Matrix([[1,         0,          0],
                 [0, cos(roll), -sin(roll)],
                 [0, sin(roll),  cos(roll)]])

   R_y = Matrix([[ cos(pitch), 0, sin(pitch)],
                 [          0, 1,          0],
                 [-sin(pitch), 0, cos(pitch)]])

   R_z = Matrix([[cos(yaw),-sin(yaw), 0],
                 [sin(yaw), cos(yaw), 0],
                 [       0,        0, 1]])
   return R_z*R_y*R_x
    
if __name__ == "__main__":
    main();