# -*- coding: utf-8 -*-
"""
Created on Wed Apr 19 16:10:42 2023

@title: Lab Session 5 Task 3: MOVEMENT SOLUTION
@author: Alejandro Pascual San Roman (bdh532)
@organisation: School of Physics, Engineering and Technology. University of York

"""

import numpy
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
from sympy import Matrix, Symbol, symbols, solveset, solve, simplify, S, diff, det, erf, log, sqrt, pi, sin, cos, tan, atan2, init_printing

def main():
    theta, p_i, p_f, dp, p, J = define_elements();
    
    print(theta);
    print(p_i);
    print(p_f);
    print(dp);
    print(p);
    print(J);
    
    return;

def define_elements():
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
    
    # You can produce the Jacobian as follows
    J = p.jacobian(theta)
    
    # Initial position θi to start movement from (straight up position, all joint angles=0)
    # theta_i = Matrix([0,0,0,0,0,0])
    
    # Or you could use the “home” position from lab 3, 
    # though you need to change the variable names to use for substitution:
    theta_i = Matrix([-pi/2,-pi/4,3*pi/4,0,pi/2,0])
    
    # Calculate the initial (current) point of the end effector directly 
    # from the joint angles by substituting theta in the forward kinematics of point p
    p_i = p.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2],
                  theta4:theta_i[3], theta5:theta_i[4],theta6:theta_i[5]}).evalf()
    
    # Define the final (target) point of the end effector also as simply 
    # a relative movement from your initial position, which you can do 
    # as follows for the example of moving the arm down in the z axis by 1cm
    p_f = p_i + Matrix([0, 0, -0.01]);
    
    # You can model the movement of the end of the arm with the 
    # difference dp = pf-pi and this must be caused by the 
    # joint angles θ moving by a difference dθ
    dp = p_f - p_i;
    
    return theta, p_i, p_f, dp, p, J;

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

main();
