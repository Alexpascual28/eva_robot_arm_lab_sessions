# -*- coding: utf-8 -*-
"""
Created on Wed Apr 19 11:54:55 2023

@title: Lab Session 3 Task 2
@author: Alejandro Pascual San Roman (bdh532)
@organisation: School of Physics, Engineering and Technology. University of York

"""

import numpy
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
from sympy import Matrix, Symbol, symbols, solveset
from sympy import S, erf, log, sqrt, pi, sin, cos, tan
from sympy import init_printing

def main():
    print(define_transformations());
    return;

# Defines robot arm transformations
def define_transformations():
    theta1 = 0;
    theta2 = 0;
    theta3 = 0;
    theta4 = 0;
    theta5 = 0;
    theta6 = 0;
    
    T1 = Ry(-pi/2) * T(0.187, 0, 0) * Rx(theta1)
    T2 = T1 * T(0.096, 0, 0) * Rz(theta2)
    T3 = T2 * T(0.205, 0, 0) * Rz(theta3)
    T4 = T3 * T(0.124, 0, 0) * Rx(theta4)
    T5 = T4 * T(0.167, 0, 0) * Rz(theta5)
    T6 = T5 * T(0.104, 0, 0) * Rx(theta6)
    
    return T1, T2, T3, T4, T5, T6;

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
