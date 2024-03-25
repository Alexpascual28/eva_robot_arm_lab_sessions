# -*- coding: utf-8 -*-
"""
Created on Wed Apr 19 15:41:50 2023

@title: Lab Session 5 Task 2: CREATE A JACOBIAN FOR YOUR ARM
@author: Alejandro Pascual San Roman (bdh532)
@organisation: School of Physics, Engineering and Technology. University of York

"""

import numpy
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
from sympy import Matrix, Symbol, symbols, solveset, solve, simplify, S, diff, det, erf, log, sqrt, pi, sin, cos, tan, atan2, init_printing

def main():
    X, Y, Z, W, J = define_elements();
    
    print(J.shape);
    print(J);
    
    # ax = create_plot();
    # draw_elements_in_plot(ax, X, Y, Z);
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
    
    # Jacobian matrix defined by differentiation
    j11 = diff(p[0], theta1) # differentiate px with theta_1
    j12 = diff(p[0], theta2) # differentiate px with theta_2
    j13 = diff(p[0], theta3) # differentiate px with theta_3
    j14 = diff(p[0], theta4) # differentiate px with theta_4
    j15 = diff(p[0], theta5) # differentiate px with theta_5
    j16 = diff(p[0], theta6) # differentiate px with theta_6
    
    j21 = diff(p[1], theta1) # differentiate py with theta_1
    j22 = diff(p[1], theta2) # differentiate py with theta_2
    j23 = diff(p[1], theta3) # differentiate py with theta_3
    j24 = diff(p[1], theta4) # differentiate py with theta_4
    j25 = diff(p[1], theta5) # differentiate py with theta_5
    j26 = diff(p[1], theta6) # differentiate py with theta_6
    
    j31 = diff(p[2], theta1) # differentiate pz with theta_1
    j32 = diff(p[2], theta2) # differentiate pz with theta_2
    j33 = diff(p[2], theta3) # differentiate pz with theta_3
    j34 = diff(p[2], theta4) # differentiate pz with theta_4
    j35 = diff(p[2], theta5) # differentiate pz with theta_5
    j36 = diff(p[2], theta6) # differentiate pz with theta_6
    
    J = Matrix([[j11, j12, j13, j14, j15, j16],
                [j21, j22, j23, j24, j25, j26],
                [j31, j32, j33, j34, j35, j36]]) # assemble into matrix form
    
    # You can also produce this Jacobian as follows
    J = p.jacobian(theta)

    # numpy.array() concatenates these points into a column (enter “soa” to see)
    soa = numpy.array([p0,p1,p2,p3,p4,p5,p6])

    # zip() extracts the columns from soa so that they are column vectors of X, Y,
    # and Z components, with the W component unused for 3D projection (always=1)
    X, Y, Z, W = zip(*soa)

    # additional calls to numpy.array() and then numpy.ndarray.flatten() are to 
    # ensure that a flat set of vectors result
    X = numpy.array(X)
    Y = numpy.array(Y)
    Z = numpy.array(Z)
    W = numpy.array(W)

    X = numpy.ndarray.flatten(X)
    Y = numpy.ndarray.flatten(Y)
    Z = numpy.ndarray.flatten(Z)
    W = numpy.ndarray.flatten(W)
    
    return X, Y, Z, W, J;

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
    
    return ax;

def draw_elements_in_plot(ax, X, Y, Z):
    # ax.plot3D(X,Y,Z, 'blue', marker="o") plots lines connecting the points specified
    # in X, Y, and Z in the colour green (you can change this colour as you wish) and 
    # with the marker style specified (we use “o” for circles at joints)
    ax.plot3D(X,Y,Z, 'green', marker="o")

    # matplotlib.pyplot.draw() draws the plot, matplotlib.pyplot.show() shows the 
    # plot, and matplotlib.pyplot.pause() runs the plotting event loop for a time.
    matplotlib.pyplot.draw()
    matplotlib.pyplot.show()
    matplotlib.pyplot.pause(1)
    
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

main();
