# -*- coding: utf-8 -*-
"""
Created on Wed Apr 19 15:13:25 2023

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
    X, Y, Z, W = define_elements();
    ax = create_plot();
    draw_elements_in_plot(ax, X, Y, Z);
    return;

def define_elements():
    theta1=-pi/2
    theta2=-pi/4
    theta3=3*pi/4
    theta4=0
    theta5=pi/2
    theta6=0
    
    T1 = Ry(-pi/2) * T(0.187, 0, 0) * Rx(theta1)
    T2 = T1 * T(0.096, 0, 0) * Rz(theta2)
    T3 = T2 * T(0.205, 0, 0) * Rz(theta3)
    T4 = T3 * T(0.124, 0, 0) * Rx(theta4)
    T5 = T4 * T(0.167, 0, 0) * Rz(theta5)
    T6 = T5 * T(0.104, 0, 0) * Rx(theta6)
    
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
    
    return X, Y, Z, W;

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
