# -*- coding: utf-8 -*-
"""
Created on Wed Apr 19 11:34:15 2023

@title: Lab Session 3 Task 1
@author: Alejandro Pascual San Roman (bdh532)
@organisation: School of Physics, Engineering and Technology. University of York

"""

import numpy
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
from sympy import Matrix, Symbol, symbols, solveset
from sympy import S, erf, log, sqrt, pi, sin, cos, tan
from sympy import init_printing

# p0, p1, p2, and p3 are points in 3D space defined as symbolic Matrices that 
# can be transformed mathematically – note that they are actually four-dimensional 
# so they can be used with homogeneous transforms with the last ‘W’ coordinate 
# always 1 (to facilitate translation as well as rotation).
p0 = Matrix([0,0,0,1])
p1 = Matrix([0,0,0.187,1])
p2 = Matrix([0,0,0.187+0.096,1])
p3 = Matrix([0.205*cos(pi/4),0,0.187+0.096+0.205*sin(pi/4),1])
p4 = Matrix([0.205*cos(pi/4)+0.124,0,0.187+0.096+0.205*sin(pi/4),1])
p5 = Matrix([0.205*cos(pi/4)+0.124+0.167,0,0.187+0.096+0.205*sin(pi/4),1])
p6 = Matrix([0.205*cos(pi/4)+0.124+0.167,0,0.187+0.096+0.205*sin(pi/4)-0.104,1])

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

# ax.plot3D(X,Y,Z, 'blue', marker="o") plots lines connecting the points specified
# in X, Y, and Z in the colour blue (you can change this colour as you wish) and 
# with the marker style specified (we use “o” for circles at joints)
ax.plot3D(X,Y,Z, 'blue', marker="o")
matplotlib.pyplot.show()
ax.plot3D(X,Y,Z, 'blue', marker="o")

# matplotlib.pyplot.draw() draws the plot, matplotlib.pyplot.show() shows the 
# plot, and matplotlib.pyplot.pause() runs the plotting event loop for a time.
matplotlib.pyplot.draw()
matplotlib.pyplot.show()
matplotlib.pyplot.pause(1)
