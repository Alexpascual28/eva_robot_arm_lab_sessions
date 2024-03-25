# -*- coding: utf-8 -*-
"""
Created on Mon May  1 17:24:39 2023

@title: Lab Session 7 Task 4: Implement Placement Motion
@author: Alejandro Pascual San Roman (bdh532)
@organisation: School of Physics, Engineering and Technology. University of York

"""

# =============================================================================

# Once the target object is in your gripper, you should move it to a new location 
# and simply put it down.  You can move the object to any location within the reach 
# of the arm that you wish, but try to place it gently at the same height that you 
# picked it up from so that it remains standing.  If you can successfully complete 
# the full pick up and place motion automatically using camera feedback, you will 
# be well on your way to doing well in the assessment for this term.

# =============================================================================


kn = Kinematics(arm_dimensions, initial_pose, simulate, dp_threshold, step_size, theta_max_step, pause, plot_dimensions, camera_view);

