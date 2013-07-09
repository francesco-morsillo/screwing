# ______________________________________________________________________________
# ******************************************************************************
#
#    OPEN GRIPPERS
#       Robot: HRP-2 N.14
#       Tasks: Open the grippers completely
# 
# ______________________________________________________________________________
# ******************************************************************************  


"""
Already loaded on the python interpreter:
	- robot with dynamics and control
	- solver to produce the control
	- dynamic graph

The needed part is strictly the task definition and the push in the stack of tasks.

This script is thought to be used after having initialized the solver with the
function 'initialize' from sot.application.acceleration.precomputed_tasks. This
script creates and loads the following tasks:
       - Left foot and right foot (as a constraint, not as a task)
       - COM for balance
       - TaskLim to respect joint limits
       - TaskPosture to keep the half-sitting posture as the aim for the unused DOF

"""

from dynamic_graph import plug

from dynamic_graph.sot.core.matrix_util import vectorToTuple

from numpy import *


#-----------------------------------------------------------------------------
# --- OPERATIONAL TASKS (For HRP2-14)-----------------------------------------
#-----------------------------------------------------------------------------

def openGrippers(robot,solver):

    # Pose definition
    pose = array(robot.halfSitting)
    pose[28] = 0.75
    pose[35] = 0.75

    # New taskPosture creation
    robot.taskPosture.ref = vectorToTuple(pose)
    robot.taskPosture.gain.setConstant = 10
