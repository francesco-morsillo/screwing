# ______________________________________________________________________________
# ******************************************************************************
#
#    GO TO HALF SITTING POSITION
#       Robot: HRP-2 N.14
#       Tasks: Go in the half-sitting pose
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

from numpy import array

from dynamic_graph.sot.screwing.openHRP.move_2ht import removeUndesiredTasks

#-----------------------------------------------------------------------------
# --- OPERATIONAL TASKS (For HRP2-14)-----------------------------------------
#-----------------------------------------------------------------------------

def goToHalfSit(robot,solver):

    # Remove Other Tasks
    removeUndesiredTasks(solver)

    # Pose definition
    pose = array(robot.halfSitting)

    # New taskPosture creation
    robot.mTasks['posture'].ref = vectorToTuple(pose)
    robot.mTasks['posture'].gain.setConstant(10)
