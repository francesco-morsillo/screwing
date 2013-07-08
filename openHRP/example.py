
# ______________________________________________________________________________
# ******************************************************************************
# SIMPLEST EXAMPLE OF INVERSE KINEMATICS.
# The robot moves the hand to a given position.
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

from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.dyninv.meta_task_dyn_6d import MetaTaskDyn6d
from dynamic_graph.sot.dyninv.meta_tasks_dyn import gotoNd
from dynamic_graph.sot.core import *
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.dyninv import *

from numpy import eye

def moveRightHandToTarget(robot,solver,target):

	############################################################
	### TASK DEFINITION ########################################
	############################################################

	# Task Right Hand
	taskRH = MetaTaskDyn6d('rh', robot.dynamic, 'rh', 'right-wrist')
	taskRH.feature.frame('desired')
	taskRH.task.dt.value = robot.timeStep
	taskRH.featureDes.velocity.value=(0,0,0,0,0,0)
	handMgrip=eye(4); handMgrip[0:3,3] = (0,0,-0.17)
	taskRH.opmodif = matrixToTuple(handMgrip)

	############################################################
	# Reference and gain setting
	############################################################

	gotoNd(taskRH,target,'000111',(50,1,0.01,0.9))

	############################################################
	# Push
	############################################################

	solver.push(taskRH.task)


