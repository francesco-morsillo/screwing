
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
"""

from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d
from dynamic_graph.sot.core.meta_tasks import gotoNd
from numpy import eye

def moveRightHandToTarget(robot,solver,target):
	
	# TASKRH Definition
	taskRH=MetaTaskKine6d('rh',robot.dynamic,'rh','right-wrist')
	handMgrip=eye(4); handMgrip[0:3,3] = (0,0,-0.21)
	taskRH.opmodif = matrixToTuple(handMgrip)
	taskRH.feature.frame('desired')

	# Reference and gain setting
	gotoNd(taskRH,target,'000111',(5,1,0.01,0.9))

	# push
	solver.sot.push(taskRH.task.name)

