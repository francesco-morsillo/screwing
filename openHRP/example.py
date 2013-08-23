
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
from dynamic_graph.sot.core.meta_task_posture import MetaTaskKinePosture
from dynamic_graph.sot.core.meta_tasks import gotoNd
from numpy import eye

def moveRightHandToTarget(robot,solver,target):
	
	# TASKRH Definition
	mTaskRH=MetaTaskKine6d('rh',robot.dynamic,'rh','right-wrist')
	handMgrip=eye(4); handMgrip[0:3,3] = (0,0,-0.21)
	mTaskRH.opmodif = matrixToTuple(handMgrip)
	mTaskRH.feature.frame('desired')

	# Reference and gain setting
	gotoNd(mTaskRH,target,'000111',(5,1,0.01,0.9))


	# Task Posture
	mTaskPosture = MetaTaskKinePosture(robot.dynamic)
	mTaskPosture.ref = robot.halfSitting

	# push
	solver.sot.push(mTaskRH.task.name)
	solver.sot.push(mTaskPosture.task.name)

