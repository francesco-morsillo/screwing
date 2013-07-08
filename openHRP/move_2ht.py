# ______________________________________________________________________________
# ******************************************************************************
#
#    BASIC EXAMPLE OF THE DYNAMIC SOT
#       Robot: HRP-2 N.14
#       Tasks: Rotate the head and move the arms
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
from dynamic_graph.sot.core import FeaturePoint6d
from dynamic_graph.sot.dyninv import TaskDynInequality
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple, matrixToRPY
from dynamic_graph.sot.dyninv.meta_task_dyn_6d import MetaTaskDyn6d
from dynamic_graph.sot.dyninv.meta_tasks_dyn import gotoNd
from dynamic_graph.sot.dyninv.meta_tasks_dyn_relative import MetaTaskDyn6dRel, gotoNdRel

from numpy import *



#-----------------------------------------------------------------------------
# --- OPERATIONAL TASKS (For HRP2-14)-----------------------------------------
#-----------------------------------------------------------------------------
def createTasks(robot, displacementMatrix):

	taskWaist = MetaTaskDyn6d('taskWaist', robot.dynamic, 'waist', 'waist')
	taskRH    = MetaTaskDyn6d('rh', robot.dynamic, 'rh', 'right-wrist')
	taskLH    = MetaTaskDyn6d('lh', robot.dynamic, 'lh', 'left-wrist')
	taskRel = MetaTaskDyn6dRel('rel',robot.dynamic,'rh','lh','right-wrist','left-wrist')

	for task in [ taskWaist, taskRH, taskLH, taskRel]:
	    task.feature.frame('current')
	    task.gain.setConstant(10)
	    task.task.dt.value = robot.timeStep
	    task.featureDes.velocity.value=(0,0,0,0,0,0)

	handMgrip=eye(4); handMgrip[0:3,3] = (0,0,-0.14)
	taskRH.opmodif = matrixToTuple(handMgrip)
	taskLH.opmodif = matrixToTuple(handMgrip)

	# RELATIVE POSITION TASK
	taskRel.opmodif = matrixToTuple(handMgrip)
	taskRel.opmodifBase = matrixToTuple(handMgrip)


	# Set the targets. Selec is the activation flag (say control only
# the XYZ translation), and gain is the adaptive gain (<arg1> at the target, <arg2>
# far from it, with slope st. at <arg3>m from the target, <arg4>% of the max gain
# value is reached
	taskRH.feature.position.recompute(robot.device.control.time)
	taskLH.feature.position.recompute(robot.device.control.time)
	targetLH = vectorToTuple(array(matrixToRPY( dot(displacementMatrix,array(taskLH.feature.position.value)) )))
	gotoNd(taskLH, targetLH, "111111",(50,1,0.01,0.9))

	gotoNdRel(taskRel,taskRH.feature.position.value,taskLH.feature.position.value,'110111',(50,1,0.01,0.9))
	taskRel.feature.errordot.value=(0,0,0,0,0)	# not to forget!!

	#Task Waist
	featureWaist = FeaturePoint6d('featureWaist')
	plug(robot.dynamic.waist,featureWaist.position)
	plug(robot.dynamic.Jwaist,featureWaist.Jq)
	taskWaist=TaskDynInequality('taskWaist')
	plug(robot.dynamic.velocity,taskWaist.qdot)
	taskWaist.add(featureWaist.name)
	taskWaist.selec.value = '110000'
	taskWaist.referenceInf.value = (0.,0.,0.,0.,-0.1,-0.7)    # Roll Pitch Yaw min
	taskWaist.referenceSup.value = (0.,0.,0.,0.,0.5,0.7)  # Roll Pitch Yaw max
	taskWaist.dt.value=robot.timeStep
	taskWaist.controlGain.value = 10

	tasks = array([taskRel.task,taskLH.task,taskWaist])

	return tasks


def move(solver, tasks):

	for i in range(size(tasks)):
		solver.push(tasks[i])
