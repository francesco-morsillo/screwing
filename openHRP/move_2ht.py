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
"""

from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.dyninv.meta_task_dyn_6d import MetaTaskDyn6d
from dynamic_graph.sot.dyninv.meta_tasks_dyn import MetaTaskDynCom, MetaTaskDynPosture, gotoNd
from dynamic_graph.sot.dyninv.meta_tasks_dyn_relative import MetaTaskDyn6dRel, gotoNdRel

from dynamic_graph.sot.fmorsill.utility import *

from numpy import *



#-----------------------------------------------------------------------------
# --- OPERATIONAL TASKS (For HRP2-14)-----------------------------------------
#-----------------------------------------------------------------------------
def createTasks(robot):

	taskWaist = MetaTaskDyn6d('taskWaist', robot.dynamic, 'waist', 'waist')
	taskChest = MetaTaskDyn6d('taskChest', robot.dynamic, 'chest', 'chest')
	taskHead  = MetaTaskDyn6d('taskHead', robot.dynamic, 'head', 'gaze')
	taskRH    = MetaTaskDyn6d('rh', robot.dynamic, 'rh', 'right-wrist')
	taskLH    = MetaTaskDyn6d('lh', robot.dynamic, 'lh', 'left-wrist')
	taskRel = MetaTaskDyn6dRel('rel',robot.dynamic,'rh','lh','right-wrist','left-wrist')

	for task in [ taskWaist, taskChest, taskHead, taskRH, taskLH, taskRel]:
	    task.feature.frame('current')
	    task.gain.setConstant(10)
	    task.task.dt.value = dt
	    task.featureDes.velocity.value=(0,0,0,0,0,0)

	handMgrip=eye(4); handMgrip[0:3,3] = (0,0,-0.14)
	taskRH.opmodif = matrixToTuple(handMgrip)
	taskLH.opmodif = matrixToTuple(handMgrip)

	# RELATIVE POSITION TASK
	taskRel.opmodif = matrixToTuple(handMgrip)
	taskRel.opmodifBase = matrixToTuple(handMgrip)

	# CoM Task
	taskCom = MetaTaskDynCom(robot.dynamic,dt)

	# Posture Task
	taskPosture = MetaTaskDynPosture(robot.dynamic,dt)

	# Angular position and velocity limits
	taskLim = TaskDynLimits('taskLim')
	plug(robot.dynamic.position,taskLim.position)
	plug(robot.dynamic.velocity,taskLim.velocity)
	taskLim.dt.value = dt

	robot.dynamic.upperJl.recompute(0)
	robot.dynamic.lowerJl.recompute(0)
	taskLim.referencePosInf.value = robot.dynamic.lowerJl.value
	taskLim.referencePosSup.value = robot.dynamic.upperJl.value

	#dqup = (0, 0, 0, 0, 0, 0, 200, 220, 250, 230, 290, 520, 200, 220, 250, 230, 290, 520, 250, 140, 390, 390, 240, 140, 240, 130, 270, 180, 330, 240, 140, 240, 130, 270, 180, 330)
	dqup = (1000,)*robotDim
	taskLim.referenceVelInf.value = tuple([-val*pi/180 for val in dqup])
	taskLim.referenceVelSup.value = tuple([ val*pi/180 for val in dqup])

	taskLim.controlGain.value = 1



	
	### Task references ------------------------------------------------
	robot.dynamic.com.recompute(0)
	taskCom.featureDes.errorIN.value = robot.dynamic.com.value
	taskCom.task.controlGain.value = 10

	taskCom.feature.selec.value = '011'
	taskCom.gain.setConstant(0.1)

	taskPosture.ref = pose


	# Set the target for RH and LH task. Selec is the activation flag (say control only
	# the XYZ translation), and gain is the adaptive gain (10 at the target, 0.1
	# far from it, with slope st. at 0.01m from the target, 90% of the max gain
	# value is reached
	displacementMatrix=eye(4); displacementMatrix[0:3,3] = array([0.1,0.,0.])

	taskRH.feature.position.recompute(0)
	taskLH.feature.position.recompute(0)
	targetRH = vectorToTuple(array(matrixToRPY( dot(displacementMatrix,array(taskRH.feature.position.value)) )))
	gotoNd(taskRH, targetRH, "111111",(50,1,0.01,0.9))

	gotoNdRel(taskRel,taskRH.feature.position.value,taskLH.feature.position.value,'110111',(50,1,0.01,0.9))
	taskRel.feature.errordot.value=(0,0,0,0,0)	# not to forget!!





#-----------------------------------------------------------------------------
# ---- CONTACT: Contact definition -------------------------------------------
#-----------------------------------------------------------------------------
def createContact(robot):

	# Left foot contact
	contactLF = MetaTaskDyn6d('contact_lleg',robot.dynamic,'lf','left-ankle')
	contactLF.featureDes.velocity.value=(0,0,0,0,0,0)
	contactLF.feature.frame('desired')
	contactLF.name = "LF"

	# Right foot contact
	contactRF = MetaTaskDyn6d('contact_rleg',robot.dynamic,'rf','right-ankle')
	contactRF.featureDes.velocity.value=(0,0,0,0,0,0)
	contactRF.feature.frame('desired')
	contactRF.name = "RF"

	contactRF.support = ((0.11,-0.08,-0.08,0.11),(-0.045,-0.045,0.07,0.07),(-0.105,-0.105,-0.105,-0.105))
	contactLF.support = ((0.11,-0.08,-0.08,0.11),(-0.07,-0.07,0.045,0.045),(-0.105,-0.105,-0.105,-0.105))

	# Imposed erordot = 0
	contactLF.feature.errordot.value=(0,0,0,0,0,0)
	contactRF.feature.errordot.value=(0,0,0,0,0,0)


def moveTool(solver):

	# Sot
	sot.clear()
	sot.addContact(contactLF)
	sot.addContact(contactRF)
	sot.push(taskLim.name)
	sot.push(taskRel.task.name)
	sot.push(taskRH.task.name)
	sot.push(taskCom.task.name)
	sot.push(taskPosture.task.name)

print "pos_err= "+str(linalg.norm(array(taskRH.feature.error.value)[0:3]))



