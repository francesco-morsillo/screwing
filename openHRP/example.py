
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

from dynamic_graph.sot.screwing.openHRP.move_2ht import removeUndesiredTasks


# --- TRACER -----------------------------------------------------------------
from dynamic_graph.tracer import *

def traceStuff(robot):
	tr = Tracer('tr')
	tr.open('/tmp/','','.dat')
	tr.start()
	robot.after.addSignal('tr.triger')
	robot.after.addSignal(robot.taskLim.name+".normalizedPosition")

	tr.add('robot.dynamic.com','com')
	tr.add(robot.mTasks['rh'].task.name+'.error','erh')
	tr.add(robot.taskLim.name+".normalizedPosition","qn")
	tr.add("robot.device.state","q")
	tr.add("robot.device.control","qdot")
	





def moveRightHandToTarget(robot,solver,target):

	############################################################
	# Reference and gain setting
	############################################################

	gotoNd(robot.mTasks['rh'],target,'000111',(50,1,0.01,0.9))

	############################################################
	# Push
	############################################################
	
	removeUndesiredTasks(solver)

	solver.push(robot.mTasks['rh'].task)


