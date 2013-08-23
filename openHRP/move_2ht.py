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

from numpy import eye, array, dot



#-----------------------------------------------------------------------------
# --- OPERATIONAL TASKS (For HRP2-14)-----------------------------------------
#-----------------------------------------------------------------------------

displacement = eye(4)
displacement[0:3,3] = array([0.1,0.,0.])

def createRelativeTask(robot):

    robot.mTasks['rel'] = MetaTaskDyn6dRel('rel',robot.dynamic,'rh','lh','right-wrist','left-wrist')

    robot.mTasks['rel'].feature.frame('current')
    robot.mTasks['rel'].gain.setConstant(10)
    robot.mTasks['rel'].task.dt.value = robot.timeStep
    robot.mTasks['rel'].featureDes.velocity.value=(0,0,0,0,0,0)

    # RELATIVE POSITION TASK
    handMgrip=eye(4); handMgrip[0:3,3] = (0,0,-0.14)
    robot.mTasks['rel'].opmodif = matrixToTuple(handMgrip)
    robot.mTasks['rel'].opmodifBase = matrixToTuple(handMgrip)


def removeUndesiredTasks(solver):

    to_keep = [ 'taskLim', 'taskcontact_lleg', 'taskcontact_rleg', 'taskcom', 'taskposture']

    for taskname in solver.toList():
        if not taskname in to_keep:
            solver.sot.rm(taskname)



def move_2ht(robot, solver, displacementMatrix):

    if 'rel' not in robot.mTasks: createRelativeTask(robot)


    # Set the targets. Selec is the activation flag (say control only
    # the XYZ translation), and gain is the adaptive gain (<arg1> at the target, <arg2>
    # far from it, with slope st. at <arg3>m from the target, <arg4>% of the max gain
    # value is reached
    robot.mTasks['rh'].feature.position.recompute(robot.device.control.time)
    robot.mTasks['lh'].feature.position.recompute(robot.device.control.time)
    targetLH = vectorToTuple(array(matrixToRPY( dot(displacementMatrix,array(robot.mTasks['lh'].feature.position.value)) )))
    gotoNd(robot.mTasks['lh'], targetLH, "111111",(50,1,0.01,0.9))

    gotoNdRel(robot.mTasks['rel'],robot.mTasks['rh'].feature.position.value,robot.mTasks['lh'].feature.position.value,'110111',(50,1,0.01,0.9))
    robot.mTasks['rel'].feature.errordot.value=(0,0,0,0,0)	# not to forget!!

    #Task Waist
    if 'taskWaistIne' not in robot.tasksIne :
        featureWaist = FeaturePoint6d('featureWaist')
	plug(robot.dynamic.waist,featureWaist.position)
	plug(robot.dynamic.Jwaist,featureWaist.Jq)
	robot.tasksIne['taskWaistIne']=TaskDynInequality('taskWaistIne')
	plug(robot.dynamic.velocity,robot.tasksIne['taskWaistIne'].qdot)
	robot.tasksIne['taskWaistIne'].add(featureWaist.name)

    robot.tasksIne['taskWaistIne'].selec.value = '110000'
    robot.tasksIne['taskWaistIne'].referenceInf.value = (0.,0.,0.,0.,-0.1,-0.7)    # Roll Pitch Yaw min
    robot.tasksIne['taskWaistIne'].referenceSup.value = (0.,0.,0.,0.,0.5,0.7)  # Roll Pitch Yaw max
    robot.tasksIne['taskWaistIne'].dt.value=robot.timeStep
    robot.tasksIne['taskWaistIne'].controlGain.value = 10

    tasks = array([robot.mTasks['rel'].task,robot.mTasks['lh'].task,robot.tasksIne['taskWaistIne']])


    # sot charging
    removeUndesiredTasks(solver)

    for i in range(len(tasks)):
        solver.push(tasks[i])



