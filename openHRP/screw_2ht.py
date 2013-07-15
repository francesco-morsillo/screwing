# ______________________________________________________________________________
# ******************************************************************************
#
#    SCREWING EXAMPLE
#       Robot: HRP-2 N.14
#       Tasks: Simulation of a sequence of points screwing
# 
# ______________________________________________________________________________
# ******************************************************************************  

"""
Already loaded on the python interpreter:
	- robot with dynamics and control
	- solver to produce the control
	- dynamic graph

The needed part is strictly the task definition and the push in the stack of tasks.

This script is thought to be used with the tool in the hands.
"""



from dynamic_graph import plug
from dynamic_graph.sot.core import FeatureGeneric, FeaturePoint6d
from dynamic_graph.sot.core.feature_vector3 import *
from dynamic_graph.sot.dyninv import TaskDynInequality
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple, matrixToRPY, RPYToMatrix
from dynamic_graph.sot.dyninv.meta_task_dyn_6d import MetaTaskDyn6d
from dynamic_graph.sot.dyninv.meta_tasks_dyn_relative import gotoNdRel, MetaTaskDyn6dRel
from dynamic_graph.sot.application.acceleration.precomputed_tasks import createBalanceAndPosture

from dynamic_graph.sot.fmorsill.utility import TwoHandToolToTriggerMatrix, TwoHandToolToSupportMatrix, TwoHandToolToScrewMatrix

from numpy import eye, array, dot, linalg

from dynamic_graph.sot.fmorsill.openHRP.move_2ht import createRelativeTask, removeUndesiredTasks



#goal = array([0.55,-0.2,0.9,0.,1.57,0.])

def screw_2ht(robot,solver,TwoHandTool,goal):

    if 'rel' not in robot.mTasks: createRelativeTask(robot)


    # Screw Lenght
    screw_len = 0.03

    refToTwoHandToolMatrix = array(RPYToMatrix(TwoHandTool))

    #-----------------------------------------------------------------------------
    # --- OPERATIONAL TASKS (For HRP2-14)-----------------------------------------
    #-----------------------------------------------------------------------------

    #RH-TwoHandTool Homogeneous Transformation Matrix (fixed in time)
    robot.mTasks['rh'].feature.position.recompute(0)
    robot.mTasks['lh'].feature.position.recompute(0)
    RHToTwoHandToolMatrix = dot(linalg.inv(array(robot.mTasks['rh'].feature.position.value)),refToTwoHandToolMatrix)
    #!!!!!! RH and Support are different references, because the X rotation is not controlled in positioning!!!!!!!!!!!!!!!!!!!!!!!!!!
    RHToScrewMatrix = dot(RHToTwoHandToolMatrix,TwoHandToolToScrewMatrix)

    # TASK Screw
    robot.mTasks['screw']=MetaTaskDyn6d('screw',robot.dynamic,'screw','right-wrist')
    handMgrip = array( robot.mTasks['rh'].opmodif )
    robot.mTasks['screw'].opmodif = matrixToTuple(dot(handMgrip,RHToScrewMatrix))
    robot.mTasks['screw'].featureDes.velocity.value=(0,0,0,0,0,0)
    robot.mTasks['screw'].feature.selec.value = '000111'
    robot.mTasks['screw'].gain.setByPoint(100,1,0.01,0.9)

    # TASK Screw orientation
    featureVecScrew = FeatureVector3("featureVecScrew")
    plug(robot.dynamic.signal('rh'),featureVecScrew.signal('position'))
    plug(robot.dynamic.signal('Jrh'),featureVecScrew.signal('Jq'))
    featureVecScrew.vector.value = array([0.,0.,-1.])
    robot.mTasks['screw'].task.add(featureVecScrew.name)

    # Task Relative
    gotoNdRel(robot.mTasks['rel'],array(robot.mTasks['rh'].feature.position.value),array(robot.mTasks['lh'].feature.position.value),'110111',500)
    robot.mTasks['rel'].feature.errordot.value=(0,0,0,0,0)	# not to forget!!




    ######################################################################
    ###------TASK INEQUALITY----------------------------------------------
    ######################################################################

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
    

    #Task Chest
    if 'taskChestIne' not in robot.tasksIne :
        featureChest = FeaturePoint6d('featureChest')
        plug(robot.dynamic.waist,featureChest.position)
        plug(robot.dynamic.Jwaist,featureChest.Jq)
        robot.tasksIne['taskChestIne']=TaskDynInequality('taskChestIne')
        plug(robot.dynamic.velocity,robot.tasksIne['taskChestIne'].qdot)
        robot.tasksIne['taskChestIne'].add(featureChest.name)

    robot.tasksIne['taskChestIne'].selec.value = '110000'
    robot.tasksIne['taskChestIne'].referenceInf.value = (0.,0.,0.,0.,-0.2,-0.7)    # Roll Pitch Yaw min
    robot.tasksIne['taskChestIne'].referenceSup.value = (0.,0.,0.,0.,0.3,0.7)  # Roll Pitch Yaw max
    robot.tasksIne['taskChestIne'].dt.value=robot.timeStep
    robot.tasksIne['taskChestIne'].controlGain.value = 10


    # Goal HM
    refToGoalMatrix = RPYToMatrix(goal)

    # Aim setting
    robot.mTasks['screw'].ref = matrixToTuple(refToGoalMatrix)
    featureVecScrew.positionRef.value = dot(refToGoalMatrix[0:3,0:3],array([0.,0.,1.]))

    tasks = array([robot.mTasks['rel'].task,robot.mTasks['screw'].task,robot.tasksIne['taskWaistIne'],robot.tasksIne['taskChestIne']])


    # sot charging
    solver.sot.damping.value = 0.001

    removeUndesiredTasks(solver)

    for i in range(len(tasks)):
        solver.push(tasks[i])
