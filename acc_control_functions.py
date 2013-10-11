# ______________________________________________________________________________
# ******************************************************************************
#
#    FUNCTIONS
#       Robot: HRP-2 N.14
#       Tasks: Functions that allow some basic behaviors
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

from dynamic_graph.sot.core.matrix_util import vectorToTuple, matrixToTuple, matrixToRPY, RPYToMatrix
from dynamic_graph.sot.core import FeatureGeneric, FeaturePoint6d
from dynamic_graph.sot.core.feature_vector3 import *
from dynamic_graph.sot.core.math_small_entities import Multiply_of_matrixHomo, Selec_column_of_matrix, Selec_of_matrix, Multiply_matrix_vector

from dynamic_graph.sot.dyninv.meta_task_dyn_6d import MetaTaskDyn6d
from dynamic_graph.sot.dyninv.meta_tasks_dyn import gotoNd
from dynamic_graph.sot.dyninv import TaskDynInequality
from dynamic_graph.sot.dyninv.meta_tasks_dyn_relative import gotoNdRel, MetaTaskDyn6dRel

from numpy import eye, array, dot, pi, linalg, matrix, ndarray

from dynamic_graph.sot.screwing.utility import TwoHandToolToTriggerMatrix, TwoHandToolToSupportMatrix, TwoHandToolToScrewMatrix

from dynamic_graph.sot.screwing.vel_control_functions import createFeatureVec, createM2Pos, createVecMult




# ________________________________________________________________________
# ************************************************************************
#    REMOVE UNDESIRED TASKS FROM SOT
#       Tasks: Leaves in the SoT only the basic tasks, removing the others.
#           Needed when creating a task without knowing the SoT composition
# ________________________________________________________________________
# ************************************************************************
def removeUndesiredTasks(solver):

    to_keep = [ 'taskLim', 'taskcontactLF', 'taskcontactRF', 'taskcom', 'taskposture']

    for taskname in solver.toList():
        if not taskname in to_keep:
            solver.sot.rm(taskname)



# ________________________________________________________________________
# ************************************************************************
#    CLOSE GRIPPERS
#       Tasks: Close the grippers till the position
#              defined in the half-sitting pose
# ________________________________________________________________________
# ************************************************************************
def closeGrippers(robot,solver):

    # Pose definition
    pose = array(robot.halfSitting)
    #pose[28] = 0.1
    #pose[35] = 0.1

    # New taskPosture creation
    robot.mTasks['posture'].ref = vectorToTuple(pose)
    robot.mTasks['posture'].gain.setConstant(10)


# ________________________________________________________________________
# ************************************************************************
#    OPEN GRIPPERS
#       Robot: HRP-2 N.14
#       Tasks: Open the grippers completely
# ________________________________________________________________________
# ************************************************************************

def openGrippers(robot,solver):

    # Pose definition
    pose = array(robot.halfSitting)
    pose[28] = 0.74
    pose[35] = 0.74

    # New taskPosture creation
    robot.mTasks['posture'].ref = vectorToTuple(pose)
    robot.mTasks['posture'].gain.setConstant(10)



# ************************************************************************
#    MOVE HEAD TO A CERTAIN ANGLE
#       Robot: HRP-2 N.14
#       Tasks: move the head
# ________________________________________________________________________
# ************************************************************************

def moveHead(robot,solver,angle):

    # Pose definition
    pose = array(robot.halfSitting)
    pose[20] = angle

    # New taskPosture creation
    robot.mTasks['posture'].ref = vectorToTuple(pose)
    robot.mTasks['posture'].gain.setConstant(10)




# ________________________________________________________________________
# ************************************************************************
#    GO TO HALF SITTING POSE
#       Tasks: Go to the half-sitting pose
# ________________________________________________________________________
# ************************************************************************
def goToHalfSitting(robot,solver,gain):

    # Remove Other Tasks
    removeUndesiredTasks(solver)

    # Pose definition
    pose = array(robot.halfSitting)

    # New taskPosture creation
    robot.mTasks['posture'].ref = vectorToTuple(pose)
    robot.mTasks['posture'].gain.setConstant(gain)



# ________________________________________________________________________
# ************************************************************************
#	MOVE RIGHT HAND TO TARGET
#          Task: The robot moves the han to a given position
# ________________________________________________________________________
# ************************************************************************
def moveRightHandToTarget(robot,solver,target,gainMax):

	############################################################
	# Reference and gain setting
	############################################################

	gotoNd(robot.mTasks['rh'],target,'000111',(gainMax,gainMax/50.0,0.01,0.9))

	############################################################
	# Push
	############################################################
	
	removeUndesiredTasks(solver)

	solver.push(robot.mTasks['rh'].task)




# ________________________________________________________________________
# ************************************************************************
#	FOLLOW A POINT IN SPACE
#          Task: The robot moves the head to follow a point in space
#          Remark: The position has to be tuple of dim 3 or a signal
# ________________________________________________________________________
# ************************************************************************

def follow3DPoint(robot,solver,target,gain):

    ############################################################
    # Reference and gain setting
    ############################################################

    if isinstance (target,tuple):
        robot.mTasks['gaze'].goto3D(target,(gain,gain/5.0,0.01,0.9))
    else: #target is a signal
        plug(target,robot.mTasks['gaze'].proj.point3D)
        robot.mTasks['gaze'].gain.setByPoint(gain,gain/5.0,0.01,0.9)
    
    ############################################################
    # Push
    ############################################################
    
    solver.push(robot.mTasks['gaze'].task)


# ________________________________________________________________________
# ************************************************************************
#	MOVE RIGHT AND LEFT HAND TO CATCH THE TOOL
#           Task: The robot moves the hands to the position where he can
#			catch the tool
# ________________________________________________________________________
# ************************************************************************
def get_2ht(robot,solver,TwoHandTool,gainMax,gainMin):
    #TwoHandTool = (0.4,-0.1,0.9,0.,0.,pi/2)
    #TwoHandTool = (xd,yd,zd,roll,pitch,yaw)
    
    # Homogeneous Matrix of the TwoHandTool. Normally given from the camera
    refToTwoHandToolMatrix = RPYToMatrix(TwoHandTool)

    # Homogeneous Matrixes
    refToTriggerMatrix=dot(refToTwoHandToolMatrix,TwoHandToolToTriggerMatrix)
    refToSupportMatrix=dot(refToTwoHandToolMatrix,TwoHandToolToSupportMatrix)


    # ---- TASKS DEFINITION -------------------------------------------------------------------
    
    # Set the targets. Selec is the activation flag (say control only
    # the XYZ translation), and gain is the adaptive gain (<arg1> at the target, <arg2>
    # far from it, with slope st. at <arg3>m from the target, <arg4>% of the max gain
    # value is reached
    target = vectorToTuple(refToSupportMatrix[0:3,3])
    gotoNd(robot.mTasks['rh'], target, "000111",(gainMax,gainMin,0.01,0.9))
    
    target = vectorToTuple(refToTriggerMatrix[0:3,3])
    gotoNd(robot.mTasks['lh'], target, "000111",(gainMax,gainMin,0.01,0.9))
    
    # Orientation RF and LF - Needed featureVector3 to get desired behaviour
    if not hasattr(robot.mTasks['rh'], 'featureVec'):
        createFeatureVec(robot,'rh',array([1.,0.,0.]))
    if not hasattr(robot.mTasks['lh'], 'featureVec'):
        createFeatureVec(robot,'lh',array([1.,0.,0.]))

    robot.mTasks['rh'].featureVec.positionRef.value = dot(refToTwoHandToolMatrix[0:3,0:3],array([1.,0.,0.]))
    robot.mTasks['lh'].featureVec.positionRef.value = dot(refToTwoHandToolMatrix[0:3,0:3],array([-1.,0.,0.]))
    
    tasks = array([robot.mTasks['rh'].task, robot.mTasks['lh'].task])


    # sot loading

    solver.sot.damping.value = 0.001

    removeUndesiredTasks(solver)

    for i in range(len(tasks)):
        solver.push(tasks[i])



# ________________________________________________________________________
# ************************************************************************
#	CREATE RELATIVE TASK
#           Function that creates a relative task.
# ________________________________________________________________________
# ************************************************************************
def createRelativeTask(robot):

    robot.mTasks['rel'] = MetaTaskDyn6dRel('rel',robot.dynamic,'rh','lh','right-wrist','left-wrist')

    robot.mTasks['rel'].feature.frame('desired')
    robot.mTasks['rel'].gain.setConstant(10)
    robot.mTasks['rel'].task.dt.value = robot.timeStep
    robot.mTasks['rel'].featureDes.velocity.value=(0,0,0,0,0,0)

    # RELATIVE POSITION TASK
    handMgrip=eye(4); handMgrip[0:3,3] = (0,0,-0.14)
    robot.mTasks['rel'].opmodif = matrixToTuple(handMgrip)
    robot.mTasks['rel'].opmodifBase = matrixToTuple(handMgrip)




# ________________________________________________________________________
# ************************************************************************
#	EXECUTE SCREWING OPERATION
#           Task: The robot takes the screw driver to the goal
# ________________________________________________________________________
# ************************************************************************

# This matrix has to be computed only before the first screw.
def createScrewTask(robot,TwoHandTool):

    refToTwoHandToolMatrix = array(RPYToMatrix(TwoHandTool))
    #RH-TwoHandTool Homogeneous Transformation Matrix (fixed in time)
    robot.mTasks['rh'].feature.position.recompute(robot.device.control.time)
    RHToTwoHandToolMatrix = dot(linalg.inv(array(robot.mTasks['rh'].feature.position.value)),refToTwoHandToolMatrix)
    #!!!!!! RH and Support are different references, because the X rotation is not controlled in positioning!!!!!!!!!!!!!!!!!!!!!!!!!!
    RHToScrewMatrix = dot(RHToTwoHandToolMatrix,TwoHandToolToScrewMatrix)

    # Screw Lenght - unused at the moment
    #screw_len = 0.03

    # TASK Screw
    robot.mTasks['screw']=MetaTaskDyn6d('screw',robot.dynamic,'screw','right-wrist')
    handMgrip = array( robot.mTasks['rh'].opmodif )
    robot.mTasks['screw'].opmodif = matrixToTuple(dot(handMgrip,RHToScrewMatrix))
    robot.mTasks['screw'].featureDes.velocity.value=(0,0,0,0,0,0)
    robot.mTasks['screw'].feature.selec.value = '000111'

    # TASK Screw orientation
    robot.mTasks['screw'].featureVec = FeatureVector3("featureVecScrew")
    plug(robot.mTasks['screw'].opPointModif.position,robot.mTasks['screw'].featureVec.signal('position'))
    plug(robot.mTasks['screw'].opPointModif.jacobian,robot.mTasks['screw'].featureVec.signal('Jq'))
    robot.mTasks['screw'].featureVec.vector.value = array([0.,0.,1.])
    robot.mTasks['screw'].task.add(robot.mTasks['screw'].featureVec.name)

def screw_2ht(robot,solver,tool,target,goal,gainMax,gainMin):
    #goal = array([0.5,-0.3,1.1,0.,1.57,0.])

    if 'rel' not in robot.mTasks: createRelativeTask(robot)
    if 'screw' not in robot.mTasks:
        createScrewTask(robot,tool)
        createVecMult(robot)
        if not hasattr(robot,'m2pos') : createM2Pos(robot)

    # Task Relative
    gotoNdRel(robot.mTasks['rel'],array(robot.mTasks['rh'].feature.position.value),array(robot.mTasks['lh'].feature.position.value),'110111',gainMax*2)
    robot.mTasks['rel'].feature.errordot.value=(0,0,0,0,0)	# not to forget!!

    # Aim setting
    if isinstance (goal,ndarray):
        if len(goal)==6:
            refToGoalMatrix = RPYToMatrix(goal)
        else:
            refToGoalMatrix = goal

        robot.mTasks['screw'].ref = matrixToTuple(refToGoalMatrix)
        robot.mTasks['screw'].featureVec.positionRef.value = dot(refToGoalMatrix[0:3,0:3],array([0.,0.,1.]))
    else: #goal is a signal
        plug(goal,robot.mTasks['screw'].featureDes.position)
        plug(goal,robot.selec.sin)
        robot.mult.sin2.value = (0.,0.,1.)
        plug(robot.mult.sout,robot.mTasks['screw'].featureVec.positionRef)

    if isinstance (target,ndarray):
        if len(target)==6:
            refToTargetMatrix = RPYToMatrix(target)
        else:
            refToTargetMatrix = target

        robot.mTasks['gaze'].proj.point3D.value=vectorToTuple(target[0:3,3])
    else: #target is a signal
        plug(target,robot.m2pos.sin)
        plug(robot.m2pos.sout,robot.mTasks['gaze'].proj.point3D)


    robot.mTasks['gaze'].gain.setByPoint(gainMax*2,gainMin*2,0.01,0.9)
    robot.mTasks['screw'].gain.setByPoint(gainMax,gainMin,0.01,0.9)
    

    tasks = array([robot.mTasks['rel'].task, robot.mTasks['gaze'].task, robot.mTasks['screw'].task])

    # sot charging
    if not ( ('taskrel' in solver.toList()) and ('taskgaze' in solver.toList()) and ('taskscrew' in solver.toList()) ):
        removeUndesiredTasks(solver)
        for i in range(len(tasks)):
            solver.push(tasks[i])
