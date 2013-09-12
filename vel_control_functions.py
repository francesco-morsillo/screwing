# ______________________________________________________________________________
# ******************************************************************************
#
#    VELOCITY CONTROL FUNCTIONS
#       Robot: HRP-2 N.14
#       Tasks: Functions that allow some basic behaviors in velocity control
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
function 'initialize' from sot.application.velocity.precomputed_meta_tasks.
This script creates and loads the following tasks:
       - Left foot and right foot (as a constraint, not as a task)
       - COM for balance
       - TaskLim to respect joint limits
       

"""

from dynamic_graph import plug

from dynamic_graph.sot.core.matrix_util import vectorToTuple, matrixToTuple, matrixToRPY, RPYToMatrix
from dynamic_graph.sot.core import FeatureGeneric, FeaturePoint6d
from dynamic_graph.sot.core.feature_vector3 import *

from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom
from dynamic_graph.sot.core.meta_tasks import gotoNd
from dynamic_graph.sot.dyninv import TaskInequality
from dynamic_graph.sot.core.meta_tasks_kine_relative import gotoNdRel, MetaTaskKine6dRel

from numpy import eye, array, dot, pi, linalg

from dynamic_graph.sot.screwing.utility import TwoHandToolToTriggerMatrix, TwoHandToolToSupportMatrix, TwoHandToolToScrewMatrix






# ________________________________________________________________________
# ************************************************************************
#    REMOVE UNDESIRED TASKS FROM SOT
#       Tasks: Leaves in the SoT only the basic tasks, removing the others.
#           Needed when creating a task without knowing the SoT composition
# ________________________________________________________________________
# ************************************************************************
def removeUndesiredTasks(solver):

    to_keep = [ 'taskLim', 'taskcontactLF', 'taskcontactRF', 'taskcom']

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

    robot.mTasks['posture'].gotoq(2,rhand=[0.1,],lhand=[0.1,])

    # Eventual taskPosture creation
    if 'taskposture' not in solver.toList():
        solver.push(robot.mTasks['posture'].task)


# ________________________________________________________________________
# ************************************************************************
#    OPEN GRIPPERS
#       Robot: HRP-2 N.14
#       Tasks: Open the grippers completely
# ________________________________________________________________________
# ************************************************************************

def openGrippers(robot,solver):

    robot.mTasks['posture'].gotoq(2,rhand=[0.74,],lhand=[0.74,])

    # Eventual taskPosture creation
    if 'taskposture' not in solver.toList():
        solver.push(robot.mTasks['posture'].task)


# ________________________________________________________________________
# ************************************************************************
#    GO TO HALF SITTING POSE
#       Tasks: Go to the half-sitting pose
# ________________________________________________________________________
# ************************************************************************
def goToHalfSitting(robot,solver):

    # Remove Other Tasks
    removeUndesiredTasks(solver)

    # Pose definition
    pose = array(robot.halfSitting)

    # Task posture reference
    robot.mTasks['posture'].gotoq(2,pose,chest=[],head=[],rleg=[],lleg=[],rarm=[],larm=[],rhand=[],lhand=[])

    # Eventual taskPosture creation
    if 'taskposture' not in solver.toList():
        solver.push(robot.mTasks['posture'].task)


# ________________________________________________________________________
# ************************************************************************
#	MOVE RIGHT HAND TO TARGET
#          Task: The robot moves the han to a given position
# ________________________________________________________________________
# ************************************************************************
def moveRightHandToTarget(robot,solver,target,gain):

	############################################################
	# Reference and gain setting
	############################################################

	gotoNd(robot.mTasks['rh'],target,'000111',(gain,gain/50.0,0.01,0.9))

	############################################################
	# Push
	############################################################
	
	removeUndesiredTasks(solver)

	solver.push(robot.mTasks['rh'].task)







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
    featureVecRH = FeatureVector3("featureVecRH")
    plug(robot.dynamic.signal('rh'),featureVecRH.signal('position'))
    plug(robot.dynamic.signal('Jrh'),featureVecRH.signal('Jq'))
    featureVecRH.vector.value = array([1.,0.,0.])
    featureVecRH.positionRef.value = dot(refToTwoHandToolMatrix[0:3,0:3],array([1.,0.,0.]))
    robot.mTasks['rh'].task.add(featureVecRH.name)
    
    featureVecLH = FeatureVector3("featureVecLH")
    plug(robot.dynamic.signal('lh'),featureVecLH.signal('position'))
    plug(robot.dynamic.signal('Jlh'),featureVecLH.signal('Jq'))
    featureVecLH.vector.value = array([1.,0.,0.])
    featureVecLH.positionRef.value = dot(refToTwoHandToolMatrix[0:3,0:3],array([-1.,0.,0.]))
    robot.mTasks['lh'].task.add(featureVecLH.name)

    robot.mTasks['posture'].gotoq(2,array(robot.halfSitting),chest=[], rknee=[],lknee=[])

    tasks = array([robot.mTasks['rh'].task, robot.mTasks['lh'].task,robot.mTasks['posture'].task])

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

    robot.mTasks['rel'] = MetaTaskKine6dRel('rel',robot.dynamic,'rh','lh','right-wrist','left-wrist')

    robot.mTasks['rel'].feature.frame('desired')
    robot.mTasks['rel'].gain.setConstant(10)
    #robot.mTasks['rel'].task.dt.value = robot.timeStep
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
def screw_2ht(robot,solver,TwoHandTool,goal,gainMax,gainMin):
    #goal = array([0.5,-0.3,1.1,0.,1.57,0.])
    gainMin = gainMax/5;

    if 'rel' not in robot.mTasks: createRelativeTask(robot)

    # Screw Lenght
    screw_len = 0.03

    refToTwoHandToolMatrix = array(RPYToMatrix(TwoHandTool))

    #RH-TwoHandTool Homogeneous Transformation Matrix (fixed in time)
    robot.mTasks['rh'].feature.position.recompute(0)
    robot.mTasks['lh'].feature.position.recompute(0)
    RHToTwoHandToolMatrix = dot(linalg.inv(array(robot.mTasks['rh'].feature.position.value)),refToTwoHandToolMatrix)
    #!!!!!! RH and Support are different references, because the X rotation is not controlled in positioning!!!!!!!!!!!!!!!!!!!!!!!!!!
    RHToScrewMatrix = dot(RHToTwoHandToolMatrix,TwoHandToolToScrewMatrix)

    # TASK Screw
    robot.mTasks['screw']=MetaTaskKine6d('screw',robot.dynamic,'screw','right-wrist')
    handMgrip = array( robot.mTasks['rh'].opmodif )
    robot.mTasks['screw'].opmodif = matrixToTuple(dot(handMgrip,RHToScrewMatrix))
    robot.mTasks['screw'].featureDes.velocity.value=(0,0,0,0,0,0)
    robot.mTasks['screw'].feature.selec.value = '000111'
    robot.mTasks['screw'].gain.setByPoint(gainMax,gainMin,0.01,0.9)

    # TASK Screw orientation
    featureVecScrew = FeatureVector3("featureVecScrew")
    plug(robot.dynamic.signal('rh'),featureVecScrew.signal('position'))
    plug(robot.dynamic.signal('Jrh'),featureVecScrew.signal('Jq'))
    featureVecScrew.vector.value = array([0.,0.,-1.])
    robot.mTasks['screw'].task.add(featureVecScrew.name)

    # Task Relative
    gotoNdRel(robot.mTasks['rel'],array(robot.mTasks['rh'].feature.position.value),array(robot.mTasks['lh'].feature.position.value),'110111',gainMax*2)
    robot.mTasks['rel'].feature.errordot.value=(0,0,0,0,0)	# not to forget!!


    robot.mTasks['posture'].gotoq(2,array(robot.halfSitting),chest=[], rknee=[],lknee=[])


    # Goal HM
    refToGoalMatrix = RPYToMatrix(goal)

    # Aim setting
    robot.mTasks['screw'].ref = matrixToTuple(refToGoalMatrix)
    featureVecScrew.positionRef.value = dot(refToGoalMatrix[0:3,0:3],array([0.,0.,1.]))

    tasks = array([robot.mTasks['rel'].task,robot.mTasks['screw'].task,robot.mTasks['posture'].task])

    # sot charging
    solver.sot.damping.value = 0.001

    removeUndesiredTasks(solver)

    for i in range(len(tasks)):
        solver.push(tasks[i])
