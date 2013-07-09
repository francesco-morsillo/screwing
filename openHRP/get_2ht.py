# ______________________________________________________________________________
# ******************************************************************************
#
#    GET USING-TOOL POSITION
#       Robot: HRP-2 N.14
#       Tasks: Reach the "using-tool position" from half-sitting position
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
from dynamic_graph.tracer import *
from dynamic_graph.sot.core import FeatureGeneric
from dynamic_graph.sot.core.feature_vector3 import *
from dynamic_graph.sot.dyninv import TaskDynInequality
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple, matrixToRPY, RPYToMatrix
from dynamic_graph.sot.dyninv.meta_task_dyn_6d import MetaTaskDyn6d
from dynamic_graph.sot.dyninv.meta_tasks_dyn import gotoNd

from dynamic_graph.sot.fmorsill.utility import TwoHandToolToTriggerMatrix, TwoHandToolToSupportMatrix

from numpy import *


#-----------------------------------------------------------------------------
# --- TRACER ------------------------------------------------------------------
#-----------------------------------------------------------------------------

def createTraces(robot):

    tr = Tracer('tr')
    tr.open('/tmp/','OH_','.dat')
    tr.start()
    robot.device.after.addSignal('tr.triger')

    tr.add('robot.device.state','qn')

    robot.device.after.addSignal('tr.triger')
    robot.device.after.addSignal('robot.device.state')


#-----------------------------------------------------------------------------
# --- OPERATIONAL TASKS (For HRP2-14)-----------------------------------------
#-----------------------------------------------------------------------------

def createTasks(robot,TwoHandTool):

    # ---- STOOL PARAMETERS -------------------------------------------------------------------
    """
    # TwoHandTool
    xd = 0.4
    yd = -0.2
    zd = 0.9
    roll = 0.
    pitch = 0.
    yaw = pi/2
    """

    #TwoHandTool = (xd,yd,zd,roll,pitch,yaw)
    
    # Homogeneous Matrix of the TwoHandTool. Normally given from the camera
    refToTwoHandToolMatrix = RPYToMatrix(TwoHandTool)

    # Homogeneous Matrixes
    refToTriggerMatrix=dot(refToTwoHandToolMatrix,TwoHandToolToTriggerMatrix)
    refToSupportMatrix=dot(refToTwoHandToolMatrix,TwoHandToolToSupportMatrix)


    # ---- TASKS DEFINITION -------------------------------------------------------------------
    taskWaist = MetaTaskDyn6d('taskWaist', robot.dynamic, 'waist', 'waist')
    taskChest = MetaTaskDyn6d('taskChest', robot.dynamic, 'chest', 'chest')
    taskRH    = MetaTaskDyn6d('rh', robot.dynamic, 'rh', 'right-wrist')
    taskLH    = MetaTaskDyn6d('lh', robot.dynamic, 'lh', 'left-wrist')
    
    for task in [ taskWaist, taskChest, taskRH, taskLH]:
        task.feature.frame('desired')
        task.gain.setConstant(10)
        task.task.dt.value = robot.timeStep
        task.featureDes.velocity.value=(0,0,0,0,0,0)
    
    
    handMgrip=eye(4); handMgrip[0:3,3] = (0,0,-0.14)
    taskRH.opmodif = matrixToTuple(handMgrip)
    taskLH.opmodif = matrixToTuple(handMgrip)
    
    
    # Task Inequality
    featureHeight = FeatureGeneric('featureHeight')
    plug(robot.dynamic.com,featureHeight.errorIN)
    plug(robot.dynamic.Jcom,featureHeight.jacobianIN)
    taskHeight=TaskDynInequality('taskHeight')
    plug(robot.dynamic.velocity,taskHeight.qdot)
    taskHeight.add(featureHeight.name)
    taskHeight.selec.value = '100'
    taskHeight.referenceInf.value = (0.,0.,0.)    # Xmin, Ymin
    taskHeight.referenceSup.value = (0.,0.,0.83)  # Xmax, Ymax
    taskHeight.dt.value=robot.timeStep

    # Set the targets. Selec is the activation flag (say control only
    # the XYZ translation), and gain is the adaptive gain (<arg1> at the target, <arg2>
    # far from it, with slope st. at <arg3>m from the target, <arg4>% of the max gain
    # value is reached
    target = vectorToTuple(refToSupportMatrix[0:3,3])
    gotoNd(taskRH, target, "000111",(50,1,0.01,0.9))
    
    target = vectorToTuple(refToTriggerMatrix[0:3,3])
    gotoNd(taskLH, target, "000111",(50,1,0.01,0.9))
    
    # Orientation RF and LF - Needed featureVector3 to get desired behaviour
    featureVecRH = FeatureVector3("featureVecRH")
    plug(robot.dynamic.signal('rh'),featureVecRH.signal('position'))
    plug(robot.dynamic.signal('Jrh'),featureVecRH.signal('Jq'))
    featureVecRH.vector.value = array([1.,0.,0.])
    featureVecRH.positionRef.value = dot(refToTwoHandToolMatrix[0:3,0:3],array([1.,0.,0.]))
    taskRH.task.add(featureVecRH.name)
    
    featureVecLH = FeatureVector3("featureVecLH")
    plug(robot.dynamic.signal('lh'),featureVecLH.signal('position'))
    plug(robot.dynamic.signal('Jlh'),featureVecLH.signal('Jq'))
    featureVecLH.vector.value = array([1.,0.,0.])
    featureVecLH.positionRef.value = dot(refToTwoHandToolMatrix[0:3,0:3],array([-1.,0.,0.]))
    taskLH.task.add(featureVecLH.name)
    
    gotoNd(taskChest,(0.,0.,0.,0.,0.,0.),'111000',(1.,))	# inside the function rot=0 --> we set a random position not to control it
    taskChest.task.errorTimeDerivative.value = [0., 0., 0.]
    
    gotoNd(taskWaist,(0.,0.,0.,0.,0.,0.),'111000',(1.,))	# inside the function rot=0 --> we set a random position not to control it
    taskWaist.task.errorTimeDerivative.value = [0., 0., 0.]
    
    
    tasks = array([taskRH.task, taskLH.task, taskWaist.task, taskChest.task, taskHeight])
    
    return tasks



def move(solver, tasks):
    
    solver.sot.damping.value = 0.001

    for i in range(size(tasks)):
        solver.push(tasks[i])

