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

from numpy import eye,array,dot,pi

from dynamic_graph.sot.fmorsill.openHRP.move_2ht import removeUndesiredTasks

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

def get_2ht(robot,solver,TwoHandTool):

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
    
    # Task Inequality
    featureHeight = FeatureGeneric('featureHeight')
    plug(robot.dynamic.com,featureHeight.errorIN)
    plug(robot.dynamic.Jcom,featureHeight.jacobianIN)
    robot.taskHeight=TaskDynInequality('taskHeight')
    plug(robot.dynamic.velocity,robot.taskHeight.qdot)
    robot.taskHeight.add(featureHeight.name)
    robot.taskHeight.selec.value = '100'
    robot.taskHeight.referenceInf.value = (0.,0.,0.)    # Xmin, Ymin
    robot.taskHeight.referenceSup.value = (0.,0.,0.83)  # Xmax, Ymax
    robot.taskHeight.dt.value=robot.timeStep

    # Set the targets. Selec is the activation flag (say control only
    # the XYZ translation), and gain is the adaptive gain (<arg1> at the target, <arg2>
    # far from it, with slope st. at <arg3>m from the target, <arg4>% of the max gain
    # value is reached
    target = vectorToTuple(refToSupportMatrix[0:3,3])
    gotoNd(robot.mTasks['rh'], target, "000111",(50,1,0.01,0.9))
    
    target = vectorToTuple(refToTriggerMatrix[0:3,3])
    gotoNd(robot.mTasks['lh'], target, "000111",(50,1,0.01,0.9))
    
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
    
    gotoNd(robot.mTasks['chest'],(0.,0.,0.,0.,0.,0.),'111000',(1.,))	# inside the function rot=0 --> we set a random position not to control it
    robot.mTasks['chest'].task.errorTimeDerivative.value = [0., 0., 0.]
    
    gotoNd(robot.mTasks['waist'],(0.,0.,0.,0.,0.,0.),'111000',(1.,))	# inside the function rot=0 --> we set a random position not to control it
    robot.mTasks['waist'].task.errorTimeDerivative.value = [0., 0., 0.]
    
    
    tasks = array([robot.mTasks['rh'].task, robot.mTasks['lh'].task, robot.mTasks['waist'].task, robot.mTasks['chest'].task, robot.taskHeight])


    # sot loading

    solver.sot.damping.value = 0.001

    removeUndesiredTasks(solver)

    for i in range(len(tasks)):
        solver.push(tasks[i])

