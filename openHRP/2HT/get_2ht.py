# ______________________________________________________________________________
# ******************************************************************************
# REACHIN A GIVEN POSITION.
# The robot (fixed in the origin) moves the hands to given positions that 
# represent the grasp points of the tool.
# ______________________________________________________________________________
# ******************************************************************************

"""
Already loaded on the python interpreter:
	- robot with dynamics and control
	- solver to produce the control
	- dynamic graph

The needed part is strictly the task definition and the push in the stack of tasks.
"""


from dynamic_graph import plug

from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY
from dynamic_graph.sot.core.meta_task_6d import MetaTask6d,toFlags
from dynamic_graph.sot.core.meta_tasks import setGain
from dynamic_graph.sot.core.meta_tasks_kine import *
from dynamic_graph.sot.core.meta_task_posture import MetaTaskKinePosture
from numpy import *

from dynamic_graph.sot.fmorsill.utility import *



# ---- SCREW-DRIVER PARAMETERS -------------------------------------------------------------------
# ---- SCREW-DRIVER PARAMETERS -------------------------------------------------------------------
# ---- SCREW-DRIVER PARAMETERS -------------------------------------------------------------------

# TwoHandTool
xd = 0.4
yd = -0.2
zd = 1.
roll = 0.
pitch = pi/5
yaw = pi/2
TwoHandTool = (xd,yd,zd,roll,pitch,yaw)


# ---- FUNCTION ---------------------------------------------------------------------------------

def get_tool(robot,solver,tool6d):


    # Homogeneous Matrixes
    refToTwoHandToolMatrix = RPYToMatrix(tool6d)
    refToTriggerMatrix=dot(refToTwoHandToolMatrix,TwoHandToolToTriggerMatrix)
    refToSupportMatrix=dot(refToTwoHandToolMatrix,TwoHandToolToSupportMatrix)



    # ---- TASK HANDS PLACEMENT

    # RIGHT HAND
    taskRH=MetaTaskKine6d('rh',robot.dynamic,'rh','right-wrist')
    handMgrip=eye(4); handMgrip[0:3,3] = (0,0,-0.14)
    taskRH.opmodif = matrixToTuple(handMgrip)
    taskRH.feature.frame('desired')
    # LEFT HAND
    taskLH=MetaTaskKine6d('lh',robot.dynamic,'lh','left-wrist')
    taskLH.opmodif = matrixToTuple(handMgrip)
    taskLH.feature.frame('desired')


    taskRH.ref = matrixToTuple(refToSupportMatrix)
    taskRH.feature.selec.value = '110111'	# RX free
    taskRH.gain.setByPoint(30,0.3,0.01,0.9)

    taskLH.ref = matrixToTuple(refToTriggerMatrix)
    taskLH.feature.selec.value = '110111'	# RX free
    taskLH.gain.setByPoint(30,0.3,0.01,0.9)
    


    # --- POSTURE ---
    taskPosture = MetaTaskKinePosture(robot.dynamic)
    taskPosture.ref = halfSittingConfig[robotName]



    # --- JOINT LIMITS
    robot.dynamic.upperJl.recompute(0)
    robot.dynamic.lowerJl.recompute(0)
    taskJL = TaskJointLimits('taskJL')
    plug(robot.dynamic.position,taskJL.position)
    taskJL.controlGain.value = 10
    taskJL.referenceInf.value = robot.dynamic.lowerJl.value
    taskJL.referenceSup.value = robot.dynamic.upperJl.value
    taskJL.dt.value = dt
    taskJL.selec.value = toFlags(range(6,22)+range(22,28)+range(29,35))

    """
    # --- CONTACTS
    contactLF = MetaTaskKine6d('contactLF',robot.dynamic,'LF','left-ankle')
    contactLF.feature.frame('desired')
    contactLF.gain.setConstant(10)
    contactRF = MetaTaskKine6d('contactRF',robot.dynamic,'RF','right-ankle')
    contactRF.feature.frame('desired')
    contactRF.gain.setConstant(10)
    """

    # ---- WAIST TASK ---
    taskWaist=MetaTask6d('waist',robot.dynamic,'waist','waist')
    gotoNd(taskWaist,(0.,0.,0.),'011000',(10,0.9,0.01,0.9))	# inside the function rot=0 --> we set a random position not to control it


    # ---- COM ----
    robot.dynamic.com.recompute(0)
    taskCom.ref = robot.dynamic.com.value
    taskCom.feature.selec.value = '011'



    # Set up the stack solver.
    #solver.addContact(contactLF)
    #solver.addContact(contactRF)
    solver.push(taskJL.task.name)
    solver.push(taskWaist.task.name)
    solver.push(taskRH.task.name)
    solver.push(taskLH.task.name)
    solver.push(taskPosture.task.name)

