# ______________________________________________________________________________
# ******************************************************************************
# FIRST EXAMPLE OF REAL USAGE OF THE TOOL.
# Given a stack of aims (holes with specified z-orientation) the robot put the
# screw driver in front of the hole and executes the screwing operation. To 
# complete the goal changing
# ______________________________________________________________________________
# ******************************************************************************

from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Matrix
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.dyninv import *
import dynamic_graph.script_shortcuts
from dynamic_graph.script_shortcuts import optionalparentheses
from dynamic_graph.matlab import matlab
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY
from dynamic_graph.sot.core.meta_task_6d import MetaTask6d,toFlags
from dynamic_graph.sot.core.meta_tasks import setGain
from dynamic_graph.sot.core.meta_tasks_kine import *
from dynamic_graph.sot.core.meta_task_posture import MetaTaskKinePosture
from dynamic_graph.sot.core.meta_task_visual_point import MetaTaskVisualPoint
from dynamic_graph.sot.core.utils.viewer_helper import addRobotViewer,VisualPinger,updateComDisplay
from dynamic_graph.sot.core.utils.attime import attime,ALWAYS,refset,sigset
from numpy import *

from dynamic_graph.sot.core.utils.history import History

from dynamic_graph.sot.dyninv.robot_specific import pkgDataRootDir,modelName,robotDimension,initialConfig,gearRatio,inertiaRotor

from dynamic_graph.sot.fmorsill.utility import *

# --- ROBOT AND SCREWTOOL SIMU ---------------------------------------------------------------
# --- ROBOT AND SCREWTOOL SIMU ---------------------------------------------------------------
# --- ROBOT AND SCREWTOOL SIMU ---------------------------------------------------------------

robotName = 'hrp14small'
robotDim=robotDimension[robotName]
robot = RobotSimu("robot")
robot.resize(robotDim)
dt=5e-3


# Robot and TwoHandTool in the origin
xr=0.
yr=0.
zr=0.64870185118253043
rollr=0.
pitchr=0.
yawr=0.

xTool = 0.4
yTool = -0.2
zTool = 1.1
rollTool = 0.
pitchTool = pi/5
yawTool = pi/2


# Initial configuration
# We give the absolute position of the robot and then extract the relative rototranslation to impose to the tool
from dynamic_graph.sot.dyninv.robot_specific import halfSittingConfig
x0=0#-1.6
y0=0.#1.8
z0=0.64870185118253043
roll0=0.
pitch0=0.
yaw0=0.#pi/2

"""
halfSittingConfig[robotName] = (x0,y0,z0,0,0,yaw0)+halfSittingConfig[robotName][6:]
initialConfig[robotName]=halfSittingConfig[robotName]
robot.set( initialConfig[robotName] )
"""

"""
# Wall initial configuration
robot.set()"""

# Center initial configuration
#for hrp14
robot.set((-0.0321239,0.0295696,0.689348,-2.73571e-19,-2.4299e-19,-0.000464104,0.000464104,-0.0504835,-0.268343,0.395009,-0.126666,0.0504835,0.000464104,-0.0506361,-0.296723,0.451101,-0.154379,0.0506361,-0.0241758,-0.0871973,-1.82436e-17,-3.09881e-18,-0.191161,-0.765804,0.554113,-1.83012,1.50936,0.393544,0.174533,-0.293434,-0.155129,-0.820303,-1.3099,-0.619151,0.740306,0.174533))

# Screw Lenght
screw_len = 0.

# TwoHandTool RotoTranslation / Homogeneous Matrix of the TwoHandTool. Normally given from the camera
RTMatrix = eye(4); RTMatrix[0:3,0:3] = calcRotFromRPY(roll0-rollr,pitch0-pitchr,yaw0-yawr)
RTMatrix[0:3,3] = (x0-xr,y0-yr,z0-zr)
TwoHandToolPos = dot(RTMatrix,array([xTool,yTool,zTool,1]))	# HOMOGENEOUS POSITION!! CUT OFF THE 1 TO USE IT :-)
TwoHandToolRot = dot(RTMatrix[0:3,0:3],calcRotFromRPY(rollTool,pitchTool,yawTool))


# goals
goal1 = array([0.5,-0.2,1.35,0.,1.57,0.])
goal2 = array([0.5,-0.3,1.35,0.,1.57,0.])
goal3 = array([0.5,-0.3,1.25,0.,1.57,0.])
goal4 = array([0.5,-0.2,1.25,0.,1.57,0.])
goal = array([goal1,goal2,goal3,goal4])

"""
goal1 = (-1.5,2.4,1.3,-1.57,0.,0.)
goal2 = (-1.7,2.4,1.3,-1.57,0.,0.)
goal3 = (-1.7,2.4,1.1,-1.57,0.,0.)
goal4 = (-1.5,2.4,1.1,-1.57,0.,0.)
"""


# visualization
addRobotViewer(robot,small=True,verbose=True)
robot.viewer.updateElementConfig('fov',[0,0,-10,3.14,0,0])
robot.viewer.updateElementConfig('TwoHandTool',vectorToTuple(TwoHandToolPos[0:3])+vectorToTuple(extractRPYFromRot(TwoHandToolRot)))
for i in range(4):
	robot.viewer.updateElementConfig('goal'+str(i+1),vectorToTuple(goal[i]))


#-----------------------------------------------------------------------------
#---- DYN --------------------------------------------------------------------
#-----------------------------------------------------------------------------

# Create the entity that computes the direct geometrics, kinematics and
# dynamics model functions.

modelDir  = pkgDataRootDir[robotName]
xmlDir    = pkgDataRootDir[robotName]
specificitiesPath = xmlDir + '/HRP2SpecificitiesSmall.xml'
jointRankPath     = xmlDir + '/HRP2LinkJointRankSmall.xml'

dyn = Dynamic("dyn")
dyn.setFiles(modelDir, modelName[robotName],specificitiesPath,jointRankPath)
dyn.parse()

dyn.inertiaRotor.value = inertiaRotor[robotName]
dyn.gearRatio.value    = gearRatio[robotName]

plug(robot.state,dyn.position)
dyn.velocity.value = robotDim*(0.,)
dyn.acceleration.value = robotDim*(0.,)

dyn.ffposition.unplug()
dyn.ffvelocity.unplug()
dyn.ffacceleration.unplug()

dyn.setProperty('ComputeBackwardDynamics','true')
dyn.setProperty('ComputeAccelerationCoM','true')

robot.control.unplug()

# ---- SOT ---------------------------------------------------------------------
# ---- SOT ---------------------------------------------------------------------
# ---- SOT ---------------------------------------------------------------------

# Create the inverse kinematics solver.
# The solver SOTH of dyninv is used, but the SOT solver should be sufficient.
sot = SolverKine('sot')
sot.setSize(robotDim)
plug(sot.control,robot.control)


# ---- TASKS -------------------------------------------------------------------
# ---- TASKS -------------------------------------------------------------------
# ---- TASKS -------------------------------------------------------------------

# ---- TASK HANDS PLACEMENT
# RIGHT HAND
taskRH=MetaTaskKine6d('rh',dyn,'rh','right-wrist')
handMgrip=eye(4); handMgrip[0:3,3] = (0,0,-0.17)
taskRH.opmodif = matrixToTuple(handMgrip)
taskRH.feature.frame('desired')
# LEFT HAND
taskLH=MetaTaskKine6d('lh',dyn,'lh','left-wrist')
taskLH.opmodif = matrixToTuple(handMgrip)
taskLH.feature.frame('desired')

# --- POSTURE ---
taskPosture = MetaTaskKinePosture(dyn)

# --- GAZE ---
taskGaze = MetaTaskVisualPoint('gaze',dyn,'head','gaze')
# Head to camera matrix transform
headMcam=array([[0.0,0.0,1.0,0.081],[1.,0.0,0.0,0.072],[0.0,1.,0.0,0.031],[0.0,0.0,0.0,1.0]])
headMcam = dot(headMcam,rotate('x',10*pi/180))
taskGaze.opmodif = matrixToTuple(headMcam)

# --- JOINT LIMITS
dyn.upperJl.recompute(0)
dyn.lowerJl.recompute(0)
taskJL = TaskJointLimits('taskJL')
plug(dyn.position,taskJL.position)
taskJL.controlGain.value = 10
taskJL.referenceInf.value = dyn.lowerJl.value
taskJL.referenceSup.value = dyn.upperJl.value
taskJL.dt.value = dt
taskJL.selec.value = toFlags(range(6,22)+range(22,28)+range(29,35))

# --- CONTACTS
contactLF = MetaTaskKine6d('contactLF',dyn,'LF','left-ankle')
contactLF.feature.frame('desired')
contactLF.gain.setConstant(10)
contactRF = MetaTaskKine6d('contactRF',dyn,'RF','right-ankle')
contactRF.feature.frame('desired')
contactRF.gain.setConstant(10)

# --- TASK SUPPORT SMALL --------------------------------------------
featureSupportSmall = FeatureGeneric('featureSupportSmall')
plug(dyn.com,featureSupportSmall.errorIN)
plug(dyn.Jcom,featureSupportSmall.jacobianIN)

taskSupportSmall=TaskInequality('taskSupportSmall')
taskSupportSmall.add(featureSupportSmall.name)
taskSupportSmall.selec.value = '011'
taskSupportSmall.dt.value=dt

# ---- WAIST TASK ---
taskWaist=MetaTask6d('waist',dyn,'waist','waist')
gotoNd(taskWaist,(0.,0.,0.),'011000',(10,0.9,0.01,0.9))	# inside the function rot=0 --> we set a random position not to control it

# --- TASK COM (if not walking)
taskCom = MetaTaskKineCom(dyn)


# TwoHandTool Moving
def updateToolDisplay(taskrh):
	tool = dot( array(taskrh.feature.position.value) , linalg.inv(TwoHandToolToSupportMatrix) )
	robot.viewer.updateElementConfig('TwoHandTool',vectorToTuple(tool[0:3,3])+(rollTool,pitchTool,yawTool))

# --- OTHER CONFIGS ----------------------------------------------------------
# --- OTHER CONFIGS ----------------------------------------------------------
# --- OTHER CONFIGS ----------------------------------------------------------


# --- TRACER -----------------------------------------------------------------
# Record some signals in the /tmp directory. Use the octave script p.m to plot
# them.
# p('com',1:2)   % Plot the X and Y components of the COM file (dyn.com signal).

from dynamic_graph.tracer import *
tr = Tracer('tr')
tr.open('/tmp/','','.dat')
tr.start()
robot.after.addSignal('tr.triger')

tr.add('dyn.com','com')
tr.add(taskJL.name+".normalizedPosition","qn")
robot.after.addSignal(taskJL.name+".normalizedPosition")
tr.add(taskRH.task.name+'.error','erh')

# --- SHORTCUTS ----------------------------------------------------------------
def push(task):
    '''Add a task at the least priority of the stack.'''
    if isinstance(task,str): taskName=task
    elif "task" in task.__dict__:  taskName=task.task.name
    else: taskName=task.name
    if taskName not in sot.toList():
        sot.push(taskName)
        if taskName!="taskposture" and "taskposture" in sot.toList():
            sot.down("taskposture")

def pop(task):
    '''Remove the least-priority task from the stack.'''
    if isinstance(task,str): taskName=task
    elif "task" in task.__dict__:  taskName=task.task.name
    else: taskName=task.name
    if taskName in sot.toList(): sot.rm(taskName)

def toList(sot):
    '''Display the stack as a list of task names.'''
    return map(lambda x: x[1:-1],sot.dispStack().split('|')[1:])
SolverKine.toList = toList

# --- RUN ----------------------------------------------------------------------
# --- RUN ----------------------------------------------------------------------
# --- RUN ----------------------------------------------------------------------

updateComDisplay(robot,dyn.com)

# Foot position recomputing
dyn.LF.recompute(0)
LFMatH = array(dyn.LF.value)
LFx = LFMatH[0][3]
LFy = LFMatH[1][3]
#LFz = 0.105
dyn.RF.recompute(0)
RFMatH = array(dyn.RF.value)
RFx = RFMatH[0][3]
RFy = RFMatH[1][3]
#RFz = 0.105

center = array([(LFx+RFx)/2,(LFy+RFy)/2,0.])	# z not important

# Set the aims
contactLF.ref = matrixToTuple(LFMatH)
contactRF.ref = matrixToTuple(RFMatH)
dyn.com.recompute(0)
COM_REF = array(dyn.com.value)
COM_REF[2] = 0.
taskCom.ref=vectorToTuple(COM_REF)
taskCom.feature.selec.value= '111011' # z not controlled --> the robot can go up and down

# Set up the stack solver.
sot.addContact(contactLF)
sot.addContact(contactRF)
push(taskJL)
push(taskCom)
push(taskRH)
push(taskLH)
push(taskWaist)


# Static task options
taskRH.feature.selec.value = '111111'	# RX no more free with the tool
taskRH.gain.setByPoint(10,0.1,0.01,0.9)
taskLH.feature.selec.value = '110111'	# RX free
taskLH.gain.setByPoint(10,0.1,0.01,0.9)


#RH-TwoHandTool Homogeneous Transformation Matrix (fixed in time)
taskRH.feature.position.recompute(0)
refToTwoHandToolMatrix = eye(4); refToTwoHandToolMatrix[0:3,0:3] = TwoHandToolRot; refToTwoHandToolMatrix[0:3,3] = TwoHandToolPos[0:3]
RHToTwoHandToolMatrix = dot(linalg.inv(array(taskRH.feature.position.value)),refToTwoHandToolMatrix)
#!!!!!! RH and Support are different references, because the X rotation is not controlled!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# The hands have to cover the same displacement, but in a different reference
RHToScrewMatrix = dot(RHToTwoHandToolMatrix,TwoHandToolToScrewMatrix)



def do():
	robot.increment(dt)
	attime.run(robot.control.time)
	updateComDisplay(robot,dyn.com)
	updateToolDisplay(taskRH)

def go_to(goal,pos_err_des,screw_len):

	# Goal HM
	refToGoalMatrix = eye(4); refToGoalMatrix[0:3,0:3]=calcRotFromRPY(goal[3],goal[4],goal[5]); refToGoalMatrix[0:3,3]=goal[0:3]

	# Preparation position
	preparation = dot(refToGoalMatrix,array([0.,0.,-0.03-screw_len,1]))

	# mini-task sequence definition
	action = array([preparation[0:3],goal[0:3],preparation[0:3]])

	robot.before.addSignal(taskRH.feature.name+".error")

	for i in range(3):

		#print "action = "+str(action[i])

		# Goal display
		robot.viewer.updateElementConfig('zmp',vectorToTuple(action[i])+(0.,0.,0.))

		# "Localization"
		refToTwoHandToolMatrix = dot(array(taskRH.feature.position.value),RHToTwoHandToolMatrix)
		refToScrewMatrix=dot(refToTwoHandToolMatrix,TwoHandToolToScrewMatrix)
		# from RH to a mat oriented as Screw, but centered in RH
		RH2ToScrewMatrix = eye(4); RH2ToScrewMatrix[0:3,0:3] = RHToScrewMatrix[0:3,0:3]; RH2ToScrewMatrix[0:3,3] = array([0.,0.,0.])

		# Determine the goal displacement in the screw reference
		screwDisplacement = dot(linalg.inv(refToScrewMatrix),hstack((action[i],1)))	# position=goal, homogeneous vector
		# and in the RH2 reference
		RHDisplacement = dot(RH2ToScrewMatrix,screwDisplacement)

		# Set the target for RH and LH task
		# To block the orientation on the screw-driver axis we have to use the actual position of the hands
		AimRHMatrix = eye(4); AimRHMatrix[0:4,3] = RHDisplacement
		taskRH.ref = matrixToTuple(dot(array(taskRH.feature.position.value),AimRHMatrix))
		taskLH.ref = matrixToTuple(dot(array(taskRH.ref),TwoHandSupportToTriggerMatrix))

		do()
		while linalg.norm(array(taskRH.feature.error.value)[0:3]) > pos_err_des:
			do()




for i in range(4):
	go_to(goal[i],pos_err_des,screw_len)

#go_to(goal1,pos_err_des,screw_len)

print "pos_err= "+str(linalg.norm(array(taskRH.feature.error.value)[0:3]))

