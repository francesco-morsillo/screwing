# ______________________________________________________________________________
# ******************************************************************************
# SIMPLEST EXAMPLE OF INVERSE KINEMATICS.
# The robot operates with the tool from a given position.
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

# --- ROBOT AND SCREWTOOL SIMU ---------------------------------------------------------------
# --- ROBOT AND SCREWTOOL SIMU ---------------------------------------------------------------
# --- ROBOT AND SCREWTOOL SIMU ---------------------------------------------------------------

def extractRPYFromRot(Rot):
	YAW=math.atan2(Rot[1][0],Rot[0][0])
	PITCH=math.atan2(-Rot[2][0],sqrt(Rot[2][1]**2+Rot[2][2]**2))
	ROLL=math.atan2(Rot[2][1],Rot[2][2])
	return array([ROLL,PITCH,YAW])

def calcRotFromRPY(roll,pitch,yaw):
	yawRot=array([[cos(yaw),-sin(yaw),0.],[sin(yaw),cos(yaw),0.],[0.,0.,1.]])
	pitchRot=array([[cos(pitch),0.,sin(pitch)],[0.,1.,0.],[-sin(pitch),0.,cos(pitch)]])
	rollRot=array([[1.,0.,0.],[0.,cos(roll),-sin(roll)],[0.,sin(roll),cos(roll)]])
	Rot=dot(dot(yawRot,pitchRot),rollRot)
	return Rot


robotName = 'hrp14'
robotDim=robotDimension[robotName]
robot = RobotSimu("robot")
robot.resize(robotDim)
dt=5e-3


# Robot and sTool in the origin
xr=0.
yr=0.
zr=0.64870185118253043
rollr=0.
pitchr=0.
yawr=0.

xd = 0.4
yd = -0.1
zd = 1.1
rolld = 0.
pitchd = pi/5
yawd = pi/2


# Initial configuration
# We give the absolute position of the robot and then extract the relative rototranslation to impose to the sTool
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
robot.set((-1.57623,1.76296,0.697904,-2.09979e-23,2.28944e-23,1.53291,0.0378841,0.0400381,-0.22156,0.278669,-0.0571098,-0.0400381,0.0378841,0.0401019,-0.186463,0.224335,-0.0378714,-0.0401019,-0.0573337,-0.0797234,6.23359e-05,0.000590181,-0.446046,-0.184923,0.882121,-1.6442,1.74405,0.433484,0.486526,0.174533,-0.64642,-0.169962,0.353173,-1.10853,1.84994,1.83295,-0.000619176,0.174533))
"""

# Center initial configuration
robot.set((-0.021742,-0.0248409,0.699608,-4.57548e-24,-1.39539e-24,-0.0396372,0.0396372,0.0417173,-0.187983,0.262897,-0.074914,-0.0417173,0.0396372,0.0417877,-0.147051,0.197411,-0.0503601,-0.0417877,-0.0791486,-0.0797719,-5.21839e-05,0.00148229,-0.417676,-0.176956,0.902757,-1.68616,1.7606,0.463635,0.488594,0.174533,-0.600312,-0.169887,0.369851,-1.18484,1.84986,1.91579,-0.000575116,0.174533))


# Screw Lenght
screw_len = 0.

# STool RotoTranslation / Homogeneous Matrix of the STool. Normally given from the camera
RTMatrix = eye(4); RTMatrix[0:3,0:3] = calcRotFromRPY(roll0-rollr,pitch0-pitchr,yaw0-yawr)
RTMatrix[0:3,3] = (x0-xr,y0-yr,z0-zr)
newSTool = dot(RTMatrix,array([xd,yd,zd,1]))
sToolPos = vectorToTuple( newSTool[0:3] )
SToolRot = dot(RTMatrix[0:3,0:3],calcRotFromRPY(rolld,pitchd,yawd))
sToolOr = vectorToTuple(extractRPYFromRot(SToolRot))
sTool = sToolPos + sToolOr

# Homogeneous Matrixes of the handles. Known from the tool model
sToolToTriggerMatrix=eye(4); sToolToTriggerMatrix[0:3,3] = (0.25,0.,0.)
sToolToSupportMatrix=eye(4); sToolToSupportMatrix[0:3,3] = (-0.03,0.,0.)

# Homogeneous Matrix of the screwing part (rotation on y). Known from the tool model
#sToolToScrewMatrix=array([[1.,0.,0.,-0.11],[0.,0.,-1.,-0.03],[0.,1.,0.,0.],[0.,0.,0.,1.]])
sToolToScrewMatrix=array([[1.,0.,0.,-0.11],[0.,1.,0.,-0.03],[0.,0.,1.,0.],[0.,0.,0.,1.]])


# Homogeneous Matrix screw to support
screwToSupportMatrix=dot(linalg.inv(sToolToScrewMatrix),sToolToSupportMatrix)

# Homogeneous Matrix support to trigger
supportToTriggerMatrix=dot(linalg.inv(sToolToSupportMatrix),sToolToTriggerMatrix)



# goals
goal1 = (0.6,-0.1,1.3,0.,1.57,0.)
#goal1 = (-1.5,2.4,1.3,-1.57,0.,0.)
goal2 = (-1.7,2.4,1.3,-1.57,0.,0.)
goal3 = (-1.7,2.4,1.1,-1.57,0.,0.)
goal4 = (-1.5,2.4,1.1,-1.57,0.,0.)



# visualization
addRobotViewer(robot,small=True,verbose=True)
robot.viewer.updateElementConfig('fov',[0,0,-10,3.14,0,0])
robot.viewer.updateElementConfig('sTool',sTool)
robot.viewer.updateElementConfig('goal1',goal1)
robot.viewer.updateElementConfig('goal2',goal2)
robot.viewer.updateElementConfig('goal3',goal3)
robot.viewer.updateElementConfig('goal4',goal4)

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


# STool Moving

class SToolPosition:
    def __init__(self,xyzRPY=(0.,0.,0.,0.,0.,0.)):
        self.sTool = xyzRPY

    def follow(self,toto):
	self.sTool = vectorToTuple(toto[0:3,3]) + vectorToTuple(extractRPYFromRot(toto[0:3,0:3]))
	robot.viewer.updateElementConfig('sTool',self.sTool)
	
dTool = SToolPosition(sTool)


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
taskSupportSmall.referenceInf.value = (center[0]-0.02,center[1]-0.05,0)    # Xmin, Ymin
taskSupportSmall.referenceSup.value = (center[0]+0.05,center[1]+0.07,0)    # Xmax, Ymax

# Set up the stack solver.
sot.addContact(contactLF)
sot.addContact(contactRF)
push(taskJL)
push(taskRH)
push(taskLH)
push(taskWaist)
push(taskSupportSmall)


# Static task options
taskRH.feature.selec.value = '111111'	# RX no more free with the tool
taskRH.gain = (1000000,1,0.01,0.9)
taskLH.feature.selec.value = '110111'	# RX free
taskLH.gain = (1000000,1,0.01,0.9)


#RH-STool Homogeneous Transformation Matrix (fixed in time)
refToSToolMatrix = eye(4); refToSToolMatrix[0:3,0:3] = SToolRot; refToSToolMatrix[0:3,3] = newSTool[0:3]
taskRH.feature.position.recompute(0)
RHToSToolMatrix = dot(linalg.inv(array(taskRH.feature.position.value)),refToSToolMatrix)
#!!!!!! RH and Support are different references, because the X rotation is not controlled!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# The hands have to cover the same displacement, but in a different reference
RHToScrewMatrix = dot(RHToSToolMatrix,sToolToScrewMatrix)

ep = 1 # random
change_goal = True
state = 0
goalGlobal = None

def posErrCalc(goal,RHToScrewMatrix):
	taskRH.feature.position.recompute
	screwMatrix = dot(array(taskRH.feature.position.value),RHToScrewMatrix)
	global ep
	ep = linalg.norm(goal - screwMatrix[0:3,3])
	return ep


def new_goal(goal):

	taskRH.feature.position.recompute
	refToSToolMatrix = dot(array(taskRH.feature.position.value),RHToSToolMatrix)

	# Homogeneous Matrixe from the reference to the screw
	refToScrewMatrix=dot(refToSToolMatrix,sToolToScrewMatrix)

	# from Screw to a mat oriented as RH, but centered in RH
	RH2ToScrewMatrix = eye(4); RH2ToScrewMatrix[0:3,0:3] = RHToScrewMatrix[0:3,0:3]; RH2ToScrewMatrix[0:3,3] = array([0.,0.,0.]) 

	if state==0: # Reach the position
		robot.viewer.updateElementConfig('zmp',vectorToTuple(goal))

	if state==1: # Execute the task
		refToGoalMatrix = eye(4); refToGoalMatrix[0:3,0:3]=calcRotFromRPY(goal[3],goal[4],goal[5]); refToGoalMatrix[0:3,3]=goal[0:3]
		global screw_len
		position = dot(refToGoalMatrix,array([0.,0.,0.03+screw_len,1]))
		goal[0:3] = position[0:3]
		robot.viewer.updateElementConfig('zmp',vectorToTuple(goal))
	
	# Determine the goal displacement in the screw reference
	screwDisplacement = dot(linalg.inv(refToScrewMatrix),hstack((goal[0:3],1)))	# position=goal, homogeneous vector
	# and in the RH2 reference
	RHDisplacement = dot(RH2ToScrewMatrix,screwDisplacement)

	# Set the target for RH and LH task
	# To block the orientation on the screw-driver axis we have to use the actual position of the hands
	AimRHMatrix = eye(4); AimRHMatrix[0:4,3] = RHDisplacement
	taskRH.ref = matrixToTuple(dot(array(taskRH.feature.position.value),AimRHMatrix))
	taskLH.ref = matrixToTuple(dot(array(taskRH.ref),supportToTriggerMatrix))	



#-------------------------------------------------------------------------------
#----- MAIN LOOP ---------------------------------------------------------------
#-------------------------------------------------------------------------------
from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
@loopInThread
def inc():
	global change_goal,state,RHToSToolMatrix,RHToScrewMatrix
	if change_goal:
		global new_goal,goalGlobal
		goalGlobal = array(goal1)
		new_goal(goalGlobal)
		change_goal = False
	robot.increment(dt)
	attime.run(robot.control.time)
	updateComDisplay(robot,dyn.com)
	# Move the sTool
	dTool.follow(dot(array(taskRH.feature.position.value),RHToSToolMatrix))

	# Position error calculation and change of state
	if posErrCalc(goalGlobal[0:3],RHToScrewMatrix)<0.01 :
		global state, change_goal
		state = 1 - state
		change_goal = True

	return change_goal,state

runner=inc()
[go,stop,next,n]=loopShortcuts(runner)


# And run
go()

