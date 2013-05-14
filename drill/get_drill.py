# ______________________________________________________________________________
# ******************************************************************************
# SIMPLEST EXAMPLE OF INVERSE KINEMATICS.
# The robot moves the hand to a given position.
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

# --- ROBOT SIMU ---------------------------------------------------------------
# --- ROBOT SIMU ---------------------------------------------------------------
# --- ROBOT SIMU ---------------------------------------------------------------

robotName = 'hrp10small'
robotDim=robotDimension[robotName]
robot = RobotSimu("robot")
robot.resize(robotDim)
dt=5e-3

from dynamic_graph.sot.dyninv.robot_specific import halfSittingConfig

x0=0.
y0=0.
z0=0.64870185118253043
halfSittingConfig[robotName] = (x0,y0,z0,0,0,0)+halfSittingConfig[robotName][6:]

q0=list(halfSittingConfig[robotName])
initialConfig[robotName]=tuple(q0)

robot.set( initialConfig[robotName] )

addRobotViewer(robot,small=True,verbose=True)
robot.viewer.updateElementConfig('fov',[0,0,-10,3.14,0,0])

#-------------------------------------------------------------------------------
#----- MAIN LOOP ---------------------------------------------------------------
#-------------------------------------------------------------------------------
from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
@loopInThread
def inc():
    robot.increment(dt)
    attime.run(robot.control.time)
    updateComDisplay(robot,dyn.com)

runner=inc()
[go,stop,next,n]=loopShortcuts(runner)

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

# ---- ROTATION FUNCTIONS -------------------------------------------------------------------
# ---- ROTATION FUNCTIONS -------------------------------------------------------------------
# ---- ROTATION FUNCTIONS -------------------------------------------------------------------

"""
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
"""

# ---- DRILL PARAMETERS -------------------------------------------------------------------
# ---- DRILL PARAMETERS -------------------------------------------------------------------
# ---- DRILL PARAMETERS -------------------------------------------------------------------

# DTool
xd = 0.4
yd = -0.15
zd = 1.0
rolld = pi/2
pitchd = 0.
yawd = pi
dTool = (xd,yd,zd,rolld,pitchd,yawd)
robot.viewer.updateElementConfig('dTool',dTool)

# Homogeneous Matrixes of the handle. Known from the tool model
dToolToTriggerMatrix=eye(4); dToolToTriggerMatrix[0:3,3] = array([0.025,-0.09,0.])
theta = math.atan2(0.025,0.09)
dToolToTriggerMatrix[0:3,0:3] = array(([-sin(theta),0.,cos(theta)],[cos(theta),0.,sin(theta)],[0.,1.,0.]))

# Homogeneous Matrix of the drilling part (rotation on z). Known from the tool model
#dToolToDrillMatrix=array([[1.,0.,0.,0.12],[0.,0.,-1.,0.],[0.,1.,0.,0.],[0.,0.,0.,1.]])
dToolToDrillMatrix=array([[0.,0.,-1,-0.12],[0.,1.,0.,0.],[1.,0.,0.,0.],[0.,0.,0.,1.]])

# Homogeneous Matrix drill to trigger
drillToTriggerMatrix=dot(linalg.inv(dToolToDrillMatrix),dToolToTriggerMatrix)

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

# --- TASK COM (if not walking)
taskCom = MetaTaskKineCom(dyn)

# --- POSTURE ---
#taskPosture = MetaTaskKinePosture(dyn)

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
#taskSupportSmall.referenceInf.value = (-0.02,-0.02,0)    # Xmin, Ymin
#askSupportSmall.referenceSup.value = (0.02,0.02,0)  # Xmax, Ymax
taskSupportSmall.referenceInf.value = (-0.02,-0.05,0)    # Xmin, Ymin
taskSupportSmall.referenceSup.value = (0.02,0.05,0)  # Xmax, Ymax
taskSupportSmall.dt.value=dt

# ---- WAIST TASK ---
taskWaist=MetaTask6d('waist',dyn,'waist','waist')
#taskWaist.task.controlGain.value = 10
gotoNd(taskWaist,(0.,0.,0.),'011000',(10,0.9,0.01,0.9))	# inside the function rot=0 --> we set a random position not to control it

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
RAD=pi/180

# Homogeneous Matrix of the DTool. Normally given from the camera
refToDToolMatrix=eye(4); refToDToolMatrix[0:3,0:3]=calcRotFromRPY(rolld,pitchd,yawd)
refToDToolMatrix[0:3,3] = array([xd,yd,zd])

# Homogeneous Matrixes To the trigger
refToTriggerMatrix = dot(refToDToolMatrix,dToolToTriggerMatrix)

# Set the target for RH and LH task. Third arg is an activation flag (say control only
# the XYZ translation), and last arg is the adaptive gain
taskRH.ref = matrixToTuple(refToTriggerMatrix)
taskRH.feature.selec.value = '110111'	# RX free
taskRH.gain = (10,1,0.01,0.9)


# Set up the stack solver.
sot.addContact(contactLF)
sot.addContact(contactRF)
push(taskJL)
push(taskRH)
push(taskWaist)
push(taskSupportSmall)

# And run.
go()
