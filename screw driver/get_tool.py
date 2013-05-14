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

robotName = 'hrp14small'
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

"""
#for hrp14
robot.set((-0.0351686,0.0170821,0.68948,-9.19102e-23,7.90629e-24,0.0106996,-0.0106996,-0.0291983,-0.280283,0.410555,-0.130272,0.0291983,-0.0106996,-0.0292376,-0.298043,0.441178,-0.143135,0.0292376,0.011906,-0.0870321,-8.89544e-19,-4.54935e-20,-0.122959,-0.749944,0.497043,-1.85726,1.5535,0.41108,0.174533,-0.289026,-0.174316,-0.806614,-1.27769,-0.657848,0.683733,0.174533))
"""

""" #for hrp10
robot.set((-0.0680573,0.0549703,0.681123,5.3467e-26,9.25815e-26,0.0607429,-0.0607429,-0.0950824,-0.333218,0.445825,-0.112607,0.0950824,-0.0607429,-0.0951705,-0.380667,0.5136,-0.132932,0.0951705,-0.196613,-0.0799953,-0.00461902,0.00508595,-0.598498,-0.0548141,0.908472,-1.45118,1.70262,0.420043,0.462092,0.174533,-0.551393,-0.169999,0.403413,-1.33973,1.85,1.66127,0.0234165,0.174533))
"""
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

# ---- SCREW-DRIVER PARAMETERS -------------------------------------------------------------------
# ---- SCREW-DRIVER PARAMETERS -------------------------------------------------------------------
# ---- SCREW-DRIVER PARAMETERS -------------------------------------------------------------------

def calcFromRPY(roll,pitch,yaw):
	yawRot=array([[cos(yaw),-sin(yaw),0.],[sin(yaw),cos(yaw),0.],[0.,0.,1.]])
	pitchRot=array([[cos(pitch),0.,sin(pitch)],[0.,1.,0.],[-sin(pitch),0.,cos(pitch)]])
	rollRot=array([[1.,0.,0.],[0.,cos(roll),-sin(roll)],[0.,sin(roll),cos(roll)]])
	Rot=dot(dot(yawRot,pitchRot),rollRot)
	return Rot

# STool
xd = 0.4
yd = -0.2
zd = 1.1
roll = 0.
pitch = pi/5
yaw = pi/2
sTool = (xd,yd,zd,roll,pitch,yaw)
robot.viewer.updateElementConfig('sTool',sTool)

# Homogeneous Matrix of the STool. Normally given from the camera
refToSToolMatrix=eye(4); refToSToolMatrix[0:3,0:3]=calcFromRPY(roll,pitch,yaw)
refToSToolMatrix[0:3,3] = (xd,yd,zd)



# Homogeneous Matrixes of the handles. Known from the tool model
sToolToTriggerMatrix=eye(4); sToolToTriggerMatrix[0:3,0:3]=array([[-1.,0.,0.],[0.,-1.,0.],[0.,0.,1.]])
sToolToTriggerMatrix[0:3,3] = (0.25,0.,0.)
sToolToSupportMatrix=eye(4)
sToolToSupportMatrix[0:3,3] = (-0.03,0.,0.)

"""
# Example
# Homogeneous Matrix of the STool. Normally given from the camera
refToSToolMatrix=array([[0.,-1.,0.,0.],[1.,0.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]])
refToSToolMatrix[0:3,3] = (0.7,0.,random.uniform(0.7,1.3))

# Homogeneous Matrixes of the handles. Known from the tool model
sToolToTriggerMatrix=array([[0.,1.,0.,0.],[0.,0.,1.,0.],[1.,0.,0.,0.],[0.,0.,0.,1.]])
sToolToTriggerMatrix[0:3,3] = (0.,0.15,-0.12)
sToolToSupportMatrix=array([[1.,0.,0.,0.],[0.,0.,1.,0.],[0.,-1.,0.,0.],[0.,0.,0.,1.]])
sToolToSupportMatrix[0:3,3] = (0.035,-0.08,-0.05)
"""

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

# --- TASK SUPPORT --------------------------------------------------
featureSupport    = FeatureGeneric('featureSupport')
plug(dyn.com,featureSupport.errorIN)
plug(dyn.Jcom,featureSupport.jacobianIN)

taskSupport=TaskInequality('taskSupport')
taskSupport.add(featureSupport.name)
taskSupport.selec.value = '011'
taskSupport.referenceInf.value = (-0.09,-0.17,0)    # Xmin, Ymin
taskSupport.referenceSup.value = (0.14,0.17,0)  # Xmax, Ymax
taskSupport.dt.value=dt

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

# Homogeneous Matrixes
refToTriggerMatrix=dot(refToSToolMatrix,sToolToTriggerMatrix)
refToSupportMatrix=dot(refToSToolMatrix,sToolToSupportMatrix)

"""
# Workaround Manipulability
WATriggerMatrix=array([[1.,0.,0.,0.],[0.,0.,-1.,0.],[0.,1.,0.,0.],[0.,0.,0.,1.]])
refToTriggerMatrix=dot(refToTriggerMatrix,WATriggerMatrix) #LH
WASupportMatrix=array([[1.,0.,0.,0.],[0.,0.,1.,0.],[0.,-1.,0.,0.],[0.,0.,0.,1.]])
refToSupportMatrix=dot(refToSupportMatrix,WASupportMatrix) #RH
"""

# --------------
#sToolToScrewMatrix=array([[1.,0.,0.,-0.11],[0.,0.,-1.,-0.03],[0.,1.,0.,0.],[0.,0.,0.,1.]])
EdgeMatrix = eye(4); EdgeMatrix[0:3,3] = array([0.,0.03,0.])
target = dot(refToSToolMatrix,EdgeMatrix)
robot.viewer.updateElementConfig('zmp',vectorToTuple(target[0:3,3])+(0,0,0))

# Set the target for RH and LH task. Third arg is an activation flag (say control only
# the XYZ translation), and last arg is the adaptive gain (5 at the target, 1
# far from it, with slope st. at 0.01m from the target, 90% of the max gain
# value is reached)
taskRH.ref = matrixToTuple(refToSupportMatrix)
taskRH.feature.selec.value = '110111'	# RX free
taskRH.gain = (100,1,0.01,0.9)

taskLH.ref = matrixToTuple(refToTriggerMatrix)
taskLH.feature.selec.value = '110111'	# RX free
taskLH.gain = (100,1,0.01,0.9)

# Set up the stack solver.
sot.addContact(contactLF)
sot.addContact(contactRF)
push(taskJL)
push(taskRH)
push(taskLH)
push(taskWaist)
push(taskSupport)

# And run.
go()
