# ______________________________________________________________________________
# ******************************************************************************
# REACHIN A GIVEN POSITION.
# The robot (in a given position) moves the hands to the positions that represent 
# the grasp points of the tool. The tool position is relative to the robot
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

# --- ROBOT AND TWOHANDTOOL SIMU ---------------------------------------------------------------
# --- ROBOT AND TWOHANDTOOL SIMU ---------------------------------------------------------------
# --- ROBOT AND TWOHANDTOOL SIMU ---------------------------------------------------------------

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

xd = 0.4
yd = -0.1
zd = 1.1
rolld = 0.
pitchd = pi/5
yawd = pi/2


# Initial configuration
# We give the absolute position of the robot and then extract the relative rototranslation to impose to the TwoHandTool
from dynamic_graph.sot.dyninv.robot_specific import halfSittingConfig
x0=-1.6
y0=1.8
z0=0.64870185118253043  #0.6487018512
roll0=0.
pitch0=0.
yaw0=pi/2

halfSittingConfig[robotName] = (x0,y0,z0,0,0,yaw0)+halfSittingConfig[robotName][6:]
initialConfig[robotName]=halfSittingConfig[robotName]
robot.set( initialConfig[robotName] )

# TwoHandTool RotoTranslation / Homogeneous Matrix of the TwoHandTool. Normally given from the camera
RTMatrix = eye(4); RTMatrix[0:3,0:3] = calcRotFromRPY(roll0-rollr,pitch0-pitchr,yaw0-yawr)
RTMatrix[0:3,3] = (x0-xr,y0-yr,z0-zr)
newTwoHandTool = dot(RTMatrix,array([xd,yd,zd,1]))
TwoHandToolPos = vectorToTuple( newTwoHandTool[0:3] )
TwoHandToolRot = dot(RTMatrix[0:3,0:3],calcRotFromRPY(rolld,pitchd,yawd))
TwoHandToolOr = vectorToTuple(extractRPYFromRot(TwoHandToolRot))
TwoHandTool = TwoHandToolPos + TwoHandToolOr

refToTwoHandToolMatrix = eye(4); refToTwoHandToolMatrix[0:3,0:3] = TwoHandToolRot; refToTwoHandToolMatrix[0:3,3] = newTwoHandTool[0:3]

# Homogeneous Matrixes
refToTriggerMatrix=dot(refToTwoHandToolMatrix,TwoHandToolToTriggerMatrix)
refToSupportMatrix=dot(refToTwoHandToolMatrix,TwoHandToolToSupportMatrix)

# visualization
addRobotViewer(robot,small=True,verbose=True)
robot.viewer.updateElementConfig('fov',[0,0,-10,3.14,0,0])
robot.viewer.updateElementConfig('TwoHandTool',TwoHandTool)

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


# --- OTHER CONFIGS ----------------------------------------------------------
# --- OTHER CONFIGS ----------------------------------------------------------
# --- OTHER CONFIGS ----------------------------------------------------------


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

robot.increment(dt)
updateComDisplay(robot,dyn.com)

# Set the target for RH and LH task.
taskRH.ref = matrixToTuple(refToSupportMatrix)
taskRH.feature.selec.value = '110111'	# RX free
taskRH.gain.setByPoint(40,4,0.01,0.9)

taskLH.ref = matrixToTuple(refToTriggerMatrix)
taskLH.feature.selec.value = '110111'	# RX free
taskLH.gain.setByPoint(40,4,0.01,0.9)

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

# And run.
go()
