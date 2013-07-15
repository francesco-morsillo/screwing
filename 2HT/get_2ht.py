# ______________________________________________________________________________
# ******************************************************************************
#
#    GET USING-TOOL POSITION
#       Robot: HRP-2 N.14
#       Tasks: Reach the "using-tool position" from half-sitting position
# 
# ______________________________________________________________________________
# ******************************************************************************  

from dynamic_graph.sot.core import *
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.dyninv import *
from dynamic_graph.script_shortcuts import optionalparentheses
from dynamic_graph import plug

from dynamic_graph.sot.core.matrix_util import matrixToTuple, matrixToRPY, RPYToMatrix
from dynamic_graph.sot.core.utils.viewer_helper import addRobotViewer,VisualPinger,updateComDisplay
from dynamic_graph.sot.core.utils.thread_interruptible_loop import *
from dynamic_graph.sot.core.utils.attime import attime

from dynamic_graph.sot.dyninv.robot_specific import pkgDataRootDir, modelName, robotDimension, initialConfig, gearRatio, inertiaRotor, halfSittingConfig
from dynamic_graph.sot.dyninv.meta_task_dyn_6d import MetaTaskDyn6d
from dynamic_graph.sot.dyninv.meta_tasks_dyn import MetaTaskDynCom, MetaTaskDynPosture, AddContactHelper, gotoNd
from dynamic_graph.sot.dyninv.meta_tasks_dyn_relative import MetaTaskDyn6dRel, gotoNdRel, goto6dRel

from dynamic_graph.sot.fmorsill.utility import *
from dynamic_graph.sot.fmorsill.rob_view_lib import *
from dynamic_graph.sot.core.feature_vector3 import *

from numpy import *


# ------------------------------------------------------------------------------
# --- ROBOT DYNAMIC SIMULATION -------------------------------------------------
# ------------------------------------------------------------------------------

robotName = 'hrp14small'
robotDim   = robotDimension[robotName]
RobotClass = RobotSimu
robot      = RobotClass("robot")
robot.resize(robotDim)

x0=0.
y0=0.
z0=0.64870185118253043
halfSittingConfig[robotName] = (x0,y0,z0,0,0,0)+halfSittingConfig[robotName][6:]
pose = halfSittingConfig[robotName]

robot.set( pose )

addRobotViewer(robot,small=True,verbose=False)

dt=5e-3

robot.setSecondOrderIntegration()

#-----------------------------------------------------------------------------
#---- DYN --------------------------------------------------------------------
#-----------------------------------------------------------------------------

modelDir          = pkgDataRootDir[robotName]
xmlDir            = pkgDataRootDir[robotName]
specificitiesPath = xmlDir + '/HRP2SpecificitiesSmall.xml'
jointRankPath     = xmlDir + '/HRP2LinkJointRankSmall.xml'

dyn = Dynamic("dyn")
dyn.setFiles(modelDir,modelName[robotName],specificitiesPath,jointRankPath)
dyn.parse()

dyn.inertiaRotor.value = inertiaRotor[robotName]
dyn.gearRatio.value    = gearRatio[robotName]

plug(robot.state,dyn.position)
plug(robot.velocity,dyn.velocity)
dyn.acceleration.value = robotDim*(0.,)

dyn.ffposition.unplug()
dyn.ffvelocity.unplug()
dyn.ffacceleration.unplug()

dyn.setProperty('ComputeBackwardDynamics','true')
dyn.setProperty('ComputeAccelerationCoM','true')

robot.control.unplug()

# ---- TWOHANDSTOOL PARAMETERS -------------------------------------------------------------------
# ---- TWOHANDSTOOL PARAMETERS -------------------------------------------------------------------
# ---- TWOHANDSTOOL PARAMETERS -------------------------------------------------------------------


# TwoHandTool
xd = 0.4
yd = -0.13
zd = 0.9
roll = 0.
#pitch = pi/5
pitch = 0
yaw = pi/2

TwoHandTool = (xd,yd,zd,roll,pitch,yaw)
robot.viewer.updateElementConfig('TwoHandTool',TwoHandTool)

# Homogeneous Matrix of the TwoHandTool. Normally given from the camera
refToTwoHandToolMatrix = RPYToMatrix(TwoHandTool)

# Homogeneous Matrixes
refToTriggerMatrix=dot(refToTwoHandToolMatrix,TwoHandToolToTriggerMatrix)
refToSupportMatrix=dot(refToTwoHandToolMatrix,TwoHandToolToSupportMatrix)


#-----------------------------------------------------------------------------
# --- OPERATIONAL TASKS (For HRP2-14)-----------------------------------------
#-----------------------------------------------------------------------------

taskWaist = MetaTaskDyn6d('taskWaist', dyn, 'waist', 'waist')
taskChest = MetaTaskDyn6d('taskChest', dyn, 'chest', 'chest')
taskHead  = MetaTaskDyn6d('taskHead', dyn, 'head', 'gaze')
taskRH    = MetaTaskDyn6d('rh', dyn, 'rh', 'right-wrist')
taskLH    = MetaTaskDyn6d('lh', dyn, 'lh', 'left-wrist')

for task in [ taskWaist, taskChest, taskHead, taskRH, taskLH]:
    task.feature.frame('desired')
    task.gain.setConstant(10)
    task.task.dt.value = dt
    task.featureDes.velocity.value=(0,0,0,0,0,0)

handMgrip=eye(4); handMgrip[0:3,3] = (0,0,-0.14)
taskRH.opmodif = matrixToTuple(handMgrip)
taskLH.opmodif = matrixToTuple(handMgrip)

# CoM Task
taskCom = MetaTaskDynCom(dyn,dt)

# Posture Task
taskPosture = MetaTaskDynPosture(dyn,dt)

# Angular position and velocity limits
taskLim = TaskDynLimits('taskLim')
plug(dyn.position,taskLim.position)
plug(dyn.velocity,taskLim.velocity)
taskLim.dt.value = dt

dyn.upperJl.recompute(0)
dyn.lowerJl.recompute(0)
taskLim.referencePosInf.value = dyn.lowerJl.value
taskLim.referencePosSup.value = dyn.upperJl.value

#dqup = (0, 0, 0, 0, 0, 0, 200, 220, 250, 230, 290, 520, 200, 220, 250, 230, 290, 520, 250, 140, 390, 390, 240, 140, 240, 130, 270, 180, 330, 240, 140, 240, 130, 270, 180, 330)
dqup = (1000,)*robotDim
taskLim.referenceVelInf.value = tuple([-val*pi/180 for val in dqup])
taskLim.referenceVelSup.value = tuple([ val*pi/180 for val in dqup])

taskLim.controlGain.value = 0.3


# Task Inequality
featureHeight = FeatureGeneric('featureHeight')
plug(dyn.com,featureHeight.errorIN)
plug(dyn.Jcom,featureHeight.jacobianIN)

taskHeight=TaskDynInequality('taskHeight')
plug(dyn.velocity,taskHeight.qdot)
taskHeight.add(featureHeight.name)
taskHeight.selec.value = '100'
taskHeight.referenceInf.value = (0.,0.,0.)    # Zmin
taskHeight.referenceSup.value = (0.,0.,0.81)  # Zmax
taskHeight.dt.value=dt
taskHeight.controlGain.value = 1.0

#-----------------------------------------------------------------------------
# --- Stack of tasks controller  ---------------------------------------------
#-----------------------------------------------------------------------------

sot = SolverKine('sot')

sot.setSize(robotDim)

sot.setSecondOrderKinematics()
plug(dyn.velocity,sot.velocity)

plug(sot.control, robot.control)

#-----------------------------------------------------------------------------
# ---- CONTACT: Contact definition -------------------------------------------
#-----------------------------------------------------------------------------

# Left foot contact
contactLF = MetaTaskDyn6d('contact_lleg',dyn,'lf','left-ankle')
contactLF.featureDes.velocity.value=(0,0,0,0,0,0)
contactLF.feature.frame('desired')
contactLF.name = "LF"

# Right foot contact
contactRF = MetaTaskDyn6d('contact_rleg',dyn,'rf','right-ankle')
contactRF.featureDes.velocity.value=(0,0,0,0,0,0)
contactRF.feature.frame('desired')
contactRF.name = "RF"


contactRF.support = ((0.11,-0.08,-0.08,0.11),(-0.045,-0.045,0.07,0.07),(-0.105,-0.105,-0.105,-0.105))
contactLF.support = ((0.11,-0.08,-0.08,0.11),(-0.07,-0.07,0.045,0.045),(-0.105,-0.105,-0.105,-0.105))

# Imposed errordot = 0
contactLF.feature.errordot.value=(0,0,0,0)
contactRF.feature.errordot.value=(0,0,0,0)

"""
gotoNd(contactLF,(0.,0.,-0.104,0.,0.,0.),'111100',(10.,))
contactLF.task.errorTimeDerivative.value = [0., 0., 0., 0.]
gotoNd(contactRF,(0.,0.,-0.104,0.,0.,0.),'111100',(10.,))
contactRF.task.errorTimeDerivative.value = [0., 0., 0., 0.]
"""

#-----------------------------------------------------------------------------
# --- TRACER ------------------------------------------------------------------
#-----------------------------------------------------------------------------

from dynamic_graph.tracer import *
tr = Tracer('tr')
tr.open('/tmp/','dyn_','.dat')
tr.start()
robot.after.addSignal('tr.triger')

tr.add('robot.state','qn')
tr.add('dyn.com','com')
tr.add('taskLim.normalizedPosition','qnorm')

robot.after.addSignal('tr.triger')
robot.after.addSignal('dyn.com')
robot.after.addSignal('taskLim.normalizedPosition')
robot.after.addSignal('robot.state')


# ------------------------------------------------------------------------------
# --- MAIN LOOP ----------------------------------------------------------------
# ------------------------------------------------------------------------------

def inc():
	robot.increment(dt)
	attime.run(robot.control.time)
	updateComDisplay(robot,dyn.com)
        #verif.record()

@loopInThread
def loop():
    inc()
runner=loop()


# --- shortcuts -------------------------------------------------
@optionalparentheses
def go(): runner.play()
@optionalparentheses
def stop(): runner.pause()
@optionalparentheses
def next(): inc()
@optionalparentheses
def iter():         print 'iter = ',robot.state.time
@optionalparentheses
def status():       print runner.isPlay


#-----------------------------------------------------------------------------
# --- RUN --------------------------------------------------------------------
#-----------------------------------------------------------------------------

# Task references
dyn.com.recompute(0)
taskCom.featureDes.errorIN.value = dyn.com.value
taskCom.task.controlGain.value = 10
taskCom.feature.selec.value = '011'
taskCom.gain.setConstant(1)

taskPosture.ref = pose
taskPosture.gain.setConstant(5)


# Set the target for RH and LH task. Selec is the activation flag (say control only
# the XYZ translation), and gain is the adaptive gain (10 at the target, 0.1
# far from it, with slope st. at 0.01m from the target, 90% of the max gain
# value is reached
target = vectorToTuple(refToSupportMatrix[0:3,3])
gotoNd(taskRH, target, "000111",(50,1,0.01,0.9))

target = vectorToTuple(refToTriggerMatrix[0:3,3])
gotoNd(taskLH, target, "000111",(50,1,0.01,0.9))

# Orientation RF and LF
featureVecRH = FeatureVector3("featureVecRH")
plug(dyn.signal('rh'),featureVecRH.signal('position'))
plug(dyn.signal('Jrh'),featureVecRH.signal('Jq'))
featureVecRH.vector.value = array([1.,0.,0.])
featureVecRH.positionRef.value = dot(refToTwoHandToolMatrix[0:3,0:3],array([1.,0.,0.]))
taskRH.task.add(featureVecRH.name)

featureVecLH = FeatureVector3("featureVecLH")
plug(dyn.signal('lh'),featureVecLH.signal('position'))
plug(dyn.signal('Jlh'),featureVecLH.signal('Jq'))
featureVecLH.vector.value = array([1.,0.,0.])
featureVecLH.positionRef.value = dot(refToTwoHandToolMatrix[0:3,0:3],array([-1.,0.,0.]))
taskLH.task.add(featureVecLH.name)



gotoNd(taskChest,(0.,0.,0.,0.,0.,0.),'111000',(1.,))	# inside the function rot=0 --> we set a random position not to control it
taskChest.task.errorTimeDerivative.value = [0., 0., 0.]


gotoNd(taskWaist,(0.,0.,0.,0.,0.,0.),'111000',(1.,))	# inside the function rot=0 --> we set a random position not to control it
taskWaist.task.errorTimeDerivative.value = [0., 0., 0.]

# Sot
sot.clear()
sot.addContact(contactLF)
sot.addContact(contactRF)
sot.push(taskLim.name)
sot.push(taskCom.task.name)
sot.push(taskRH.task.name)
sot.push(taskLH.task.name)
sot.push(taskWaist.task.name)
sot.push(taskChest.task.name)
sot.push(taskHeight.name)
sot.push(taskPosture.task.name)

"""
taskRH.feature.error.recompute(0)
while linalg.norm(array(taskRH.feature.error.value)[0:3]) > pos_err_des:
	robot.increment(dt)
	attime.run(robot.control.time)
	updateComDisplay(robot,dyn.com)

print "pos_err= "+str(linalg.norm(array(taskRH.feature.error.value)[0:3]))
"""


