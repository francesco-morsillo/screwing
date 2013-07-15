# ______________________________________________________________________________
# ******************************************************************************
#
#    BASIC EXAMPLE OF THE DYNAMIC SOT
#       Robot: HRP-2 N.14
#       Tasks: Rotate the head and move the arms
# 
# ______________________________________________________________________________
# ******************************************************************************  

from dynamic_graph.sot.core import *
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.dyninv import *
from dynamic_graph.script_shortcuts import optionalparentheses
from dynamic_graph import plug

from dynamic_graph.sot.core.matrix_util import matrixToTuple, RPYToMatrix, matrixToRPY
from dynamic_graph.sot.core.utils.viewer_helper import addRobotViewer,VisualPinger,updateComDisplay
from dynamic_graph.sot.core.utils.thread_interruptible_loop import *
from dynamic_graph.sot.core.utils.attime import attime

from dynamic_graph.sot.dyninv.robot_specific import pkgDataRootDir, modelName, robotDimension, initialConfig, gearRatio, inertiaRotor
from dynamic_graph.sot.dyninv.meta_task_dyn_6d import MetaTaskDyn6d
from dynamic_graph.sot.dyninv.meta_tasks_dyn import MetaTaskDynCom, MetaTaskDynPosture, AddContactHelper, gotoNd
from dynamic_graph.sot.dyninv.meta_tasks_dyn_relative import MetaTaskDyn6dRel, gotoNdRel, goto6dRel

from dynamic_graph.sot.fmorsill.utility import *
from dynamic_graph.sot.fmorsill.rob_view_lib import *

from numpy import *
import time

# ------------------------------------------------------------------------------
# --- ROBOT DYNAMIC SIMULATION -------------------------------------------------
# ------------------------------------------------------------------------------

robotName = 'hrp14small'
robotDim   = robotDimension[robotName]
RobotClass = RobotSimu
robot      = RobotClass("robot")
robot.resize(robotDim)

pose = (-0.030073618647834879, 0.016415887002202933, 0.62245944857810387, -5.4272918219751486e-20, 2.2304361400809676e-20, 0.0092340515812077348, -0.0092340515812077365, -0.031684365127837365, -0.5963821161312286, 1.0426697092875947, -0.44628759315636635, 0.031684365127837365, -0.0092340515812077348, -0.03174248663241639, -0.60591626591827519, 1.0568307464403983, -0.45091448052212324, 0.03174248663241639, -0.011123297511703737, -0.087265999999999927, -0.00033750538179865144, -0.0005624238088200199, -0.30213952018426748, -0.71906866277071557, 0.70978391033338306, -1.7041087671646877, 1.5030587263678217, 0.48707940835840557, 0.17569313715492132, -0.38348731360690541, -0.17453299999999988, -0.80343898121984014, -1.041280119185096, -0.51958844641260804, 0.54990959413946805, 0.17519281104793244)
#write_xml(pose)
#write_pos_py(pose[6:36])
robot.set(pose)

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


#-----------------------------------------------------------------------------
# --- OPERATIONAL TASKS (For HRP2-14)-----------------------------------------
#-----------------------------------------------------------------------------

taskRH    = MetaTaskDyn6d('rh', dyn, 'rh', 'right-wrist')
taskLH    = MetaTaskDyn6d('lh', dyn, 'lh', 'left-wrist')
taskRel = MetaTaskDyn6dRel('rel',dyn,'rh','lh','right-wrist','left-wrist')

for task in [ taskRH, taskLH, taskRel]:
    task.feature.frame('current')
    task.gain.setConstant(10)
    task.task.dt.value = dt
    task.featureDes.velocity.value=(0,0,0,0,0,0)

handMgrip=eye(4); handMgrip[0:3,3] = (0,0,-0.14)
taskRH.opmodif = matrixToTuple(handMgrip)
taskLH.opmodif = matrixToTuple(handMgrip)

# RELATIVE POSITION TASK
taskRel.opmodif = matrixToTuple(handMgrip)
taskRel.opmodifBase = matrixToTuple(handMgrip)

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

taskLim.controlGain.value = 1

#-----------------------------------------------------------------------------
# --- Stack of tasks controller  ---------------------------------------------
#-----------------------------------------------------------------------------

sot = SolverKine('sot')

sot.setSecondOrderKinematics()
plug(dyn.velocity,sot.velocity)

sot.setSize(robotDim)

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

# Imposed erordot = 0
contactLF.feature.errordot.value=(0,0,0,0,0,0)
contactRF.feature.errordot.value=(0,0,0,0,0,0)



#-----------------------------------------------------------------------------
# --- TRACER ------------------------------------------------------------------
#-----------------------------------------------------------------------------

from dynamic_graph.tracer import *
tr = Tracer('tr')
tr.open('/tmp/','dyn_','.dat')

tr.add('dyn.com','com2')

robot.after.addSignal('tr.triger')
robot.after.addSignal('dyn.com')
robot.after.addSignal('taskLim.normalizedPosition')

# ---- STOOL PARAMETERS -------------------------------------------------------------------
# ---- STOOL PARAMETERS -------------------------------------------------------------------
# ---- STOOL PARAMETERS -------------------------------------------------------------------


# TwoHandTool
xd = 0.4
yd = -0.2
zd = 1.
roll = 0.
pitch = pi/5
yaw = pi/2

TwoHandTool = (xd,yd,zd,roll,pitch,yaw) 
robot.viewer.updateElementConfig('TwoHandTool',TwoHandTool)

# Homogeneous Matrix of the TwoHandTool. Normally given from the camera
refToTwoHandToolMatrix = array(RPYToMatrix(TwoHandTool))

# Homogeneous Matrixes
refToTriggerMatrix=dot(refToTwoHandToolMatrix,TwoHandToolToTriggerMatrix)
refToSupportMatrix=dot(refToTwoHandToolMatrix,TwoHandToolToSupportMatrix)

# Homogeneous Matrix of the screwing part (rotation on z)
refToScrewMatrix=dot(refToTwoHandToolMatrix,TwoHandToolToScrewMatrix)
robot.viewer.updateElementConfig('zmp',vectorToTuple(refToScrewMatrix[0:3,3])+(0,0,0))



#-----------------------------------------------------------------------------
# --- RUN --------------------------------------------------------------------
#-----------------------------------------------------------------------------

# Task references
dyn.com.recompute(0)
taskCom.featureDes.errorIN.value = dyn.com.value
taskCom.task.controlGain.value = 10

taskCom.feature.selec.value = '011'
taskCom.gain.setConstant(0.1)

taskPosture.ref = pose


# Set the target for RH and LH task. Selec is the activation flag (say control only
# the XYZ translation), and gain is the adaptive gain (10 at the target, 0.1
# far from it, with slope st. at 0.01m from the target, 90% of the max gain
# value is reached
displacementMatrix=eye(4); displacementMatrix[0:3,3] = array([0.1,0.,0.])

taskRH.feature.position.recompute(0)
taskLH.feature.position.recompute(0)
targetRH = vectorToTuple(array(matrixToRPY( dot(displacementMatrix,array(taskRH.feature.position.value)) )))
gotoNd(taskRH, targetRH, "111111",(50,1,0.01,0.9))

gotoNdRel(taskRel,taskRH.feature.position.value,taskLH.feature.position.value,'110111',(50,1,0.01,0.9))
taskRel.feature.errordot.value=(0,0,0,0,0)	# not to forget!!


############################################################


#Task Waist
dyn.createOpPoint('waist','waist')
featureWaist    = FeaturePoint6d('featureWaist')
plug(dyn.waist,featureWaist.position)
plug(dyn.Jwaist,featureWaist.Jq)
taskWaist=TaskDynInequality('taskWaist')
plug(dyn.velocity,taskWaist.qdot)
taskWaist.add(featureWaist.name)
taskWaist.selec.value = '110000'
taskWaist.referenceInf.value = (0.,0.,0.,0.,-0.1,-0.7)    # Roll Pitch Yaw min
taskWaist.referenceSup.value = (0.,0.,0.,0.,0.5,0.7)  # Roll Pitch Yaw max
taskWaist.dt.value=dt
taskWaist.controlGain.value = 10


############################################################


# Sot
sot.clear()
sot.addContact(contactLF)
sot.addContact(contactRF)
sot.push(taskLim.name)
sot.push(taskRel.task.name)
sot.push(taskRH.task.name)
sot.push(taskCom.task.name)
sot.push(taskWaist.name)
sot.push(taskPosture.task.name)

taskRH.feature.error.recompute(0)
RHToTwoHandToolMatrix = dot(linalg.inv(array(taskRH.feature.position.value)),refToTwoHandToolMatrix)



# ------------------------------------------------------------------------------
# --- MAIN LOOP ----------------------------------------------------------------
# ------------------------------------------------------------------------------

def inc():
    robot.increment(dt)
    attime.run(robot.control.time)
    updateComDisplay(robot,dyn.com)
    updateToolDisplay(taskRH,RHToTwoHandToolMatrix,robot)
    # verif.record()

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



"""
start = time.clock()
print start

while linalg.norm(array(taskRH.feature.error.value)[0:3]) > pos_err_des:
	robot.increment(dt)
	attime.run(robot.control.time)
	updateComDisplay(robot,dyn.com)
	updateToolDisplay(taskRH,RHToTwoHandToolMatrix,robot)

print "pos_err= "+str(linalg.norm(array(taskRH.feature.error.value)[0:3]))
print "execution time = " +str(robot.control.time)

stop = time.clock()
print stop

print "calculous time = " + str(stop-start)
"""
