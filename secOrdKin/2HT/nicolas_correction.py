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

from dynamic_graph.sot.core.meta_task_posture import *

from dynamic_graph.sot.core.matrix_util import *
from dynamic_graph.sot.core.utils.viewer_helper import addRobotViewer,VisualPinger,updateComDisplay
from dynamic_graph.sot.core.utils.thread_interruptible_loop import *
from dynamic_graph.sot.core.utils.attime import attime

from dynamic_graph.sot.dyninv.robot_specific import pkgDataRootDir, modelName, robotDimension, initialConfig, gearRatio, inertiaRotor, halfSittingConfig
from dynamic_graph.sot.dyninv.meta_task_dyn_6d import MetaTaskDyn6d
from dynamic_graph.sot.dyninv.meta_tasks_dyn import MetaTaskDynCom, MetaTaskDynPosture, AddContactHelper, gotoNd
from dynamic_graph.sot.dyninv.meta_tasks_dyn_relative import MetaTaskDyn6dRel, gotoNdRel, goto6dRel

from dynamic_graph.sot.fmorsill.utility import *
from dynamic_graph.sot.fmorsill.rob_view_lib import *

from numpy import *


# ------------------------------------------------------------------------------
# --- ROBOT DYNAMIC SIMULATION -------------------------------------------------
# ------------------------------------------------------------------------------

robotName = 'hrp14small'
robotDim   = robotDimension[robotName]
RobotClass = RobotDynSimu
robot      = RobotClass("robot")
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
zTool = 1.
rollTool = 0.
pitchTool = pi/5
yawTool = pi/2


# Initial configuration
# We give the absolute position of the robot and then extract the relative rototranslation to impose to the tool
x0=0#-1.6
y0=0.#1.8
z0=0.64870185118253043
roll0=0.
pitch0=0.
yaw0=0.#pi/2

# Center initial configuration
pose = (0.00890154,0.0486058,0.683817,-2.99066e-20,4.01579e-20,-0.0248718,0.0248718,-0.0835992,-0.2366,0.46607,-0.22947,0.0835992,0.0248718,-0.0839582,-0.27058,0.544286,-0.273706,0.0839582,-0.110689,-0.0869153,-1.09935e-17,-5.52732e-19,-0.225138,-0.628069,0.730439,-1.61574,1.54156,0.443013,0.174533,-0.239188,-0.164848,-0.741815,-1.01466,-0.538398,0.504463,0.174533)
#write_xml("/opt/grx3.0/HRP2LAAS/project/",pose)
#write_pos_py("/opt/grx3.0/HRP2LAAS/script/airbus_robot/",pose[6:36])
robot.set(pose)

# Screw Lenght
screw_len = 0.

# TwoHandTool RotoTranslation / Homogeneous Matrix of the TwoHandTool. Normally given from the camera
RTMatrix = array(RPYToMatrix((x0-xr,y0-yr,z0-zr,roll0-rollr,pitch0-pitchr,yaw0-yawr)))
refToTwoHandToolMatrix = dot( RTMatrix , array(RPYToMatrix((xTool,yTool,zTool,rollTool,pitchTool,yawTool))) )



#goals
goal4 = array([0.55,-0.2,1.15,0.,1.57,0.])
goal3 = array([0.55,-0.3,1.15,0.,1.57,0.])
goal1 = array([0.55,-0.3,1.,0.,1.57,0.])
goal2 = array([0.55,-0.2,1.,0.,1.57,0.])

goal = array([goal1,goal2,goal3,goal4])


# visualization
addRobotViewer(robot,small=True,verbose=True)
robot.viewer.updateElementConfig('TwoHandTool',vectorToTuple( array(matrixToRPY(refToTwoHandToolMatrix)) ))
robot.viewer.updateElementConfig('zmp',vectorToTuple(array(matrixToRPY( dot(refToTwoHandToolMatrix,TwoHandToolToScrewMatrix) ))))
for i in range(4):
	robot.viewer.updateElementConfig('goal'+str(i+1),vectorToTuple(goal[i]))


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

taskWaist = MetaTaskDyn6d('taskWaist', dyn, 'waist', 'waist')
taskChest = MetaTaskDyn6d('taskChest', dyn, 'chest', 'chest')
taskHead  = MetaTaskDyn6d('taskHead', dyn, 'head', 'gaze')
taskRH    = MetaTaskDyn6d('rh', dyn, 'rh', 'right-wrist')
taskLH    = MetaTaskDyn6d('lh', dyn, 'lh', 'left-wrist')
taskRel = MetaTaskDyn6dRel('rel',dyn,'rh','lh','right-wrist','left-wrist')

for task in [taskHead, taskRH, taskLH, taskRel, taskWaist, taskChest]:
    task.feature.frame('current')
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
taskPosture2 = MetaTaskKinePosture(dyn,"posture2")

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

#-----------------------------------------------------------------------------
# --- Stack of tasks controller  ---------------------------------------------
#-----------------------------------------------------------------------------

sot = SolverKine('sot')

sot.setSize(robotDim)

plug(sot.control, robot.control)
plug(sot.control, robot.acceleration)


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
tr.start()

robot.after.addSignal('tr.triger')
robot.after.addSignal('dyn.com')
robot.after.addSignal('taskLim.normalizedPosition')

tr.add('taskLim.normalizedPosition','qn')
tr.add('dyn.com','')

"""
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



#-----------------------------------------------------------------------------
# --- RUN --------------------------------------------------------------------
#-----------------------------------------------------------------------------

# Task references
dyn.com.recompute(0)
taskCom.featureDes.errorIN.value = dyn.com.value
taskCom.task.controlGain.value = 10

taskPosture.ref = halfSittingConfig[robotName]
#taskPosture.ref = initialConfig[robotName]
#taskPosture.ref = pose

#taskPosture2.gotoq(10,pose)

#Not meaningfull, use feature (not des) to set it up (nicolas): taskCom.featureDes.selec.value = '000011'
taskCom.gain.setConstant(0.1)

#RH-TwoHandTool Homogeneous Transformation Matrix (fixed in time)
taskRH.feature.position.recompute(0)
taskLH.feature.position.recompute(0)
RHToTwoHandToolMatrix = dot(linalg.inv(array(taskRH.feature.position.value)),refToTwoHandToolMatrix)
#!!!!!! RH and Support are different references, because the X rotation is not controlled in positioning!!!!!!!!!!!!!!!!!!!!!!!!!!
RHToScrewMatrix = dot(RHToTwoHandToolMatrix,TwoHandToolToScrewMatrix)

# TASK SCREW creation
taskScrew=MetaTaskDyn6d('screw',dyn,'screw','right-wrist'); taskScrew.opmodif = matrixToTuple(dot(handMgrip,RHToScrewMatrix))
taskScrew.featureDes.velocity.value=(0,0,0,0,0,0)
gotoNd(taskScrew, goal1, "011111",(100,5,0.01,0.9))

gotoNdRel(taskRel,array(taskRH.feature.position.value),array(taskLH.feature.position.value),'110111',1000)
taskRel.feature.errordot.value=(0,0,0,0,0)	# not to forget!!

###################################################################################################################


gotoNd(taskChest,(0.,0.,0.,0.,0.1,0.),'010000')	# inside the function rot=0 --> we set a random position not to control it
taskChest.gain.setConstant(1)

gotoNd(taskWaist,(0.,0.,0.,0.,0.,0.),'010000')	# inside the function rot=0 --> we set a random position not to control it
taskWaist.gain.setConstant(1)


"""
dyn.createOpPoint('waist','waist')
featureWaist    = FeaturePoint6d('featureWaist')
plug(dyn.waist,featureWaist.position)
plug(dyn.Jwaist,featureWaist.Jq)
taskWaist=TaskInequality('taskWaist')
taskWaist.add(featureWaist.name)
taskWaist.selec.value = '010000'
taskWaist.referenceInf.value = (0.,-0.1,0,0,0,0)    # Xmin, Ymin
taskWaist.referenceSup.value = (0.,0.5,0,0,0,0)  # Xmax, Ymax
taskWaist.dt.value=dt


dyn.createOpPoint('chest','chest')
featureChest    = FeaturePoint6d('featureChest')
plug(dyn.chest,featureChest.position)
plug(dyn.Jchest,featureChest.Jq)
taskChest=TaskInequality('taskChest')
taskChest.add(featureChest.name)
taskChest.selec.value = '010000'
taskChest.referenceInf.value = (0.,-0.5,0,0,0,0)    # Xmin, Ymin
taskChest.referenceSup.value = (0.,0.,0,0,0,0)  # Xmax, Ymax
taskChest.dt.value=dt
"""

###################################################################################################################

# Sot
sot.clear()
sot.addContact(contactLF)
sot.addContact(contactRF)
sot.push(taskLim.name)
sot.push(taskRel.task.name)
sot.push(taskScrew.task.name)
sot.push(taskCom.task.name)
sot.push(taskWaist.task.name)
sot.push(taskChest.task.name)
sot.push(taskPosture.task.name)

# Motion recording
zmp_out = open("/tmp/data.zmp","w")
hip_out = open("/tmp/data.hip","w")
pos_out = open("/tmp/data.pos","w")

def do():
	robot.increment(dt)
	attime.run(robot.control.time)
	updateComDisplay(robot,dyn.com)
	updateToolDisplay(taskScrew,linalg.inv(TwoHandToolToScrewMatrix),robot)
	record_zmp(robot,dyn,zmp_out,dt)
	record_hip(robot,dyn,hip_out,dt)
	record_pos(robot,dyn,pos_out,dt)

def go_to(goal,pos_err_des,screw_len):

	# Goal HM
	refToGoalMatrix = eye(4); refToGoalMatrix[0:3,0:3]=calcRotFromRPY(goal[3],goal[4],goal[5]); refToGoalMatrix[0:3,3]=goal[0:3]

	# Preparation position
	preparationMatrix = dot(refToGoalMatrix,array([[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,-0.03-screw_len],[0.,0.,0.,1.]]))

	# mini-task sequence definition
	action = array([preparationMatrix,refToGoalMatrix,preparationMatrix])

	for i in range(1):

		# Goal display
		robot.viewer.updateElementConfig('zmp',vectorToTuple(action[i][0:3,3])+(0.,0.,0.))
		
		# Aim setting
		taskScrew.ref = matrixToTuple(action[i])
		do()
		for i in range(800): #		while linalg.norm(array(taskScrew.feature.error.value)[0:3]) > pos_err_des:
			do()

#sot.rm(taskCom.task.name)
sot.rm(taskWaist.task.name)
sot.rm(taskChest.task.name)
#sot.rm(taskLim.name)
#taskLim.controlGain.value = 1e9
taskLim.dt.value = 1.0/200.0 / 0.05
taskLim.referenceVelInf.value = (-1000,)*36
taskLim.referenceVelSup.value = (+1000,)*36
taskCom.task.controlGain.value = 0.01
#taskPosture.task.controlGain.value = 10
sot.up(taskCom.task.name)
sot.up(taskCom.task.name)
sot.up(taskCom.task.name)
sot.up(taskCom.task.name)
sot.up(taskCom.task.name)

for i in range(1):
	go_to(goal[i],pos_err_des,screw_len)



