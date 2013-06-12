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
pose = (-0.030073618647834879, 0.016415887002202933, 0.62245944857810387, -5.4272918219751486e-20, 2.2304361400809676e-20, 0.0092340515812077348, -0.0092340515812077365, -0.031684365127837365, -0.5963821161312286, 1.0426697092875947, -0.44628759315636635, 0.031684365127837365, -0.0092340515812077348, -0.03174248663241639, -0.60591626591827519, 1.0568307464403983, -0.45091448052212324, 0.03174248663241639, -0.011123297511703737, -0.087265999999999927, -0.00033750538179865144, -0.0005624238088200199, -0.30213952018426748, -0.71906866277071557, 0.70978391033338306, -1.7041087671646877, 1.5030587263678217, 0.48707940835840557, 0.17569313715492132, -0.38348731360690541, -0.17453299999999988, -0.80343898121984014, -1.041280119185096, -0.51958844641260804, 0.54990959413946805, 0.17519281104793244)
#write_xml("/opt/grx3.0/HRP2LAAS/project/",pose)
write_pos_py("/opt/grx3.0/HRP2LAAS/script/airbus_robot/",pose[6:36])
robot.set(pose)

# Screw Lenght
screw_len = 0.03

# TwoHandTool RotoTranslation / Homogeneous Matrix of the TwoHandTool. Normally given from the camera
RTMatrix = array(RPYToMatrix((x0-xr,y0-yr,z0-zr,roll0-rollr,pitch0-pitchr,yaw0-yawr)))
refToTwoHandToolMatrix = dot( RTMatrix , array(RPYToMatrix((xTool,yTool,zTool,rollTool,pitchTool,yawTool))) )


#goals
goal3 = array([0.55,-0.2,0.9,0.,1.57,0.])
goal4 = array([0.55,-0.3,0.9,0.,1.57,0.])
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


# Angular position and velocity limits
taskLim = TaskDynLimits('taskLim')
plug(dyn.position,taskLim.position)
plug(dyn.velocity,taskLim.velocity)
taskLim.dt.value = dt

dyn.upperJl.recompute(0)
dyn.lowerJl.recompute(0)
taskLim.referencePosInf.value = dyn.lowerJl.value
taskLim.referencePosSup.value = dyn.upperJl.value

taskLim.controlGain.value = 1

#dqup = (0, 0, 0, 0, 0, 0, 200, 220, 250, 230, 290, 520, 200, 220, 250, 230, 290, 520, 250, 140, 390, 390, 240, 140, 240, 130, 270, 180, 330, 240, 140, 240, 130, 270, 180, 330)
dqup = (1000,)*robotDim
taskLim.referenceVelInf.value = tuple([-val*pi/180 for val in dqup])
taskLim.referenceVelSup.value = tuple([ val*pi/180 for val in dqup])

###
featureHeight = FeatureGeneric('featureHeight')
plug(dyn.com,featureHeight.errorIN)
plug(dyn.Jcom,featureHeight.jacobianIN)



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

robot.after.addSignal('tr.triger')
robot.after.addSignal('dyn.com')
robot.after.addSignal('taskLim.normalizedPosition')

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
#taskPosture.ref = pose

taskCom.feature.selec.value = '011'
taskCom.gain.setConstant(0.1)

#RH-TwoHandTool Homogeneous Transformation Matrix (fixed in time)
taskRH.feature.position.recompute(0)
taskLH.feature.position.recompute(0)
RHToTwoHandToolMatrix = dot(linalg.inv(array(taskRH.feature.position.value)),refToTwoHandToolMatrix)
#!!!!!! RH and Height are different references, because the X rotation is not controlled in positioning!!!!!!!!!!!!!!!!!!!!!!!!!!
RHToScrewMatrix = dot(RHToTwoHandToolMatrix,TwoHandToolToScrewMatrix)

# TASK SCREW creation
taskScrew=MetaTaskDyn6d('screw',dyn,'screw','right-wrist'); taskScrew.opmodif = matrixToTuple(dot(handMgrip,RHToScrewMatrix))
taskScrew.featureDes.velocity.value=(0,0,0,0,0,0)
taskScrew.feature.selec = '011111'
taskScrew.gain.setByPoint((100,5,0.01,0.9))

#gotoNd(taskScrew, goal1, "011111",(100,5,0.01,0.9))

gotoNdRel(taskRel,array(taskRH.feature.position.value),array(taskLH.feature.position.value),'110111',1000)
taskRel.feature.errordot.value=(0,0,0,0,0)	# not to forget!!

###################################################################################################################


gotoNd(taskChest,(0.,0.,0.,0.,0.1,0.),'110000')	# inside the function rot=0 --> we set a random position not to control it
taskChest.gain.setConstant(1)

gotoNd(taskWaist,(0.,0.,0.,0.,0.,0.),'110000')	# inside the function rot=0 --> we set a random position not to control it
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

	for i in range(3):

		# Goal display
		robot.viewer.updateElementConfig('zmp',vectorToTuple(action[i][0:3,3])+(0.,0.,0.))
		
		# Aim setting
		taskScrew.ref = matrixToTuple(action[i])
		do()
		while linalg.norm(array(taskScrew.feature.error.value)[0:3]) > pos_err_des: #for j in range(1): #
			do()

for i in range(4):
	go_to(goal[i],pos_err_des,screw_len)



