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
RobotClass = RobotSimu
robot      = RobotClass("robot")
robot.resize(robotDim)

dt=5e-3

robot.setSecondOrderIntegration()

# Robot and TwoHandTool in the origin
xr=0.
yr=0.
zr=0.64870185118253043
rollr=0.
pitchr=0.
yawr=0.

xTool = 0.4
yTool = -0.2
zTool = 0.9
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

#grip_angle = 0.17569313715492132
grip_angle_rh = 0.11
grip_angle_sh = 0.05

# Center initial configuration
pose = (-0.040171724537852692, 0.012554979677786016, 0.60038972621653741, -1.8324817401555846e-19, 3.3881986601669945e-19, -4.9281601434382094e-19, 8.7466141738804274e-19, -0.025315586588404569, -0.68866188267197115, 1.1779122640040864, -0.48925038133211551, 0.025315586588404573, 3.8928477315524647e-19, -0.02536092979672944, -0.69429072446021389, 1.1884597599934046, -0.49416903553319086, 0.025360929796729451, 3.2487817247183095e-20, -9.8883744822907284e-21, 0.0052206037085911327, -0.015577636215755253, -0.35409243163848308, -0.45866960043208704, 0.90541127100176388, -1.4009012738515936, 1.6057030000000001, 0.70851947851179609, grip_angle_rh, -0.63488862696875059, -0.17453299999999999, -1.0736852932781615, -0.70107411555135457, -0.120960197588031, 0.36479488979894936, grip_angle_sh)
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

taskRH    = MetaTaskDyn6d('rh', dyn, 'rh', 'right-wrist')
taskLH    = MetaTaskDyn6d('lh', dyn, 'lh', 'left-wrist')
taskRel = MetaTaskDyn6dRel('rel',dyn,'rh','lh','right-wrist','left-wrist')

for task in [taskRH, taskLH, taskRel]:
    task.feature.frame('current')
    task.task.dt.value = dt
    task.featureDes.velocity.value=(0,0,0,0,0,0)

handMgrip=eye(4); handMgrip[0:3,3] = (0,0,-0.14)
taskRH.opmodif = matrixToTuple(handMgrip)
taskLH.opmodif = matrixToTuple(handMgrip)

# RELATIVE POSITION TASK
taskRel.opmodif = matrixToTuple(handMgrip)
taskRel.opmodifBase = matrixToTuple(handMgrip)

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



#-----------------------------------------------------------------------------
# --- Stack of tasks controller  ---------------------------------------------
#-----------------------------------------------------------------------------

sot = SolverKine('sot')

sot.setSize(robotDim)

sot.setSecondOrderKine(True)
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


#-----------------------------------------------------------------------------
# --- RUN --------------------------------------------------------------------
#-----------------------------------------------------------------------------

######################################################################
###------TASK EQUALITY------------------------------------------------
######################################################################

#RH-TwoHandTool Homogeneous Transformation Matrix (fixed in time)
taskRH.feature.position.recompute(0)
taskLH.feature.position.recompute(0)
RHToTwoHandToolMatrix = dot(linalg.inv(array(taskRH.feature.position.value)),refToTwoHandToolMatrix)
#!!!!!! RH and Height are different references, because the X rotation is not controlled in positioning!!!!!!!!!!!!!!!!!!!!!!!!!!
RHToScrewMatrix = dot(RHToTwoHandToolMatrix,TwoHandToolToScrewMatrix)

# TASK Screw
taskScrew=MetaTaskDyn6d('screw',dyn,'screw','right-wrist'); taskScrew.opmodif = matrixToTuple(dot(handMgrip,RHToScrewMatrix))
taskScrew.featureDes.velocity.value=(0,0,0,0,0,0)
taskScrew.feature.selec.value = '011111'
taskScrew.gain.setByPoint(100,1,0.01,0.9)

# Task Relative
gotoNdRel(taskRel,array(taskRH.feature.position.value),array(taskLH.feature.position.value),'110111',500)
taskRel.feature.errordot.value=(0,0,0,0,0)	# not to forget!!

# Task Posture
taskPosture = MetaTaskDynPosture(dyn,dt)
#taskPosture.ref = halfSittingConfig[robotName]
taskPosture.ref = pose
taskPosture.gain.setConstant(5)

# Task Com
taskCom = MetaTaskDynCom(dyn,dt)
dyn.com.recompute(0)
taskCom.featureDes.errorIN.value = dyn.com.value
taskCom.feature.selec.value = '011'
taskCom.gain.setConstant(1)


######################################################################
###------TASK INEQUALITY----------------------------------------------
######################################################################

#Task Waist
dyn.createOpPoint('waist','waist')
featureWaist    = FeaturePoint6d('featureWaist')
plug(dyn.waist,featureWaist.position)
plug(dyn.Jwaist,featureWaist.Jq)
taskWaist=TaskDynInequality('taskWaist')
plug(dyn.velocity,taskWaist.qdot)
taskWaist.add(featureWaist.name)
taskWaist.selec.value = '110000'
taskWaist.referenceInf.value = (0.,0.,0.,0.,-0.2,-0.7)    # Roll Pitch Yaw min
taskWaist.referenceSup.value = (0.,0.,0.,0.,0.5,0.7)  # Roll Pitch Yaw max
taskWaist.dt.value=dt
taskWaist.controlGain.value = 10

#Task Chest
dyn.createOpPoint('chest','chest')
featureChest    = FeaturePoint6d('featureChest')
plug(dyn.chest,featureChest.position)
plug(dyn.Jchest,featureChest.Jq)
taskChest=TaskDynInequality('taskChest')
plug(dyn.velocity,taskChest.qdot)
taskChest.add(featureChest.name)
taskChest.selec.value = '110000'
taskChest.referenceInf.value = (0.,0.,0.,0.,-0.2,-0.7)    # Roll Pitch Yaw min 
taskChest.referenceSup.value = (0.,0.,0.,0.,0.3,0.7)   # Roll Pitch Yaw max
taskChest.dt.value=dt
taskChest.controlGain.value = 10


######################################################################
###------SOT CHARGING-------------------------------------------------
######################################################################

sot.clear()
sot.addContact(contactLF)
sot.addContact(contactRF)
sot.push(taskLim.name)
sot.push(taskRel.task.name)
sot.push(taskCom.task.name)
sot.push(taskScrew.task.name)
sot.push(taskWaist.name)
sot.push(taskChest.name)
sot.push(taskPosture.task.name)

sot.damping.value = 0.1

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
	desired_error = array([pos_err_des*10,pos_err_des,pos_err_des*10])

	for i in range(3):

		# Goal display
		robot.viewer.updateElementConfig('zmp',vectorToTuple(action[i][0:3,3])+(0.,0.,0.))
		
		# Aim setting
		taskScrew.ref = matrixToTuple(action[i])
		do()
		while linalg.norm(array(taskScrew.feature.error.value)[0:3]) > desired_error[i]:
			do()

for i in range(4):
	go_to(goal[i],pos_err_des,screw_len)



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
