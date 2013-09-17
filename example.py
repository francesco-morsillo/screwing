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

from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.utils.viewer_helper import addRobotViewer,VisualPinger,updateComDisplay
from dynamic_graph.sot.core.utils.thread_interruptible_loop import *
from dynamic_graph.sot.core.utils.attime import attime

from dynamic_graph.sot.dyninv.robot_specific import pkgDataRootDir, modelName, robotDimension, initialConfig, gearRatio, inertiaRotor, halfSittingConfig
from dynamic_graph.sot.dyninv.meta_task_dyn_6d import MetaTaskDyn6d
from dynamic_graph.sot.dyninv.meta_tasks_dyn import MetaTaskDynCom, MetaTaskDynPosture, AddContactHelper, gotoNd

from dynamic_graph.sot.core.utils.history import History

from numpy import *

############################################################
###   CHOICE OF FIRST OR SECOND ORDER
############################################################
#-------------------------------------------------------------------------
# VELOCITY CONTROL
#from dynamic_graph.sot.application.velocity.precomputed_meta_tasks import initialize
#from dynamic_graph.sot.screwing.vel_control_functions import moveRightHandToTarget

# ACCELERATION CONTROL
from dynamic_graph.sot.application.acceleration.precomputed_tasks import initialize
from dynamic_graph.sot.screwing.acc_control_functions import moveRightHandToTarget
#-------------------------------------------------------------------------





class Robot:
    device = None
    dynamic = None
    timeStep = 0.005

    def __init__(self, device=None, dynamic=None, dt=0.005):
        self.device = device
        self.dynamic = dynamic
        self.timeStep = dt



# ------------------------------------------------------------------------------
# --- ROBOT DYNAMIC SIMULATION -------------------------------------------------
# ------------------------------------------------------------------------------

robot = Robot()

robotName = 'hrp14small'
robot.dimension   = robotDimension[robotName]
RobotClass = RobotSimu
robot.device      = RobotClass("robot")
robot.device.resize(robot.dimension)

x0=0.
y0=0.
z0=0.64870185118253043
robot.halfSitting = (x0,y0,z0,0,0,0)+halfSittingConfig[robotName][6:]
pose = robot.halfSitting

robot.device.set( pose )

addRobotViewer(robot.device,small=True,verbose=False)

robot.timeStep=5e-3

#-----------------------------------------------------------------------------
#---- DYN --------------------------------------------------------------------
#-----------------------------------------------------------------------------

modelDir          = pkgDataRootDir[robotName]
xmlDir            = pkgDataRootDir[robotName]
specificitiesPath = xmlDir + '/HRP2SpecificitiesSmall.xml'
jointRankPath     = xmlDir + '/HRP2LinkJointRankSmall.xml'

robot.dynamic = Dynamic("dyn")
robot.dynamic.setFiles(modelDir,modelName[robotName],specificitiesPath,jointRankPath)
robot.dynamic.parse()

robot.dynamic.inertiaRotor.value = inertiaRotor[robotName]
robot.dynamic.gearRatio.value    = gearRatio[robotName]

plug(robot.device.state,robot.dynamic.position)
robot.dynamic.velocity.value = robot.dimension*(0.,)
robot.dynamic.acceleration.value = robot.dimension*(0.,)

robot.dynamic.ffposition.unplug()
robot.dynamic.ffvelocity.unplug()
robot.dynamic.ffacceleration.unplug()

robot.dynamic.setProperty('ComputeBackwardDynamics','true')
robot.dynamic.setProperty('ComputeAccelerationCoM','true')

robot.device.control.unplug()

# ------------------------------------------------------------------------------
# --- SOLVER ----------------------------------------------------------------
# ------------------------------------------------------------------------------

# initialize in acceleration mode
solver = initialize(robot)

# ------------------------------------------------------------------------------
# --- MAIN LOOP ----------------------------------------------------------------
# ------------------------------------------------------------------------------

def inc():
    robot.device.increment(robot.timeStep)
    attime.run(robot.device.control.time)
    updateComDisplay(robot.device,robot.dynamic.com)

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
def iter():         print 'iter = ',robot.device.state.time
@optionalparentheses
def status():       print runner.isPlay

"""
#-----------------------------------------------------------------------------
# --- TRACER ------------------------------------------------------------------
#-----------------------------------------------------------------------------
from dynamic_graph.tracer import *
tr = Tracer('tr')
tr.open('/tmp/','','.dat')
tr.start()
robot.after.addSignal('tr.triger')

dyn = robot.dynamic

tr.add('dyn.com','com2')
tr.add(taskRH.task.name+'.error','erh2')

tr.add(taskLim.name+".normalizedPosition","qn")
robot.after.addSignal(taskLim.name+".normalizedPosition")

tr.add("dyn.position","q")
robot.after.addSignal("dyn.position")
tr.add("dyn.velocity","dq")
robot.after.addSignal("dyn.velocity")
tr.add("dyn.acceleration","ddq")
robot.after.addSignal("dyn.acceleration")
"""

#-----------------------------------------------------------------------------
# --- RUN --------------------------------------------------------------------
#-----------------------------------------------------------------------------

# RH metaTask
target = (0.5,-0.2,1.3)
robot.device.viewer.updateElementConfig('zmp',target+(0,0,0))
moveRightHandToTarget(robot,solver,target,50)


### EXPERIMENT
"""
solver.rm(robot.mTasks['posture'].task)
from dynamic_graph.sot.dyninv.meta_tasks_dyn import MetaTaskDynPosture

robot.mTasks['posture1'] = MetaTaskDynPosture(robot.dynamic,robot.timeStep,'posture1')
robot.mTasks['posture1'].gotoq(10,array(robot.halfSitting),rknee=[],lknee=[],larm=[],rwrist=[],chest=[],head=[])

robot.mTasks['posture2'] = MetaTaskDynPosture(robot.dynamic,robot.timeStep,'posture2')
robot.mTasks['posture2'].gotoq(10,array(robot.halfSitting),rhip=[],rankle=[],lhip=[],lankle=[],rshoulder=[],relbow=[],rhand=[],lhand=[])

solver.push(robot.mTasks['posture1'].task)
solver.push(robot.mTasks['posture2'].task)
"""

"""
robot.mTasks['posturerleg'] = MetaTaskDynPosture(robot.dynamic,robot.timeStep,'posturerleg')
robot.mTasks['posturerleg'].gotoq(1,array(robot.halfSitting),rleg=[])

robot.mTasks['posturelleg'] = MetaTaskDynPosture(robot.dynamic,robot.timeStep,'posturelleg')
robot.mTasks['posturelleg'].gotoq(1,array(robot.halfSitting),lleg=[])

robot.mTasks['posturechest'] = MetaTaskDynPosture(robot.dynamic,robot.timeStep,'posturechest')
robot.mTasks['posturechest'].gotoq(1,array(robot.halfSitting),chest=[])

robot.mTasks['posturehead'] = MetaTaskDynPosture(robot.dynamic,robot.timeStep,'posturehead')
robot.mTasks['posturehead'].gotoq(1,array(robot.halfSitting),head=[])

robot.mTasks['posturerarm'] = MetaTaskDynPosture(robot.dynamic,robot.timeStep,'posturerarm')
robot.mTasks['posturerarm'].gotoq(1,array(robot.halfSitting),rarm=[],rhand=[])

robot.mTasks['posturelarm'] = MetaTaskDynPosture(robot.dynamic,robot.timeStep,'posturelarm')
robot.mTasks['posturelarm'].gotoq(1,array(robot.halfSitting),larm=[],lhand=[])


#solver.push(robot.mTasks['posturerleg'].task)
#solver.push(robot.mTasks['posturelleg'].task)
#solver.push(robot.mTasks['posturechest'].task)
#solver.push(robot.mTasks['posturerarm'].task)
#solver.push(robot.mTasks['posturelarm'].task)
#solver.push(robot.mTasks['posturehead'].task)

attime(2
       ,(lambda: solver.push(robot.mTasks['posturelarm'].task), "Add LArm to posture")
       ,(lambda: solver.push(robot.mTasks['posturerleg'].task), "Add RLeg to posture")
       ,(lambda: solver.push(robot.mTasks['posturelleg'].task), "Add LLeg to posture")
       )

attime(10
       ,(lambda: solver.push(robot.mTasks['posturerarm'].task), "Add RArm to posture")
       ,(lambda: solver.push(robot.mTasks['posturechest'].task), "Add Chest to posture")
       ,(lambda: solver.push(robot.mTasks['posturehead'].task), "Add Head to posture")
       )
"""


"""
rarmDes = initialConfig[robotName][22:28]
larmDes = initialConfig[robotName][29:35]
taskPosture.gotoq(10) #rarm=rarmDes,larm=larmDes)

"""
