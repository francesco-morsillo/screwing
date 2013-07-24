# ______________________________________________________________________________
# ******************************************************************************
#
#    FEASIBILITY STUDY
#       Robot: HRP-2 N.14
#       Tasks: Simulation of a sequence of screwings to random points of the montage structure
# 
# ______________________________________________________________________________
# ******************************************************************************  



from dynamic_graph.sot.core.utils.thread_interruptible_loop import *

from dynamic_graph.sot.fmorsill.openHRP.screw_2ht import screw_2ht
from dynamic_graph.sot.fmorsill.openHRP.get_2ht import get_2ht
from dynamic_graph.sot.fmorsill.utility import *
from numpy import array, linalg, pi


pos_err_des = 0.00001

def stuck(mTask):
    #if linalg.norm(array(mTask.feature.error.value)[0:3] - mTask.old_err) < pos_err_des:
    if math.isnan(mTask.feature.error.value[0]):
        res = True
    else: res = False

    mTask.old_err = array(mTask.feature.error.value)[0:3]
    return res

def FSM(state):

    print "State: " + state

    if state == 'tryF':
        robot.mTasks['screw'].feature.error.recompute(robot.control.time)
        if linalg.norm(array(robot.mTasks['screw'].feature.error.value)[0:3]) < pos_err_des:
            print "Error: " + (robot.mTasks['screw'].feature.error.value)
            print "\nSUCCESS \n"
            state = 'success'
            get_2ht(robot,solver,tool)
            
        elif stuck(robot.mTasks['screw']):
            print "\nFAILURE \n"
            state =  'failure'
            get_2ht(robot,solver,tool)
            
                
    if state == 'success':
        robot.mTasks['lh'].feature.error.recompute(robot.control.time)
        if linalg.norm(array(robot.mTasks['lh'].feature.error.value)[0:3]) < pos_err_des:
            print "Error: " + (robot.mTasks['lh'].feature.error.value)
            print "\nTRY \n"
            state = 'tryF'
            screw_2ht(robot,solver,tool,goal)
                        
                        
    if state == 'failure':
        robot.mTasks['lh'].feature.error.recompute(robot.control.time)
        if linalg.norm(array(robot.mTasks['lh'].feature.error.value)[0:3]) < pos_err_des:
            print "Error: " + (robot.mTasks['lh'].feature.error.value)
            print "\nTRY \n"
            state = 'tryF'
            screw_2ht(robot,solver,tool,goal)



def supervisor(robot, solver):

    ### Start script
    tool = (0.4,-0.1,0.9,0.,0.,pi/2)
    get_2ht(robot,solver,tool)

    goal = array([0.55,-0.2,0.9,0.,1.57,0.])
    
    state = 'success' #to initialize I need to go to get_2htt position


    @loopInThread
    def Runner():
        FSM(state)

    runner = Runner()
    runner.play()

    return (runner,state)
