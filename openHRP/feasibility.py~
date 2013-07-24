# ______________________________________________________________________________
# ******************************************************************************
#
#    FEASIBILITY STUDY
#       Robot: HRP-2 N.14
#       Tasks: Simulation of a sequence of screwings to random points of the montage structure
# 
# ______________________________________________________________________________
# ******************************************************************************  


from dynamic_graph.sot.fmorsill.openHRP.screw_2ht import screw_2ht

from numpy import array, linalg


pos_err_des = 0.00001

def stuck(mTask):
    if linalg.norm(array(mTask.feature.error.value)[0:3] - mTask.old_err) < pos_err_des:
        res = True
    else: res = False

    mTask.old_err = array(mTask.feature.error.value)[0:3]
    return res


class TryF:

    mTask_ = None
    
    def __init__(self,mTask):
        self.mTask_ = mTask
        # need to initialize old_err for the failure case
        # using zero assures that this value will never cause an immediate changing to the failure case, because if error is near to zero we fall in the success case
        self.mTask_.old_err = array([0.,0.,0.])

    # check if reached condition of state changing
    def check_cond(self):        
        # success check
        print str(self)
        self.mTask_.feature.error.recompute(0)
        if linalg.norm(array(self.mTask_.feature.error.value)[0:3]) < pos_err_des:
            print "\n\n SUCCESS \n"
            return (True, 'success')
        elif stuck(self.mTask_):
            print "\n\n FAILURE \n"
            return (True, 'failure')
        else :
            return (False, 'tryF')  #Unuseful return of tryF




class FSM:
    state_ = ''
    operator_ = dict()

    def __init__(self,initial_state, mTask):
        self.state_ = initial_state
        self.mTask_ = mTask
        self.operator_['try'] = TryF(self.mTask_)

    def check_cond(self):
        return  self.operator_[self.state_].check_cond()

    def state_evaluation(self):
        (res, new_state) = check_cond()
        if res : self.state_ = new_state


#def feasibility_try():
    
