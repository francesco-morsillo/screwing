
from dynamic_graph.sot.screwing.acc_control_functions import get_2ht, screw_2ht, goToHalfSitting, closeGrippers, openGrippers
from numpy import pi, array

class WrapperInt:
    def __init__(self,x):
        self.value = x

    def incr(self,n):
        self.value += n
        return self.value


tool = (0.4,-0.1,0.8,0.,0.,pi/2)

P72 = (0.75,-0.45,1.05,0.,0.,1.57)

goal1 = array(P72) + array([-0.22,0.2,0.2,0.,1.57,-1.57])
goal2 = array(P72) + array([-0.22,0.1,0.25,0.,1.57,-1.57])
goal3 = array(P72) + array([-0.22,0.,0.2,0.,1.57,-1.57])

goal10 = array(P72) + array([-0.2,0.2,-0.2,0.,1.57,-1.57])
goal9 = array(P72) + array([-0.2,0.1,-0.25,0.,1.57,-1.57])
goal8 = array(P72) + array([-0.2,0.,-0.2,0.,1.57,-1.57])

goal4 = goal3 + 0.2*(goal8-goal3)
goal5 = goal3 + 0.4*(goal8-goal3)
goal6 = goal3 + 0.6*(goal8-goal3)
goal7 = goal3 + 0.8*(goal8-goal3)

goal11 = goal10 + 0.2*(goal1-goal10)
goal12 = goal10 + 0.4*(goal1-goal10)
goal13 = goal10 + 0.6*(goal1-goal10)
goal14 = goal10 + 0.8*(goal1-goal10)

goal = array([goal9,goal10,goal11,goal12,goal13,goal14,goal1,goal2,goal3,goal4,goal5,goal6,goal7,goal8])

i = WrapperInt(-1)
