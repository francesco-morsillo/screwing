
from dynamic_graph.sot.screwing.vel_control_functions import get_2ht, screw_2ht, goToHalfSitting
from numpy import pi, array

tool = (0.4,-0.1,0.9,0.,0.,pi/2)

P72 = (0.7,0.,0.9,0.,0.,1.57)

limit1 = array(P72) + array([-0.23,-0.3,0.23,0.,1.57,-1.57])
limit2 = array(P72) + array([-0.22,0.3,0.32,0.,1.57,-1.57])
limit3 = array(P72) + array([-0.22,0.3,-0.33,0.,1.57,-1.57])
limit4 = array(P72) + array([-0.23,-0.3,-0.38,0.,1.57,-1.57])

goal1 = limit1
goal2 = limit1 + 0.3*(limit4-limit1)
goal3 = limit1 + 0.7*(limit4-limit1)
goal4 = limit1 + 1.0*(limit4-limit1)

goal = array([goal1,goal2,goal3,goal4])

i=0
