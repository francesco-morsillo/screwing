from dynamic_graph import plug

from dynamic_graph.sot.screwing.vel_control_functions import follow3DPoint

from dynamic_graph.sot.core.math_small_entities import Multiply_of_matrixHomo, Add_of_vector
from dynamic_graph.sot.core.matrix_util import vectorToTuple, matrixToTuple

"""
target = plugObject(robot)
follow3DPoint(robot,solver,target,2)
"""

VECTOR2SIGNAL = Add_of_vector("VECTOR2SIGNAL")

VECTOR2SIGNAL.sin1.value = (0.,0.,0.)

targetIN=VECTOR2SIGNAL.sin2
targetIN.value = (0.5,-0.5,1.3)
target_sig = VECTOR2SIGNAL.sout

#follow3DPoint(robot,solver,target_sig,5)
