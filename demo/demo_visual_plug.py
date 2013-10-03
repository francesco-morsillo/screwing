from dynamic_graph import plug

from dynamic_graph.sot.screwing.vel_control_functions import follow3DPoint

from dynamic_graph.sot.screwing.vision_data import plugObject
from dynamic_graph.sot.core.math_small_entities import Multiply_of_matrixHomo, Selec_column_of_matrix
from dynamic_graph.sot.core.matrix_util import vectorToTuple, matrixToTuple



m2pos = Selec_column_of_matrix("M2POS")
m2pos.selecCols(3)
m2pos.selecRows(0,3)

target_sig = m2pos.sout

"""
targetIN = plugObject(robot)
plug(targetIN,m2pos.sin)
follow3DPoint(robot,solver,target_sig,2)
"""
