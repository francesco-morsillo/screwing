from dynamic_graph import plug
from dynamic_graph.sot.core.math_small_entities import Multiply_of_matrixHomo, Selec_of_vector, Selec_column_of_matrix

A = ((0,1,2,3),(4,5,6,7),(8,9,10,11),(12,13,14,15))




m2pos1 = Selec_column_of_matrix("M2POS1")
m2pos1.selecCols(3)
m2pos1.selecRows(0,3)
m2pos1.sin.value = A


"""
m2pos2 = Selec_of_vector("M2POS2")
plug(m2pos1.sout,m2pos2.sin)
"""
target_sig = m2pos1.sout
