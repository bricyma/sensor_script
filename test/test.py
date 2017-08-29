# test numpy return
# return numpy.bool
import numpy as np

def a():
    return np.mean([0,0,0,0,0]) < 0.1


print type(a())
