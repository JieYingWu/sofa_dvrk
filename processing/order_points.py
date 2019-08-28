# Create a canonical grid order from the initial phantom mesh before deformation

import sys
import numpy as np

arr = np.loadtxt(sys.argv[1])
ind = np.lexsort((arr[:,2], arr[:,1], arr[:,0]))
np.savetxt('grid_order.txt',ind)
