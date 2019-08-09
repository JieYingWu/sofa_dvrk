# Script to read in a set of Polaris measurements and convert everything relative to the first frame

import sys
import numpy as np
import geometry_util as geo
import transformations as T

scale = 1
data = np.genfromtxt(str(sys.argv[1]), delimiter=',')
new_data = np.zeros((data.shape[0], 8))
new_data[:,0] = (data[:,0] - data[1,0])/1e9
base = data[1,4:11]
euler = T.euler_from_quaternion(np.append(base[6],base[3:6]))
base_transform = T.compose_matrix(translate=base[1:4]*scale, angles=euler)
inverse_base = geo.invertTransformation(base_transform)
#inverse_base = np.matmul(np.array([[1,0,0,0],[0,1,0,-0.1],[0,0,1,0],[0,0,0,1]],inverse_base))

for i in range(1,data.shape[0]):
    cur = data[i, 4:11]
    euler = T.euler_from_quaternion(np.append(cur[6],cur[3:6]))
    cur_transform = T.compose_matrix(translate=cur[1:4]*scale, angles=euler)
#    corrected_transform = cur_transform
    corrected_transform = np.matmul(cur_transform, inverse_base)
    corrected_transform = np.matmul(corrected_transform,np.array([[0,1,0,0],[0,0,-1,100],[-1,0,0,0],[0,0,0,1]]))
    scale, shear, angles, translate, perspective = T.decompose_matrix(corrected_transform)

    # Check that we found a valid rigid transformation
    if not ((scale < 1.2).all() and (scale > 0.8).all()):
        print(corrected_transform)
        print(scale)
        print('Scale is wrong')
        exit()
    shear = np.array(shear)
    if not ((shear < 0.1).all() and (shear > -0.1).all()):
        print(corrected_transform)
        print(shear)
        print('Shear is wrong')
        exit()
    if not ((perspective > [-0.1,-0.1,-0.1,0.9]).all() and (perspective < [0.1,0.1,0.1,1.1]).all()):
        print(corrected_transform)
        print(perspective)
        print('Perspective is wrong')
        exit()
        
    q = T.quaternion_from_euler(angles[0], angles[1], angles[2])
    # Convert from SI to milimetre for SOFA simulation
    new_data[i][1:8] = np.append(np.append(translate, q[1:4]), q[0])

new_data = new_data[1:]
name = sys.argv[1][0:-4]
np.savetxt(name + '_processed' + '.csv', new_data, delimiter=',')
