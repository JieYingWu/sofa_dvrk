# Script to register robot to simulation scene based on pointing to the top of each marker

import sys
import numpy as np
import cisstNumericalPython
import geometry_util as geo
import transformations as T
from scipy.spatial.transform import Rotation as R

scale = 1000
##### For 2019-08-08-Lego ####
#phantom_points = np.array([[31.8,96.05,0], [15.9,96.05,-31.8], [-31.8,96.05,-15.9], [-31.8,96.05,31.8]])

# Currently manually read out from file
#measured_points = np.array([[-0.00282601136754,-0.0062888038662,-0.0932844615085],[0.0236521042868,0.0105876960433,-0.0935333671393],[0.00579400872003,0.0526017481315,-0.093906899099],[-0.0362004390969,0.0487251420195,-0.0957448310665]])*scale

#### For 2019-08-14-GelPhantom1 ####
# Height is adjusted since we poke into the phantom a little
height_scale = 0.5
phantom_points = np.array([[34.35,17.9+height_scale,-19.65], [-34.35,17.9+height_scale,-19.65], [-34.35,17.9+height_scale,19.65], [34.35,17.9+height_scale,19.65]])
measured_points = np.array([[-0.0796385679491,0.0143042152692,-0.166926943272],[-0.027116819443,0.0448946184372,-0.16551352955],[-0.0426067612174,0.0737392043797,-0.164060336369],[-0.0952109779515,0.0467586154014,-0.161902662597]])*scale

# Registers measured_points to phantom_points
transform = cisstNumericalPython.nmrRegistrationRigid(measured_points, phantom_points)
transform = transform[0]
q = R.from_dcm(transform.Rotation()).inv().as_quat()
q_inv = R.from_dcm(transform.Rotation()).as_quat()
transform = np.concatenate((transform.Rotation(), np.expand_dims(transform.Translation(), 1)), 1)
transform = np.concatenate((transform, np.array([[0,0,0,1]])), 0)

data = np.genfromtxt(str(sys.argv[1]), delimiter=',')
length = data.shape[0]-1 # First line is field names
new_data = np.zeros((length, 8))

# Copy over the time in seconds
new_data[:,0] = (data[1:,0] - data[1,0])/1e9

# Transform the position
data_translation = np.array(data[1:,4:7]) * scale
data_translation = np.concatenate((data_translation, np.ones((length,1))), 1)
data_translation = np.transpose(data_translation)
new_points =  np.matmul(transform, data_translation)
new_points = np.transpose(new_points)
new_data[:,1:4] = new_points[:,0:3]

# Transform the orientation
temp = np.matmul(np.expand_dims(q,0), np.transpose(data[1:,7:11]))
new_data[:,4:8] = np.matmul(np.transpose(temp), np.expand_dims(q_inv,0))

name = sys.argv[1][0:-4]
np.savetxt(name + '_processed' + '.csv', new_data, delimiter=',')
