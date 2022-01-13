import sys
import numpy as np

for n in range(12):

    all_time_steps = [0.0332, 0.0332, 0.0329, 0.0332, 0.0332, 0.0333, 0.0331, 0.0332, 0.0332, 0.0328, 0.0455, 0.0473]
    lengths = [114450, 72679, 59968, 58071, 48895, 59941, 47274]
    time_scale = all_time_steps[n]*10000
    folder_name = 'data' + str(n)

    robot_pos = np.genfromtxt('../dataset/2019-10-09-GelPhantom1/dvrk/' + folder_name  + '_robot_cartesian_velocity.csv', delimiter=',')
    time = range(robot_pos.shape[0])
    interpolated_time = np.array(range(0,robot_pos.shape[0]*int(time_scale)))/time_scale
    interpolated_robot_pos = None
    for i in range(robot_pos.shape[1]):
        temp = np.expand_dims(np.interp(interpolated_time, time, robot_pos[:,i]), axis=1)
        if interpolated_robot_pos is None:
            interpolated_robot_pos = temp
        else:
            interpolated_robot_pos = np.concatenate((interpolated_robot_pos, temp), axis=1)

    interpolated_robot_pos = interpolated_robot_pos[0::11, :]
    interpolated_robot_pos = interpolated_robot_pos[0:lengths[n], :]
    print(interpolated_robot_pos.shape)
    np.savetxt('../dataset/2019-10-09-GelPhantom1/dvrk/' + folder_name + '_robot_cartesian_velocity_fine.csv', interpolated_robot_pos, delimiter=',')
