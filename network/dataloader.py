import csv
import numpy as np

class SimulatorDataset(Dataset):
    def __init__(self, kinematics_file, simulator_file, label_file):
        self.kinematics_file = kinematics_file
        self.kinematics_array = np.genfromtxt(kinematics_file, delimiter=',')
        self.simulator_file = simulator_file
        self.simulator_array = np.genfromtxt(simulator_file, delimiter=',')
        self.label_file = label_file
        self.label_array = np.genfromtxt(label_file, delimiter=',')

        # Should do some time sync here
        
    def __len__(self):
        self.input_array.shape[0]

    def __getitem__(self, idx):
        return self.kinematics_array[idx] + self.simulator_array[idx], self.label_array[idx]
