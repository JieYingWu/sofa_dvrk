import csv
import numpy as np

class SimulatorDataset(Dataset):
    def __init__(self, input_filename, label_filename):
        self.input_filename = input_filename
        self.input_array = np.genfromtxt('input_filename', delimiter=',')
        self.label_filename = label_filename
        self.label_array = np.genfromtxt('label_filename')

    def __len__(self):
        self.input_array.shape[0]

    def __getitem__(self, idx):
        return self.input_array[idx], self.label_array[idx]
