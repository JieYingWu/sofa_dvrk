import torch
from torch import nn

# Testing fully connected networks for predicting positions
class SpringNetwork(nn.Module):

    def __init__(self):
        super(SpringNetwork, self).__init__()
        # Input info: 4 points of the tabletop * 3 coordinate = 12 + 7 pos, 7 vel from dVRK = 26
        model = nn.Sequential(
            nn.Linear(26, 128),
            nn.ReLU(),
            nn.Linear(128, 256),
            nn.ReLU()
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, 12)
            )

    def forward(self, x):
        return self.model(x)

    
    
