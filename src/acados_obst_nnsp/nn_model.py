import torch

from torch import nn
from torch.optim import Adam
from torch.nn import Conv2d, Linear, MaxPool2d, ReLU, LogSoftmax, Dropout, BatchNorm1d, Tanh
from torch.nn import Module
from torch import flatten

import ml_casadi.torch as ml_cas

class KinematicModel(Module):
    def __init__(self):
        super().__init__()
        self.fc1 = Linear(in_features=5, out_features = 32)
        #self.BN = BatchNorm1d(32)
        self.relu3 = ReLU()
        self.fc2 = Linear(in_features=32, out_features = 128)
        self.relu4 = ReLU()
        self.dropout = Dropout(p=0.2)
        self.fc3 = Linear(in_features=128, out_features=5)
        #self.tanh = Tanh()
    def forward(self,data):
        x = self.fc1(data)
        #x = self.BN(x)
        x = self.relu3(x)
        x = self.fc2(x)
        x = self.relu4(x)
        x = self.dropout(x)
        x = self.fc3(x) 
        #self.tanh(x)
        return x
        
def load_nn_model(model_path, USE_GPU=True):
    model = KinematicModel()
    model.load_state_dict(torch.load(model_path))
    learned_dyn = ml_cas.TorchMLCasadiModuleWrapper(model, input_size=5, output_size=5)
    if USE_GPU:
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        print('Using device:', device)
        learned_dyn = learned_dyn.to(device)
    return learned_dyn
        
