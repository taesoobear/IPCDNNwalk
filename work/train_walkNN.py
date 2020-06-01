# -*- coding: utf-8 -*
# sample_python aims to allow seamless integration with lua.
# see examples below
import torch
from torch.autograd import Variable
import torch.nn.functional as F
import torch.utils.data as Data

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import pdb
#%matplotlib inline
import numpy as np

#torch.manual_seed(1)    # reproducible
import os
import sys
import pdb # use pdb.set_trace() for debugging
import code # or use code.interact(...) for debugging. see below.
sys.path.append(os.getcwd())
sys.path.append(os.path.join(os.path.abspath('..'),'work'))
print(os.path.join(os.path.abspath('..'),'work'))
print(os.getcwd())

# the following three modules are important!!!
import libmainlib as m   
import luamodule as lua  # see luamodule.py
import numpy as np 


def main():

    lua.init_console()
    lua.dostring("g_dataset=util.loadTable('feature_data.dat')")


    if True:
        l=m.getPythonWin()
        l.getglobal('g_dataset')
        l.replaceTop(1)
        mat=l.popmatrixn()

    matfeature=lua.getglobal_mat('g_dataset', 1)
    matdata=lua.getglobal_mat('g_dataset', 2)
    matdotdata=lua.getglobal_mat('g_dataset', 3)


    train(matfeature, matdata, 'walkrun.net')
    #train(matfeature, matdotdata, 'dotwalkrun.net')

def train(matfeature,matdata, filename):
    x=np.array(matfeature)
    y=np.array(matdata)
    x=torch.tensor(x.astype(np.float32))
    y=torch.tensor(y.astype(np.float32))
    b_eval=True
    if b_eval:
        lua.dostring("g_dataset_eval=util.loadTable('feature_data_eval.dat')")
        l=m.getPythonWin()
        l.getglobal('g_dataset_eval')
        l.replaceTop(1)
        mat2=l.popmatrixn()
        matfeature_eval=lua.getglobal_mat('g_dataset_eval', 1)
        matdata_eval=lua.getglobal_mat('g_dataset_eval', 2)
        matdotdata_eval=lua.getglobal_mat('g_dataset_eval', 3)

    xe=np.array(matfeature_eval)
    ye=np.array(matdata_eval)
    xe=torch.tensor(xe.astype(np.float32))
    ye=torch.tensor(ye.astype(np.float32))

    # torch can only train on Variable, so convert them to Variable
    x, y = Variable(x), Variable(y)

    nin=matfeature.shape[1]
    nout=y.shape[1]

    print(nin, nout) # 105, 1500

    nhidden1=50
    #nhidden2=50 # works ok
    #nhidden2=24 # works ok
    nhidden2=12 # works ok    
    #nhidden2=15 # works ok
    net=torch.nn.Sequential(
            torch.nn.Dropout(p=0.01),
            torch.nn.Linear(nin, nhidden1),
            torch.nn.ELU(),
            torch.nn.Linear(nhidden1, nhidden2),
            torch.nn.ELU(),
            torch.nn.Linear(nhidden2, 200),
            torch.nn.ELU(),
            #torch.nn.Dropout(p=0.1),
            torch.nn.Linear(200, nout),
            )


    optimizer = torch.optim.Adam(net.parameters(), lr=0.001)
    loss_func = torch.nn.MSELoss()  # this is for regression mean squared loss

    BATCH_SIZE =32
    EPOCH = 2000  # MSE:  0.0007 ->  3705 (15)
 #                               -> 4401 (12)
    #EPOCH = 200 # 0.0012 -> MSE: 6710

    torch_dataset = Data.TensorDataset(x, y)

    loader = Data.DataLoader(
            dataset=torch_dataset, 
            batch_size=BATCH_SIZE, 
            shuffle=True, num_workers=2,)

    b_plot=True

    # start training
    for epoch in range(EPOCH):
        print('epoch', epoch)
        for step, (batch_x, batch_y) in enumerate(loader): # for each training step

            #print(step)
            b_x = Variable(batch_x)
            b_y = Variable(batch_y)

            prediction = net(b_x)     # input x and predict based on x

            loss = loss_func(prediction, b_y)     # must be (1. nn output, 2. target)
            #regularity =  torch.norm(net[0].weight, p=1) +  torch.norm(net[2].weight, p=1)
            #cost = loss + 0.01*regularity
            if step == 1:
                print('epoch', epoch, 'loss', loss.data.numpy())

            optimizer.zero_grad()   # clear gradients for next train
            loss.backward()         # backpropagation, compute gradients
            #cost.backward()         # backpropagation, compute gradients
            optimizer.step()        # apply gradients
        if b_eval:
            prediction = net(x)     # input x and predict based on x
            predictione = net(xe)     # input x and predict based on x
            print(':', epoch, ((y-prediction)**2).mean().detach().numpy(), ((ye-predictione)**2).mean().detach().numpy())

        if b_plot:
            prediction = net(x)     # input x and predict based on x
            print(epoch, ((y-prediction)**2).mean())

    if b_plot:
        torch.save(net, filename+'.plot.dat')
    else:
        torch.save(net, filename)

    prediction = net(x)     # input x and predict based on x
    print('final MSE=',torch.sum((y-prediction)**2))
    print('final MSE col0=',torch.sum((y[:,0]-prediction[:,0])**2))
     
    #epoch 200, n_hidden2=12:
    #MSE ->  4401

if __name__=="__main__":
    main()
