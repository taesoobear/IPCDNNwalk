import os, sys, time, pdb
import mujoco
import numpy as np

import libcalab_ogre3d 
from  libcalab_ogre3d import *
import genesis as gs # used 0.3.4 version
import work.GenesisAddon as GenesisAddon

def pd_servo(rotation_delta, curr_ang_velocity):
    kp = 400.0
    kd = 30.0
    torque = rotation_delta *kp - curr_ang_velocity*kd
    return torque


# motion dof in a txt file.
def get_mocap_data():
    mocap_path_walk = "gym_trackSRB/motiondata_mujoco_refined/walk1_subject5.txt"

    with open(mocap_path_walk, 'r') as f:
        mocap_data = f.read()
        mocap_data = np.array(eval(mocap_data))

        return mocap_data


# main
timestep=0.0083332000

RE.createMainWin(sys.argv)
xmlpath='gym_trackSRB/Characters/humanoid_deepmimic_withpalm_Tpose.xml'
RE.setViewZup() # mujoco xml files usually use ZUP.

# you can directly using a mujoco xml scene in control.MujocoSim (a wrapper class for the mujoco sim) - see controlmodule.py. 
# In this case, simLoaders becomes the output (sim.loaders). 
# or you can use a list of loaders (each loader can be created from an xml file)
sim=GenesisAddon.GenesisSim(xmlpath, None, timestep, m.vector3(0,0, 9.8),libcalab_ogre3d) 
simLoaders=sim.loaders 

skins=RE.createSkin(simLoaders)
skins.setScale(100)

skins[0].setMaterial('shiny') # character 0

mocap_data = lua.mat(get_mocap_data()) # motiondof

# center motion data
mocap_data.array[:,0]-=mocap_data[0][0]
mocap_data.array[:,1]-=mocap_data[0][1]

mocap_dmot=mocap_data.derivative(30.0) # unlike mujoco qvel, mocap_dmot.cols()==mocap_data.cols()  because mocap_dmot.cols(3) is not used (MotionDOF에서 root quaternion.w를 저장하는 column)


# center mocap.


start_id = 100
character_id=0
sim.setPoseDOF(character_id,mocap_data[start_id]) # qpos
sim.setLinkVel(character_id, mocap_dmot[start_id])


for i in range(start_id+1, mocap_data.rows()):
    target_rotation = mocap_data[i].slice(7,0) # excluding root
    for _ in range(4):
        curr_rotation = sim.getPoseDOF(0).slice(7,0) # excluding root
        curr_vel = sim.getLinkVel(0).slice(7,0)
        torque = pd_servo(target_rotation-curr_rotation, curr_vel) 
        sim.setCtrl( character_id, torque)
        sim.stepSimulation()

    skins.setSamePose(sim) # similar to viewer.sync()
    if not RE.renderOneFrame(True): break
    time.sleep(0.05)
