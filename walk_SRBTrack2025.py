import time
from libcalab_ogre3d import lua, RE, control
import work.luascript
#from env.FullbodyVAE import FullbodyVAEEncoder, FullbodyVAEDecoder
from gym_trackSRB import taesooLib
from gym_trackSRB.taesooLib.FullbodyVAE import FullbodyVAEEncoder, FullbodyVAEDecoder
import torch, random
#from stable_baselines3 import PPO
from env.myppo import MyPPO as PPO
#from env.SRBEnv5_mocap_list import SRBEnv
from env.SRBEnv5_mocap_list_window import SRBEnv
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation as R

import sys,os, pdb, math
from pathlib import Path
from easydict import EasyDict as edict
m=lua.taesooLib()
import gym_trackSRB.taesooLib.DeltaExtractor as ED 



"""
    To test RL results, you need to keep the motion type in mocap_path and model_path the same in your environment
"""
reattachCamera=True

# 加载环境
mocap_filename='walk1_subject5'
mocap_path = "motiondata_mujoco_refined/"+mocap_filename+".txt"
mocap_path2='lafan1/'+mocap_filename+'.bvh'
mocap_frame_rate=30

mocap_path_0 = "motiondata_mujoco_refined/walk1_subject5.txt"
mocap_path_1 = "motiondata_mujoco_refined/run1_subject5.txt"
mocap_path_2 = "motiondata_mujoco_refined/jumps1_subject1.txt"
mocap_path_3 = "motiondata_mujoco_refined/jumps1_subject5.txt"
mocap_path_4 = "motiondata_mujoco_refined/sprint1_subject2.txt"
mocap_path_5 = "motiondata_mujoco_refined_mirrored/walk1_subject5_mirrored.txt"
mocap_path_6 = "motiondata_mujoco_refined_mirrored/run1_subject5_mirrored.txt"
mocap_path_7 = "motiondata_mujoco_refined_mirrored/jumps1_subject1_mirrored.txt"
mocap_path_8 = "motiondata_mujoco_refined_mirrored/jumps1_subject5_mirrored.txt"
mocap_path_9 = "motiondata_mujoco_refined_mirrored/sprint1_subject2_mirrored.txt"
mocap_path_10 = "motiondata_mujoco_refined/stand1.txt"
mocap_path_list = [ mocap_path_0, mocap_path_1, mocap_path_2, mocap_path_3, mocap_path_4, mocap_path_10]
mocap_id_to_show = 1  # the id in mocap list, decide which motion to show

debugMode=False

_lastPose=None

def projectToGround(v):
    v.y=0


replanningTriggered=False
def onCallback(w, id):
    global _lastPose

    if w.id()=="center":
        if _lastPose!=None:
            lua.M(RE.viewpoint(),"lookAt", currPredictedPose.toVector3(0), { 'moveGridFloor':True, 'reattach':True, 'offset':m.vector3(-50,100,0)})

    elif w.id().startswith('paused'):
        global paused
        paused = w.checkButtonValue()
    elif w.id()=='use momentum mapping':
        if w.checkButtonValue():
            trajOpt.set('weightMomentum', weightMomentum)
        else:
            trajOpt.set('weightMomentum', 0)

    elif w.id()=='use COM':
        if w.checkButtonValue():
            trajOpt.set('weightCOM', weightCOM)
        else:
            trajOpt.set('weightCOM', 0)
    elif w.id()=='offset SRB':
        global draw_offset_SRB
        if w.checkButtonValue():
            draw_offset_SRB=m.vector3(-1,0,0)
        else:
            draw_offset_SRB=m.vector3(0,0,0)
        mSkin.setTranslation(draw_offset_SRB*100)
    elif w.id().startswith('push') and not w.id()=='push direction' and not w.id()=='push magnitude':
        simulationFrameRate=1/env.model.opt.timestep
        env.impulse=0.2*simulationFrameRate # duration

        pushMag=this.findWidget('push magnitude').sliderValue()
        env.force=(m.quater(this.findWidget('push direction').sliderValue(),m.vector3(0,1,0))*m.vector3(-1,0,0))*pushMag
        env.impulse_lpos=m.vector3(0,0,0)
        env.gizmo=['Hips', m.vector3(0, 0.2, 0), False]

        # todo : add fullbody marker constraints at push locations
        complianceFactor=0.5
        if w.id().startswith('push head'):
            env.impulse_lpos=m.vector3(0,0.65,0.03)*complianceFactor
            env.gizmo=['Neck', m.vector3(0, 0.09, 0), False]
        elif w.id().startswith('push arm'):
            env.impulse_lpos=m.vector3(-0.18,0.18,0.13)*complianceFactor
            env.gizmo=['RightElbow', m.vector3(-0.27, 0, 0), False]
        elif w.id().startswith('push leg'):
            env.impulse_lpos=m.vector3(-0.18,-0.22,0.13)*complianceFactor
            env.gizmo=['RightHip', m.vector3(-0.08, -0.25, 0), False]

        f=np.array([0,0,0,0,0,0])
        RE.setVec3(f[0:3], env.force.YtoZ())
        RE.setVec3(f[3:6], env.impulse_lpos.YtoZ().cross(env.force.YtoZ()))
        env.force=f
        RE.output("impulse", str(env.impulse))

    elif w.id()=='attach camera':
        global reattachCamera
        if w.checkButtonValue():
            reattachCamera=True

def frameMove(fElapsedTime):
    RE.updateBillboards(fElapsedTime)
    return 0

# main
RE.createMainWin(sys.argv)

from gym_trackSRB.taesooLib.mmsto_config import *

env = SRBEnv(mocap_path_list)
env.impulse_interval=1e9 # turn off.
# 加载训练好的模型
ppomodel = PPO.load(model_path, exclude=["optimizer"])

paused = False


useThreads=True
threads=[ m.ThreadedScript() , m.ThreadedScript()]
for i, v in enumerate(threads):
    v.dostring('require("common") require("module")')

def set_feet_site_mocap(viewer, feet_pos_list_this_step, feet_ori_list_this_step):  # set feet state in simulation
    feet_site_list_SRB = ['left_foot_center', 'left_foot_lf', 'left_foot_rf', 'left_foot_lb', 'left_foot_rb',
                          'right_foot_center', 'right_foot_lf', 'right_foot_rf', 'right_foot_lb', 'right_foot_rb']
    i = 0
    j = 0
    offset = 5
    # the relative pos betweenn site_center and site_front,site_back
    rel_lf = np.array([0.1, 0.03, 0])
    rel_rf = np.array([0.1, -0.03, 0])
    rel_lb = np.array([-0.1, 0.03, 0])
    rel_rb = np.array([-0.1, -0.03, 0])
    while j < len(feet_site_list_SRB):

        material='blue'
        if j==env.right_site_id:
            material='red'
        for k in range(offset):
            RE.delayedDraw('SphereM',drawDelay, RE.toVector3(env.data.site_xpos[j+k]).ZtoY()+draw_offset_SRB, str(k)+"_site"+str(j),material,0.02)
            RE.delayedErase('SphereM', drawDelay, str(j)+"_"+str(k))

        if feet_pos_list_this_step[i][2] != 0:  # If the z-coordinate is not 0, skip this site.
            for k in range(offset):
                factor = [0, 1, 1, 1, 1]
                rel = [np.array([0, 0, 0]), rel_lf, rel_rf, rel_lb, rel_rb]
                pos=feet_pos_list_this_step[i] + np.array([0, 1, 0])+feet_ori_list_this_step[i]@(factor[k]*rel[k])  # mocap com position
                RE.delayedDraw('SphereM',drawDelay, RE.toVector3(pos).ZtoY()+draw_offset_SRB, str(j)+"_"+str(k),'green',0.05)

            j += offset
            i += 1
            continue

        for k in range(offset):
            # mocap
            factor = [0, 1, 1, 1, 1]
            rel = [np.array([0, 0, 0]), rel_lf, rel_rf, rel_lb, rel_rb]
            pos=feet_pos_list_this_step[i] + np.array([0, 1, 0]) + feet_ori_list_this_step[i]@(factor[k] * rel[k])  # mocap com position
            RE.delayedDraw('SphereM',drawDelay, RE.toVector3(pos).ZtoY()+draw_offset_SRB,str(j)+"__"+str(k),'white',0.05)

        j += offset
        i += 1


def show_SRB_mocap(viewer, env):
    size=np.array([0.19557607215607947, 0.25406692031825, 0.667757440991862])
    pos=env.mocap_pos_com[env.current_frame+1] + np.array([0, 1, 0])  # mocap com position
    mat=env.mocap_ori_com[env.current_frame+1].flatten()  # 方向
    rgba=np.array([1, 0, 0, 0.5])  # 红色
    label = 'SRB mocap'


    RE.delayedDraw('Box', drawDelay, RE.toTransf(mat,pos).ZtoY().translate(draw_offset_SRB), label,  RE.toVector3(size).ZtoY()*2, 100, 'red_transparent')

    #viewer.user_scn.ngeom += 1
    curr_feet_pos = env.feet_pos_list[env.current_frame+1].copy()
    curr_feet_pos[:, 2] = 0
    curr_feet_ori = env.feet_ori_list[env.current_frame+1].copy()
    set_feet_site_mocap(viewer, curr_feet_pos, curr_feet_ori)


def show_text(viewer, nameid, text_pos, text):
    size=np.array([0.03, 0.03, 0.03])  # mocap关节的视觉尺寸
    pos=text_pos  # mocap关节的位置
    rgba=np.array([0, 0, 1, 1])  # 蓝色
    RE.output(nameid, text)


mLoader_full=RE.WRLloader('gym_trackSRB/taesooLib/hanyang_lowdof_T_sh.wrl')


full_pred, device=loadPosePredictor()
this=m.getPythonWin()
this.create("Check_Button", "attach camera", "attach camera", 0,2)
this.widget(0).checkButtonValue(False)
this.create("Button", "center", "center", 2,3)
this.resetToDefault()
this.widget(0).buttonShortcut('FL_CTRL+p')
this.addCheckButton('paused (ctrl+s)', False)
this.widget(0).buttonShortcut('FL_CTRL+s') 
this.addCheckButton('draw mocap SRB', False)
this.addCheckButton('draw predicted feet', False)
this.addCheckButton('draw optimized feet', False)
this.addCheckButton('draw COM', False)
this.addCheckButton('offset SRB', True)
this.addCheckButton('draw constrained feet', False)
this.addCheckButton('solve COM IK', True)
this.addCheckButton('solve limb IK', True)
this.addCheckButton('use momentum mapping', True)
this.addCheckButton('use COM', True)

this.addFloatSlider('push direction', 0, 0, 180)
this.addFloatSlider('push magnitude', 200, 200,1200)
this.create('Button', 'push', 'push (ctrl+o)')
this.widget(0).buttonShortcut('FL_CTRL+o') 
this.create('Button', 'push head', 'push head')
this.create('Button', 'push arm', 'push arm')
this.create('Button', 'push leg', 'push leg')
draw_offset_SRB=m.vector3(-1,0,0)
this.updateLayout()

if True:
    start = time.time()
    env.testing_mode = True

    model_path = 'gym_trackSRB/Characters/SRB4.xml'
    mLoader=RE.MujocoLoader(model_path,{'convertToYUP':True})

    srb_tree=m.LoaderToTree(mLoader, False, False)

    mSkin=RE.createSkin(mLoader)
    mSkin.setScale(100)
    mSkin.setMaterial('lightgrey_transparent')
    mSkin.setTranslation(draw_offset_SRB*100)
    if True:
        # prepare robot rendering

        lua.addPackagePath(Path(os.getcwd()).joinpath('mujoco'))
        lua.require_as('gym_trackSRB/taesooLib/LafanRetarget', 'lafan')
        lua.require('gym_cdm2/module/hanyangToRobot')


        mHanyangSkin=RE.createSkin(mLoader_full)
        mHanyangSkin.setScale(100)
        mHanyangSkin.setTranslation(100,0,0)
        mHanyangSkin.setMaterial('lightgrey_verytransparent')
        mHanyangSkin.setVisible(False)

        #_robotLoader=RE.FBXloader('../../FBX/fbxModels/robot/Robot_TPOSE.fbx', {'useTexture':True, 'newRootBone':'pelvis', 'scale':1.02/100.0, 'boneScale':{'neck_01':0.8, 'ball_l':0.01, 'ball_r':0.01} })
        #_robotLoader.fbxInfo[1].material='Body_robot_low_BaseColor.tga' # use hand-designed material instead of auto-gen one
        _robotLoader=RE.FBXloader('gym_trackSRB/Characters/robot.fbx.dat')
        curr_robot_pose=m.Pose()
        lua.F("RE.loadPose", curr_robot_pose, 'gym_trackSRB/Characters/robot.fbx.pose')
        _robotLoader.loader.setPose(curr_robot_pose)

        lua.F_lua('mHanyangToRobot', 'RE.createPoseTransferFromHanyangLowDOFtoRobot', mLoader_full, _robotLoader, True)

        # robot skin과 collider 수작업으로 최대한 맞추기. 
        mLoader_full.VRMLbone(mLoader_full.getTreeIndexByName('LeftAnkle')).translateMesh(m.vector3(0.0, 0.02, 0.0))
        mLoader_full.VRMLbone(mLoader_full.getTreeIndexByName('RightAnkle')).translateMesh(m.vector3(0.0, 0.02, 0.0))

        mRobotSkin=RE.createSkin(_robotLoader, {'adjustable':True})
        mRobotSkin.setPose(_robotLoader.loader.pose())
        mRobotSkin.setScale(100)
        mRobotSkin.setTranslation(0,0,0)
        lua.M(mRobotSkin, 'setPoseTransfer', lua.instance('mHanyangToRobot.pt'))


    if True: # prepare pose optimizer
        lua.require_as('gym_cdm2/shortTermOptimizer', 'ShortTrajOpt') 
        outputMotionBuffer=m.MotionDOF(mLoader_full.dofInfo)
        outputMotionBuffer.resize(planningHorizon+1)
        for i in range(planningHorizon+1):
            outputMotionBuffer.row(i).assign(mLoader_full.getPoseDOF())

        trajOpt=lua.new('ShortTrajOpt', mLoader_full, outputMotionBuffer)

        if useThreads:
            for i in range(2):
                threads[i].dostring("""
                ShortTrajOpt=require('gym_cdm2/shortTermOptimizer')
                function createTrajOpt(loader, outputMotionBuffer)
                    trajOpt=ShortTrajOpt(loader, outputMotionBuffer)
                end
                function setParam(weightMomentum, weightCOM,  desiredeuler, dx, ddx, com_target, momentum_target)
                    trajOpt.weightAngleOriginal= 100000
                    trajOpt.weightDX= 100
                    trajOpt.weightDDX= 10
                    trajOpt.weightEE= 100000
                    trajOpt.weightEEcon= 0
                    trajOpt.weightEEvel= 0
                    trajOpt.weightVelHalfSpace= 100000
                    trajOpt.weightMomentum= weightMomentum
                    trajOpt.weightCOM= weightCOM
                    trajOpt.x_original:assign(desiredeuler:toVector())
                    trajOpt.dx:assign(dx:toVector())
                    trajOpt.ddx:assign(ddx:toVector())
                    trajOpt.com= com_target:copy()
                    trajOpt.momentum= momentum_target:copy()
                end
                function setCon(conp) 
                    trajOpt.con={
                        conp:column(0):copy(),
                        conp:column(1):copy(),
                        conp:column(2):copy(),
                        conp:column(3):copy(),
                    }
                end
                function solveEE_part1(buffer,gpos_target, hsVelConInfo)
                    g_args={buffer:copy(), gpos_target:copy(), deepCopyTable(hsVelConInfo)}
                end
                function solveEE_part2()
                    return trajOpt:solveEndEffectors(g_args[1], markers,g_args[2], nil, nil, g_args[3])
                end
                """)
                threads[i].blockingCall('createTrajOpt', mLoader_full, outputMotionBuffer)


        # set toe markers later.
        mConConfig=lua.F('dofile', 'work/taesooLib/Resource/motion/MOB1/hanyang_lowdof_T.con_config.lua')

        markers=[]
        for i in range(4):
            markers.append(edict(weight=100))
        markers[0].lpos=  mConConfig.toes_lpos[0] # l toe
        markers[1].lpos=  mConConfig.ankles_lpos[0] # l heel
        markers[2].lpos=  mConConfig.toes_lpos[1] # r toe
        markers[3].lpos=  mConConfig.ankles_lpos[1]
        markers[0].bone=mLoader_full.getBoneByName(mConConfig.toes[0])
        markers[1].bone=mLoader_full.getBoneByName(mConConfig.ankles[0])
        markers[2].bone=mLoader_full.getBoneByName(mConConfig.toes[1])
        markers[3].bone=mLoader_full.getBoneByName(mConConfig.ankles[1])


        onlineIK=[None, None]
        for ifoot in range(2):
            onlineIK[ifoot]=RE.OnlineSingleLimbIK(mLoader_full, markers[ifoot*2].bone.parent().parent(), markers[ifoot*2].bone, 5)
        #onlineFilterPose=RE.OnlineFilter(onlineFilterSize, mLoader_full)
        
        mask=lua.vec(np.bitwise_or(onlineIK[0].mask.ref()>0.5, onlineIK[1].mask.ref()>0.5).astype(float))
        mask.slice(0,7).setAllValue(1.0)
        onlineFilterPose=RE.MaskedOnlineFilter(onlineFilterSize, mask, mLoader_full)
        onlineFilterLen=RE.OnlineFilter(onlineFilterSize)

        # copy table to lua. (매번 파라미터로 넘기는 것보다 빠를 것 같아서) 
        lua.setGlobal('markers', markers)
        if useThreads:
            for i in range(2):
                threads[i]('setGlobal','markers', markers)


        #lua.F('printTable', lua.instance('markers'))
        full_tree=trajOpt.get('loaderToTree')
        if True:
            # make ankle joints more flexible
            s_l=full_tree.getVarIndex(markers[1].bone.treeIndex(),0)
            s_r=full_tree.getVarIndex(markers[3].bone.treeIndex(),0)
            trajOpt.get('dof_weights').range(s_l, s_l+3).setAllValue(0.1)
            trajOpt.get('dof_weights').range(s_r, s_r+3).setAllValue(0.1)
            if useThreads:
                for i in range(2):
                    threads[i]('dostring',"""
                    function setDOFweights(s_l)
                        trajOpt.dof_weights:range(s_l, s_l+3):setAllValue(0.1)
                    end""")
                    threads[i].blockingCall('setDOFweights',s_l)
                    threads[i].blockingCall('setDOFweights',s_r)

        limbIKsolver=lua.new('createLimbIksolverToeAndHeel', mLoader_full, lua.instance('markers'))


    if True: # prepare limbik solver
        lua.require_as('retargetting/module/footSkateCleanup', 'fc')
        footLen=markers[0].lpos.distance(markers[1].lpos)


    # env.testing_mode = False
    obs, info = env.reset(mocap_id=mocap_id_to_show)  # 重置环境状态
    env.impulse_interval = 1e9  
    # visuaRE.viewpoint():setFOVy(45.000000)
    RE.viewpoint().setZoom(1.000000)
    RE.viewpoint().vpos.set(-192.964016, 84.407388, 803.501714)
    RE.viewpoint().vat.set(-215.039250, 84.407388, 408.111988)
    RE.viewpoint().update()

    viewer=None  

    if this.findWidget('draw mocap SRB').checkButtonValue():
        show_SRB_mocap(viewer, env)
    show_text(viewer, 'frame', env.data.qpos[:3] + np.array([0.0, 0.0, 1.0]), f'current step: {env.current_frame}')

    obs_buffer=m.matrixn()
    fullstate_buffer=[]

    # fullbody states (workspace + history buffers)
    fullbody_io_buffers=ED.create_fullbody_io_buffers(mLoader_full, mmsto_config, outputMotionBuffer)
    desired_fullbody_pose_buffer, desired_euler_dx_ddx, markerTarget, com_target, momentum_target, con_target, feet_traj,  _=fullbody_io_buffers
    velocity_constraints=[None]*planningHorizon

    prev_thread=None
    while True:
        if not paused:

            timer=m.Timer()
            timer.start()
            action, _ = ppomodel.predict(obs, deterministic=True)  # 使用模型预测动作
            RE.output('ppo predict (ms)', timer.stop2()/1e3)
            timer.start()

            #print(env.mocap_pos_com[env.current_frame + 1 + env.random_initial_frame, 2]-env.data.qpos[2])
            #for i in range(len(action)):
            #    if action[i] >= 2:
            #        print(action)
            #        break
            #start = time.time()
            obs, _, done, _, _ = env.step(action)
            RE.output('env step (ms)', timer.stop2()/1e3)

            fullstate_buffer.append(env.getFullState())
            if len(fullstate_buffer)>planningHorizon:
                fullstate_buffer.pop(0)

            obs_extra=ED.pack_obs_extra(env) # global information

            com_speed=obs_extra.toVector3(10).length()
            RE.output('com speed', com_speed)
            if True:
                srbpose=obs_extra.slice(0,7).toTransf()
                if srbpose.translation.y<0.45 or srbpose.rotation.offsetQ().rotationAngle()>math.radians(60):
                    # detect falldown. use leg length, orientation and so on.
                    if srbpose.translation.y<0.45:
                        print('passive', 'height too low')
                        RE.output('passive', 'height too low')
                    else:
                        print('too much leaning angle')
                        RE.output('passive', 'too much leaning angle')
                    this.findWidget('use passive sim (ctrl+p)').checkButtonValue(True)



            #obs_buffer.pushBack(lua.vec(obs))
            #obs_buffer.pushBack(lua.vec(ED.convertTo6D(obs)))
            obs_buffer.enqueue(lua.vec(obs[0:SRB_obs_dim])|obs_extra, obs_history_buffer_size+planningHorizon)


            if obs_buffer.rows()>=obs_history_buffer_size:
                ED.drawOBS(obs_buffer[-(planningHorizon-1)], mSkin, draw_offset_SRB) # this is rendered one frame earlier than fullbody pose, due to the onlinefilter
                #ED.drawOBS(obs_buffer[-(planningHorizon)], mSkin, draw_offset_SRB) # this looks correct although render_pose corresponds to -(planningHorizon-1)

                #allPredictions=(humanPose, dx_ddx, feetpos, com, momentum, con, con_float) for this frame.
                timer.start()
                allPredictions_stacked=ED.predictSingleFrameFullbody(device, srb_tree,mLoader, full_pred, obs_buffer.sub(-obs_history_buffer_size, 0,0,0), mmsto_config)
                RE.output('fullbody predict (ms)', timer.stop2()/1e3)
                allPredictions=allPredictions_stacked[-1]


                # save predictions to the buffers
                #ED.enqueueToFullbodyBuffers(*fullbody_io_buffers,*allPredictions, mmsto_config, obs_buffer)
                ED.updateFullbodyBuffers(fullbody_io_buffers,allPredictions_stacked, mmsto_config, obs_buffer)

                if False and obs_buffer.rows()>=obs_history_buffer_size+planningHorizon:
                    # test replanning -> should produce the same buffers using the same input
                    temp_buffers=ED.create_fullbody_io_buffers(mLoader_full, mmsto_config)

                    for i in range(-planningHorizon,1):
                        temp_predictions=ED.predictSingleFrameFullbody(device, srb_tree,mLoader, full_pred, obs_buffer.sub(-obs_history_buffer_size+i,0+i,0,0), mmsto_config)
                        ED.enqueueToFullbodyBuffers(*temp_buffers, *temp_predictions, mmsto_config, obs_buffer.sub(0, i,0,0))
                    # 오차 범위내 동일함. 

                if this.findWidget('draw predicted feet').checkButtonValue():
                    feetpos=allPredictions[2]
                    con=allPredictions[5]
                    for icontact in range(4):
                        # show feet pos prediction  
                        material='red'
                        if ifoot==0:
                            material='blue'
                        r=0.1
                        if con(icontact)<0:
                            material=material+'_transparent'
                            r=0.05

                        RE.delayedDraw('SphereM',drawDelay, feetpos.toVector3(icontact*3),'foot__'+str(icontact),material,r)
                        #RE.delayedDraw('Axes',drawDelay, refCoord,'footref'+str(icontact),100)




                if desired_fullbody_pose_buffer.rows()>planningHorizon:
                    # MMSTO


                    # now ready to perform MMSTO (this euler conversion can be done incrementally per-frame but for simplicity)
                    desired_euler_motion=trajOpt('calculateOriginalEulerPoses', desired_fullbody_pose_buffer, {'poseOnly':True})

                    nDOF=int(desired_euler_dx_ddx.cols()/2)
                    desired_dx=desired_euler_dx_ddx.sub(-(planningHorizon-1),0,0, nDOF).copy()
                    desired_ddx=desired_euler_dx_ddx.sub(-(planningHorizon-1),-1,nDOF, 0).copy()

                    assert(markerTarget.rows()==planningHorizon)
                    assert(desired_dx.rows()==planningHorizon-1)
                    assert(desired_ddx.rows()==planningHorizon-2)
                                                
                    weightMomentum=1000
                    weightCOM=100000
                    thread=threads[env.current_frame%2]

                    if useThreads:
                        thread.blockingCall( 'setParam', weightMomentum, weightCOM, desired_euler_motion.matView(nDOF).sub(-planningHorizon, 0, 0,0), desired_dx, desired_ddx, com_target, momentum_target)

                    else:
                        trajOpt.set('weightAngleOriginal', 100000)
                        trajOpt.set('weightDX', 100)
                        trajOpt.set('weightDDX', 10)
                        trajOpt.set('weightEE', 100000)
                        trajOpt.set('weightEEcon', 0)
                        trajOpt.set('weightEEvel', 0)
                        trajOpt.set('weightVelHalfSpace', 100000)
                        trajOpt.set('weightMomentum', weightMomentum)
                        trajOpt.set('weightCOM', weightCOM)
                        trajOpt.get('x_original').assign(desired_euler_motion.matView(nDOF).sub(-planningHorizon, 0, 0,0).toVector())
                        trajOpt.get('dx').assign(desired_dx.toVector())
                        trajOpt.get('ddx').assign(desired_ddx.toVector())
                        trajOpt.set('com', com_target)
                        trajOpt.set('momentum', momentum_target)

                    # feet and toe positions.
                    gpos_target=markerTarget
                    con_copy=con_target.copy()
                    for i in range(4):
                        lua.F('math.medianFilter', 4, con_copy.column(i))

                    con_planned=con_copy.ref()[-planningHorizon:,:]>0.5
                    # convert to taeosoLib types
                    if useThreads:
                        thread.blockingCall( 'setCon', con_planned)
                    else:
                        con_planned2=[ lua.ivec(con_planned[:,0]), lua.ivec(con_planned[:,1]), lua.ivec(con_planned[:,2]), lua.ivec(con_planned[:,3])]
                        trajOpt.set('con', con_planned2)

                    if len(velocity_constraints)>planningHorizon:
                        velocity_constraints.pop(0)
                    velcon=velocity_constraints[-(planningHorizon-1)]
                    hsVelConInfo={}
                    if velcon!=None and len(velcon)>0:
                        con_min_speed=m.vectorn()
                        con_normals=m.vector3N()
                        tree_indices=m.intvectorn()
                        local_positions=m.vector3N()
                        for i in range(len(velcon)):
                            v=velcon[i]
                            vel=v[2]
                            con_min_speed.pushBack(vel.length())
                            vel.normalize()
                            con_normals.pushBack(vel)
                            tree_indices.pushBack(v[0])
                            local_positions.pushBack(v[1])

                        # 3 corresponds to frame 2 (1-indexing in lua)
                        hsVelConInfo[3]=[ con_min_speed, con_normals, tree_indices, local_positions]
                    

                    timer.start()

                    # the first two frames of the output Motion buffer are used as constraints.
                    if useThreads:
                        thread.blockingCall('solveEE_part1',outputMotionBuffer.slice(1, 0), gpos_target, hsVelConInfo)
                        thread.nonblockingCall('solveEE_part2')
                    else:
                        out=trajOpt('solveEndEffectors', outputMotionBuffer.slice(1, 0), lua.instance('markers'), gpos_target, None, None, hsVelConInfo)
                    RE.output('solveEE (ms)', timer.stop2()/1e3)


                    if useThreads :
                        if prev_thread is not None :
                            # feedback results 
                            out=prev_thread.getResults()
                            outputMotionBuffer.range(0, planningHorizon).assign(out)

                        prev_thread=thread
                    else:
                        # feedback results 
                        outputMotionBuffer.range(1, planningHorizon+1).assign(out)

                    if this.findWidget('solve COM IK').checkButtonValue():
                        com_srb=m.vector3N()
                        com_srb.reserve(planningHorizon+1)
                        for i in range(-planningHorizon-1,0):
                            com_srb.pushBack(obs_buffer[i].toVector3(SRB_obs_dim))

                        if False and env.current_frame==107:
                            tbl=edict()
                            tbl.com_srb=com_srb
                            tbl.com=com_target
                            tbl.output=outputMotionBuffer
                            RE.saveTable(tbl, 'plan_com.dat')
                            print('plan.dat saved')


                        ED.removeCOMsliding_online(mLoader_full,  com_srb, com_target, outputMotionBuffer) 

                        #ED.removeCOMsliding_online( com_srb, com_target, outputMotionBuffer) 

                    if this.findWidget('solve limb IK').checkButtonValue():
                        markerTraj=lua.F('fc.getMarkerTraj', lua.instance('markers'), mLoader_full,outputMotionBuffer.sub(-planningHorizon,0,0,0)) 
                        conAll=con_target.sub(-planningHorizon,0,0,0)

                        if feet_traj.rows()==0:
                            feet_traj.assign(markerTraj[0].matView()| markerTraj[1].matView()|markerTraj[2].matView()| markerTraj[3].matView())
                            initialFeet=feet_traj.row(0).copy()
                        else:
                            initialFeet=feet_traj.row(1).copy() # guarantee continuity

                        feet_traj=ED.removeFootSliding_online(footLen, conAll, initialFeet, markerTraj, markerTarget.sub(0,0,0,12),markerTarget.sub(0,0,12,0) , projectToGround)
                        #feet_traj=ED.removeFootSliding_online(footLen, conAll, initialFeet, markerTraj, markerTarget.sub(0,0,0,12))


                        if this.findWidget('draw optimized feet').checkButtonValue():
                            mRobotSkin.setMaterial('lightgrey_transparent')

                            for im in range(4):
                                for f in range(2):
                                    mat='blue'
                                    if conAll[-planningHorizon+1](im)>0.5:
                                        mat='red'
                                    if f==0:
                                        mat=mat+"_transparent"
                                    RE.delayedDraw('SphereM',math.floor(onlineFilterSize/2), feet_traj[-planningHorizon+f].toVector3(im*3),f'optmarker{im}{f}',mat,0.03)
                                RE.delayedDraw('Billboard', math.floor(onlineFilterSize/2), feet_traj.sub(0,0,im*3,(im+1)*3)*100,  'marker'+str(im), 'solidred', 2, 'BillboardLineList' )

                        if useThreads:
                            renderPose=outputMotionBuffer[-(planningHorizon-2)].copy()
                            marker_pos=feet_traj[-(planningHorizon-2)].vec3View()
                        else:
                            renderPose=outputMotionBuffer[-(planningHorizon-1)].copy()
                            marker_pos=feet_traj[-(planningHorizon-1)].vec3View()
                        importance=lua.ones(4)
                        leglen=limbIKsolver('IKsolve', renderPose, marker_pos, importance)
                        mRobotSkin.setLengthAndPoseDOF(leglen, renderPose) 
                        onlineFilterPose.setCurrPose(renderPose)
                        onlineFilterLen.setCurrPose(leglen)

                        # this is the latest fixed pose. later poses can be changed next frame. 
                        currPredictedPose=onlineFilterPose.getFiltered()
                        mRobotSkin.setLengthAndPoseDOF(onlineFilterLen.getFiltered(), currPredictedPose) 

                        if this.findWidget('attach camera').checkButtonValue():
                            print('reattach', reattachCamera)
                            lua.M(RE.viewpoint(),"lookAt", currPredictedPose.toVector3(0), { 'moveGridFloor':True, 'reattach':reattachCamera})
                            reattachCamera=False


                        RE.output('con', con_copy.row(con_copy.rows()-(planningHorizon-1)))
                    else:
                        currPredictedPose=outputMotionBuffer.row(2)
                        mRobotSkin.setPoseDOF(outputMotionBuffer.row(2)) # this is the latest fixed pose. later poses can be changed next frame. 
                    
                    if this.findWidget('draw COM').checkButtonValue():
                        targetCOM=trajOpt.get('com')[-(planningHorizon-1)]
                        mLoader_full.setPoseDOF(renderPose)
                        actualCOM=mLoader_full.calcCOM()
                        RE.delayedDraw('SphereM',math.floor(onlineFilterSize/2), targetCOM,'targetCOM','red',0.2)
                        RE.delayedDraw('SphereM',math.floor(onlineFilterSize/2), actualCOM,'actualCOM','blue',0.2)
                    if currPredictedPose!=None:
                        if hasattr(env,'impulse') and env.impulse>0 :
                            f=RE.toVector3(env.force).ZtoY()
                            if f.length()>1e-3:
                                #mLoader_full.setPoseDOF(currPredictedPose)
                                mLoader_full.setPoseDOF(outputMotionBuffer[-1])

                                if hasattr(env, 'gizmo'):
                                    b=mLoader_full.getBoneByName(env.gizmo[0])
                                    p=b.getFrame()*env.gizmo[1]
                                    velocity_constraints.append([[b.treeIndex(), env.gizmo[1], f*0.005]])
                                else:
                                    p=renderPose.toVector3(0)
                                RE.delayedDraw('Arrow', drawDelay, p*100-f, p*100, 'impulseGizmo')
                                RE.output('impuse', 'draw')

                                #RE.draw('Arrow', p*100-f, p*100, 'impulseGizmo')
                                #IKconfig.vel_con={{self.gizmo, p, f*0.01}}
                        else:
                            #RE.erase('Arrow', 'impulseGizmo')
                            RE.delayedErase('Arrow', drawDelay, 'impulseGizmo')
                            RE.output('impuse', 'erase')
                            velocity_constraints.append([])


                        _lastPose=currPredictedPose.copy()



                
            offset=5
            for site_id in range(offset*2):
                nameid='f'+str(site_id)
                RE.delayedErase('ArrowM',drawDelay, nameid)

            if not env.left_foot_in_contact and not env.right_foot_in_contact:  # if no foot is on the floor, pass
                pass
            else:
                j = 0
                for site_id in env.site_to_apply_force_on:
                    if site_id == -1:
                        continue
                    else:
                        nameid='f'+str(site_id)
                        pos=env.data.site_xpos[site_id]
                        force=env.contact_forces_list[j]
                        scale=0.01
                        RE.delayedDraw('ArrowM',drawDelay, RE.toVector3(pos).ZtoY()+draw_offset_SRB,RE.toVector3(pos+force*scale*0.5).ZtoY()+draw_offset_SRB,nameid,0.05,'green')
                        j += 1
            if this.findWidget('draw mocap SRB').checkButtonValue():
                show_SRB_mocap(viewer, env)
            show_text(viewer, 'frame', env.data.qpos[:3]+np.array([0.0, 0.0, 1.0]), f'current step: {env.current_frame}')
            # obs = env._get_obs()
            # env.current_frame += 1
            if env.current_frame >= len(env.mocap_qpos)-2:
            #if env.current_frame >= 100:
                print(env.current_frame)
                # obs, info = env.reset()  # 重置环境状态
                break
            #time.sleep(1.0/mocap_frame_rate) # unnecessary. already too slow.
        RE.delayedDrawTick()
        if not RE.renderOneFrame(True): break

for i in range(2):
    threads[i].stop()
env.close()  # 关闭环境
