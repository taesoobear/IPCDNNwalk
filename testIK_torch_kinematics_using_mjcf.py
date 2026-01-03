import pdb,sys, os
import torch
import pytorch_kinematics as pk # pip install pytorch-kinematics

from libcalab_ogre3d import RE,m, lua, control


config=lua.Table( skel="work/taesooLib/Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T.wrl",\
    motion="work/taesooLib/Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_locomotion_hl.dof",\
    IKconfig=[
        ['LeftElbow',  m.vector3(0.33, 0, 0)], 
    ],
    skinScale=100,
    initialHeight=0,
)

def eventFunction(ev, i):
    limbIK()

def limbIK():
    global q_all, q_all_orig, idx, chain, target_pos, mCON, T_offset
    q_all=q_all_orig.clone() 

    cpos=mCON.conPos(0)/100
    target_pos = torch.tensor([[cpos.x, cpos.y, cpos.z]], dtype=torch.float32, device=device)

    q_left = q_all[idx]
    # ✅ 초기 joint angle (batch 1, 3 DOF)
    q = torch.tensor(q_left, dtype=torch.float32, device=device, requires_grad=True)

    # ✅ 옵티마이저 설정
    optimizer = torch.optim.Adam([q], lr=0.05)

    # ✅ 반복 학습으로 IK 풀기
    for step in range(100):
        optimizer.zero_grad()

        # Forward kinematics
        T_elbow = chain.forward_kinematics(q)
        T_ee = T_elbow.compose(T_offset) # wrist

        mat=T_ee.get_matrix()
        ee_pos = mat[:,:3,3]  # end-effector 위치 (batch x 3)

        # 손실: 목표 위치와의 L2 거리
        loss = torch.nn.functional.mse_loss(ee_pos, target_pos)

        loss.backward()
        optimizer.step()

        if step % 50 == 0:
            print(f"[{step:03d}] loss={loss.item():.6f}, q={q.detach().cpu().numpy()}")

    print("\n✅ 최종 결과:")
    print("End-effector pos:", ee_pos.detach().cpu().numpy())
    print("Target pos:", target_pos.cpu().numpy())
    print("Joint angles (rad):", q.detach().cpu().numpy())

    q_all[idx]=q.detach()
    mot.skin.setPoseDOF( toQuaterPose(lua.vec(q_all.detach().cpu())))

    if True:
        # debug draw -> buggy
        ret = tree.forward_kinematics(q_all.detach().cpu())
        for k,v  in ret.items():
            ee_pos = v.get_matrix()[:,:3,3]  # end-effector 위치 (batch x 3)
            print(ee_pos)
            RE.namedDraw('SphereM', RE.toVector3(ee_pos.detach().cpu()[0]),k,'blue')

    RE.draw('SphereM', RE.toVector3(ee_pos.detach().cpu()[0]),'resulting_pos','blue')

def handleRendererEvent(ev, button, x,y) :
    global mCON
    if mCON :
        return mCON.handleRendererEvent(ev, button, x,y)

    return 0

def loadMotion(skel, motion, skinScale=None):
    mot=lua.Table()
    mot.loader=RE.WRLloader(skel)
    mot.motionDOFcontainer=m.MotionDOFcontainer(mot.loader.dofInfo, motion)
    if skinScale :
        mot.skin=RE.createSkin(mot.loader)
        mot.skin.setScale( skinScale)
        #mot.skin.applyMotionDOF(mot.motionDOFcontainer.mot)
        mot.skin.setMaterial('lightgrey_transparent')

    return mot



if torch.cuda.is_available():
    device = torch.device("cuda")
elif torch.backends.mps.is_available():
    device = torch.device("mps")
else:
    device = torch.device("cpu")
# cpu is faster for this 
device = torch.device("cpu")

this=RE.createMainWin(sys.argv)

mot=loadMotion(config.skel, config.motion, 100)

def toEulerPose(pose):
    q=pose.toQuater(3)
    euler=m.vector3()
    q.getRotation('YZX', euler)

    pose2=pose.slice(1,0).copy()
    pose2.setVec3(0, pose.toVector3(0))
    pose2.setVec3(3, euler)
    return pose2
def toQuaterPose(pose2):
    pose=m.vectorn(pose2.size()+1)

    q=m.quater()
    euler=pose2.toVector3(3)
    q.setRotation('YZX', euler)
    pose.slice(1,0).assign(pose2)
    pose.setVec3(0, pose2.toVector3(0))
    pose.setQuater(3, q)
    return pose


# euler skeleton (with removeCompositeJoints ==True such that the free root joint is represented using euler angles and translations)

mjcf_string=RE.createMujocoXMLstring(mot.loader, removeCompositeJoints=True) # free root joint --> XYZ slide + YZX euler joints
print(mjcf_string)
tree=pk.build_chain_from_mjcf(mjcf_string)
print(tree)
tree.print_tree()
chain = pk.SerialChain(tree, config.IKconfig[0][0], "world")
chain=chain.to(dtype=torch.float32, device=device)

T_offset=pk.transforms.Transform3d(
            rot=torch.eye(3),
            pos=torch.tensor(config.IKconfig[0][1].array)
        )
T_offset=T_offset.to(dtype=torch.float32, device=device)


print(chain)
# prints out list of joint names
print(chain.get_joint_parameter_names())

poseDOF=mot.motionDOFcontainer.row(0)
mot.loader.setPoseDOF(poseDOF)
poseEuler=toEulerPose(poseDOF)

print('ctor finished')

q_all = torch.tensor(poseEuler.array, dtype=torch.float32, device=device)
q_all_orig=q_all.clone()

all_joint_names=tree.get_joint_parameter_names()
left_arm_names=chain.get_joint_parameter_names()

# 오른팔 체인 joint 인덱스
idx = [all_joint_names.index(name) for name in left_arm_names]

# ✅ 목표 end-effector 위치
target_pos = torch.tensor([[0.5, 0.5, 0.8]], dtype=torch.float32, device=device)
originalPos=RE.toVector3(target_pos[0])

mCON=RE.Constraints([originalPos*100])
mCON.connect(__name__, "eventFunction")  # 현재 모듈(__name__)의 eventFunction 함수(위에 정의됨)에 event 연결.

m.startMainLoop() # this finishes when program finishes

