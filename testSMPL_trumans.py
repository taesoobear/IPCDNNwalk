import pickle, pdb, os
from libcalab_ogre3d import RE, m, lua, control
import numpy as np
from mathutils import Vector, Quaternion # pip install mathutils

datasetRoot=RE.path('../../d/sample_SAMP') # RE.path returns pathlib.Path (normalized)
trumans_motion_path=RE.path('../../d/trumans_utils/drive_download/smplx_result/2023-02-19@22-41-20_smplx_results.pkl')
with open(trumans_motion_path, "rb") as f:
    zfile = pickle.load(f, encoding="latin1")

model_path=RE.path('../../d/sample_SAMP/SMPL-X/model1.1/models')  # path to SMPL models
bm_path=model_path/'smplx/SMPLX_male.npz' 


ui=RE.createMainWin()
ui.updateLayout()

def onFrameChanged( iframe):
    global skin, mot
    if iframe<mot.numFrames() :
        skin.setPose(mot.pose(iframe))

if 'betas' in zfile and zfile['betas'].size>0:
    fbx=RE.createSMPLskeleton(str(bm_path), zfile['betas'])
else:
    fbx=RE.createSMPLskeleton(str(bm_path))
pelvis_offset=fbx.loader.bone(1).getFrame().translation.copy()

def get_quat_from_rodrigues(rodrigues):
    rod = Vector((rodrigues[0], rodrigues[1], rodrigues[2]))
    angle_rad = rod.length
    axis = rod.normalized()
    quat = Quaternion(axis, angle_rad)
    return quat
def get_quat_from_rodrigues2(rodrigues, rodrigues_reference):
    return get_quat_from_rodrigues(rodrigues+rodrigues_reference)


def load_trumans_animation(data, fbx, root_offset):
    SMPLX_JOINT_NAMES = [
            'pelvis','left_hip','right_hip','spine1','left_knee','right_knee','spine2','left_ankle','right_ankle','spine3', 'left_foot','right_foot','neck','left_collar','right_collar','head','left_shoulder','right_shoulder','left_elbow', 'right_elbow','left_wrist','right_wrist',
            'jaw','left_eye_smplhf','right_eye_smplhf','left_index1','left_index2','left_index3','left_middle1','left_middle2','left_middle3','left_pinky1','left_pinky2','left_pinky3','left_ring1','left_ring2','left_ring3','left_thumb1','left_thumb2','left_thumb3','right_index1','right_index2','right_index3','right_middle1','right_middle2','right_middle3','right_pinky1','right_pinky2','right_pinky3','right_ring1','right_ring2','right_ring3','right_thumb1','right_thumb2','right_thumb3'
            ]
    NUM_SMPLX_JOINTS = len(SMPLX_JOINT_NAMES)
    NUM_SMPLX_BODYJOINTS = 21
    NUM_SMPLX_HANDJOINTS = 15

    LEFT_HAND_POSE = np.array([ 0.11167871,  0.04289218, -0.41644183,  0.10881133, -0.06598568, -0.75622   , -0.09639297, -0.09091566, -0.18845929, -0.11809504, 0.05094385, -0.5295845 , -0.14369841,  0.0552417 , -0.7048571 , -0.01918292, -0.09233685, -0.3379135 , -0.45703298, -0.19628395, -0.6254575 , -0.21465237, -0.06599829, -0.50689423, -0.36972436, -0.06034463, -0.07949023, -0.1418697 , -0.08585263, -0.63552827, -0.3033416 , -0.05788098, -0.6313892 , -0.17612089, -0.13209307, -0.37335458,  0.8509643 ,  0.27692273, -0.09154807, -0.49983943, 0.02655647,  0.05288088,  0.5355592 ,  0.04596104, -0.27735803])

    RIGHT_HAND_POSE = np.array([ 0.11167871, -0.04289218,  0.41644183,  0.10881133,  0.06598568, 0.75622   , -0.09639297,  0.09091566,  0.18845929, -0.11809504, -0.05094385,  0.5295845 , -0.14369841, -0.0552417 ,  0.7048571 , -0.01918292,  0.09233685,  0.3379135 , -0.45703298,  0.19628395, 0.6254575 , -0.21465237,  0.06599829,  0.50689423, -0.36972436, 0.06034463,  0.07949023, -0.1418697 ,  0.08585263,  0.63552827, -0.3033416 ,  0.05788098,  0.6313892 , -0.17612089,  0.13209307, 0.37335458,  0.8509643 , -0.27692273,  0.09154807, -0.49983943, -0.02655647, -0.05288088,  0.5355592 , -0.04596104,  0.27735803])


    hand_pose_relaxed = np.concatenate((LEFT_HAND_POSE, RIGHT_HAND_POSE)).reshape(NUM_SMPLX_HANDJOINTS * 2, 3)


    translation = None
    global_orient = None

    if "transl" in data:
        translation = np.array(data["transl"]).reshape(-1, 3)

    if "global_orient" in data:
        global_orient = np.array(data["global_orient"]).reshape(-1, 3)

    body_pose = np.array(data["body_pose"]).reshape(-1, NUM_SMPLX_BODYJOINTS, 3)
    left_hand_pose = np.zeros((body_pose.shape[0], NUM_SMPLX_HANDJOINTS, 3))
    right_hand_pose = np.zeros((body_pose.shape[0], NUM_SMPLX_HANDJOINTS, 3))

    num_keyframes = len(body_pose)


    motion=m.Motion(fbx.loader)
    motion.resize(num_keyframes)

    global_orients = np.array([get_quat_from_rodrigues(rodrigues) for rodrigues in global_orient])
    body_poses = {bone_name: np.array([get_quat_from_rodrigues(rodrigues) for rodrigues in body_pose[:, index, :]])
                  for index, bone_name in enumerate(SMPLX_JOINT_NAMES[1: NUM_SMPLX_BODYJOINTS + 1])}

    start_name_index = 1 + NUM_SMPLX_BODYJOINTS + 3
    left_hand_poses = {bone_name: np.array([get_quat_from_rodrigues2(rodrigues, hand_pose_relaxed[i]) for rodrigues in left_hand_pose[:, i, :]])
                       for i, bone_name in enumerate(SMPLX_JOINT_NAMES[start_name_index: start_name_index + NUM_SMPLX_HANDJOINTS])}

    start_name_index = 1 + NUM_SMPLX_BODYJOINTS + 3 + NUM_SMPLX_HANDJOINTS
    right_hand_poses = {bone_name: np.array([get_quat_from_rodrigues2(rodrigues, hand_pose_relaxed[NUM_SMPLX_HANDJOINTS + i]) for rodrigues in right_hand_pose[:, i, :]])
                        for i, bone_name in enumerate(SMPLX_JOINT_NAMES[start_name_index: start_name_index + NUM_SMPLX_HANDJOINTS])}

    body_poses = {**body_poses, **left_hand_poses, **right_hand_poses}
    body_poses['pelvis'] = global_orients


    for i in range(num_keyframes):
        motion.pose(i).translations(0).assign(RE.toVector3(translation[i]))


    for bone_name, quaternions in body_poses.items():
        ri=fbx.loader.getRotJointIndexByName(bone_name)
        for i in range(num_keyframes):
            motion.pose(i).rotations(ri).assign(RE.toQuater(quaternions[i]))

    motion.translate(root_offset)
    return motion


mot=load_trumans_animation(zfile, fbx, pelvis_offset)


mTimeline=RE.Timeline("Timeline", mot.numFrames(), 1/mot.frameRate())
skinScale=100 # rendering in cm unit.

drawSkeleton=True
skin=RE.createFBXskin(fbx, drawSkeleton)
skin.setScale(skinScale, skinScale, skinScale)

while True:
    checkEvents=True
    if not RE.renderOneFrame(checkEvents): break

