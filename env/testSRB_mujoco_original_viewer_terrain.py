import platform
if platform.machine()=='ARM64':
    import mujoco
else:
    # i think mujoco.viewer needs to be imported only where absolutely needed. For example, inside the play function.
    import mujoco.viewer
import numpy as np
import time
from scipy.linalg import logm, block_diag
from scipy import sparse
import clarabel
import transformations

"""
    用来在mujoco中展示reference motion和一些函数， SRBdata类将作为SRBEnv的数据处理类
    Used to present reference motion and some functions in mujoco, the SRBdata class will serve as SRBEnv's data processing class
"""


def foot_contact_to_one_hot(foot_in_contact):
    """
    将两个布尔值的列表转换为4维独热编码。

    参数:
        foot_in_contact (list): 一个包含两个布尔值的列表 [左脚是否接触, 右脚是否接触]。

    返回:
        np.ndarray: 4维独热编码的numpy数组。
    """
    # 使用布尔值直接计算索引
    if foot_in_contact == [False, False]:
        index = 0
    elif foot_in_contact == [True, False]:
        index = 1
    elif foot_in_contact == [False, True]:
        index = 2
    elif foot_in_contact == [True, True]:
        index = 3
    else:
        raise ValueError("输入值必须是布尔值列表，例如 [False, False]")

    # 初始化一个长度为4的独热编码数组
    one_hot = np.zeros(4, dtype=int)

    # 将对应索引置为1
    one_hot[index] = 1

    return one_hot


def rotation_matrix_angle(R1, R2):
    """
    计算两个旋转矩阵之间的最小旋转角度

    参数:
    R1 (numpy.ndarray): 第一个旋转矩阵 (3x3)
    R2 (numpy.ndarray): 第二个旋转矩阵 (3x3)

    返回:
    float: 两个旋转矩阵之间的最小旋转角度（以弧度为单位）
    """
    # 计算相对旋转矩阵
    R_rel = R1.T @ R2

    # 计算旋转角度
    trace_R_rel = np.trace(R_rel)
    cos_theta = (trace_R_rel - 1) / 2

    # 修正 cos_theta 的范围到 [-1, 1]
    cos_theta = np.clip(cos_theta, -1.0, 1.0)

    angle = np.arccos(cos_theta)

    return angle


def global2SRB(data, cdm_body_id, SRB_origin, global_pos):
    cdm_ori = data.xmat[cdm_body_id].copy().reshape(3, 3)
    SRB_ori = np.array(cdm_ori)
    local_pos_SRB = SRB_ori.T @ (global_pos - SRB_origin)
    return local_pos_SRB, SRB_ori


def global2forward_facing_SRB(data, cdm_body_id, SRB_origin, global_pos):
    cdm_ori = data.xmat[cdm_body_id].copy().reshape(3, 3)
    SRB_ori = np.array(cdm_ori)
    x_SRB = SRB_ori[:, 0]
    x_proj = np.array([x_SRB[0], x_SRB[1], 0])
    if np.linalg.norm(x_proj) != 0:
        x_ffSRB = x_proj / np.linalg.norm(x_proj)
    else:
        x_ffSRB = x_proj
    z_ffSRB = np.array([0, 0, 1])
    y_ffSRB = np.cross(z_ffSRB, x_ffSRB)
    ffSRB_ori = np.column_stack((x_ffSRB, y_ffSRB, z_ffSRB))
    local_pos_ffSRB = ffSRB_ori.T @ (global_pos - SRB_origin)
    return local_pos_ffSRB, ffSRB_ori


def global2projected_SRB(data, cdm_body_id, SRB_origin, global_pos):
    cdm_ori = data.xmat[cdm_body_id].copy().reshape(3, 3)
    SRB_ori = np.array(cdm_ori)
    x_SRB = SRB_ori[:, 0]
    x_proj = np.array([x_SRB[0], x_SRB[1], 0])
    if np.linalg.norm(x_proj) != 0:
        x_ffSRB = x_proj / np.linalg.norm(x_proj)
    else:
        x_ffSRB = x_proj
    z_ffSRB = np.array([0, 0, 1])
    y_ffSRB = np.cross(z_ffSRB, x_ffSRB)
    ffSRB_ori = np.column_stack((x_ffSRB, y_ffSRB, z_ffSRB))
    SRB_origin_proj = SRB_origin.copy()
    SRB_origin_proj[2] = 0
    local_pos_SRB_proj = ffSRB_ori.T @ (global_pos - SRB_origin_proj)
    return local_pos_SRB_proj, ffSRB_ori


def global2SRB_mocap(SRB_rot_mat, SRB_origin, global_pos):
    cdm_ori = SRB_rot_mat
    SRB_ori = np.array(cdm_ori)
    local_pos_SRB = SRB_ori.T @ (global_pos - SRB_origin)
    return local_pos_SRB, SRB_ori


def global2projected_SRB_mocap(SRB_rot_mat, SRB_origin, global_pos):
    cdm_ori = SRB_rot_mat
    SRB_ori = np.array(cdm_ori)
    x_SRB = SRB_ori[:, 0]
    x_proj = np.array([x_SRB[0], x_SRB[1], 0])
    if np.linalg.norm(x_proj) != 0:
        x_ffSRB = x_proj / np.linalg.norm(x_proj)
    else:
        x_ffSRB = x_proj
    z_ffSRB = np.array([0, 0, 1])
    y_ffSRB = np.cross(z_ffSRB, x_ffSRB)
    ffSRB_ori = np.column_stack((x_ffSRB, y_ffSRB, z_ffSRB))
    SRB_origin_proj = SRB_origin.copy()
    SRB_origin_proj[2] = 0
    local_pos_SRB_proj = ffSRB_ori.T @ (global_pos - SRB_origin_proj)
    return local_pos_SRB_proj, ffSRB_ori


def global2forward_facing_SRB_mocap(SRB_rot_mat, SRB_origin, global_pos):
    cdm_ori = SRB_rot_mat
    SRB_ori = np.array(cdm_ori)
    x_SRB = SRB_ori[:, 0]
    x_proj = np.array([x_SRB[0], x_SRB[1], 0])
    if np.linalg.norm(x_proj) != 0:
        x_ffSRB = x_proj / np.linalg.norm(x_proj)
    else:
        x_ffSRB = x_proj
    z_ffSRB = np.array([0, 0, 1])
    y_ffSRB = np.cross(z_ffSRB, x_ffSRB)
    ffSRB_ori = np.column_stack((x_ffSRB, y_ffSRB, z_ffSRB))
    local_pos_ffSRB = ffSRB_ori.T @ (global_pos - SRB_origin)
    return local_pos_ffSRB, ffSRB_ori


def integrate_rotation_z_axis(rot_matrix, angular_velocity_z, dt):
    """
    专门处理绕z轴旋转的旋转矩阵更新。
    """
    delta_angle = angular_velocity_z * dt
    delta_rot_matrix = z_axis_rotation_matrix(delta_angle)
    return delta_rot_matrix @ rot_matrix


def transformation_matrix(R, p):  # create transformation matrix
    """Create a homogeneous transformation matrix from rotation matrix R and translation vector p."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def log_se3(T_t, T_hat):  # log function for converting SE(3) to se(3)
    """
       Compute the logarithm of the relative transformation matrix T_t^-1 * T_hat.

       Parameters:
       - T_t: SE3, current transformation matrix
       - T_hat: SE3, reference transformation matrix

       Returns:
       - xi: 6x1 numpy array, the generalized velocity (twist) representing the log of the transformation
       """
    T_t_inv = np.linalg.inv(T_t)
    T_rel = np.dot(T_t_inv, T_hat)

    log_term = logm(T_rel)
    omega_hat = log_term[:3, :3]  # 3x3 skew-symmetric matrix
    v = log_term[:3, 3]  # 3x1 translation vector

    # Convert the skew-symmetric matrix to a vector
    omega = np.array([omega_hat[2, 1], omega_hat[0, 2], omega_hat[1, 0]])

    # Combine omega and v into a single twist vector
    twist = np.zeros(6)
    twist[:3] = v  # Translation part
    twist[3:] = omega  # Rotation part

    return twist


def rotation_matrix_from_vectors(vec1, vec2):
    """ Find the rotation matrix that aligns vec1 to vec2 """
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + np.dot(kmat, kmat) * ((1 - c) / (s ** 2))
    return rotation_matrix


def add_force_vector(viewer, pos, force, scale=0.01):  # visualize forces
    """
    在指定位置添加一个箭头，表示施加的力。

    参数:
    - viewer: Mujoco viewer 对象
    - pos: 力的施加位置 (3,)
    - force: 力向量 (3,)
    - scale: 缩放因子，用于调整箭头的长度
    """
    # 计算旋转矩阵，将 z 轴对齐到力的方向
    z_axis = np.array([0, 0, 1])
    rotation_matrix = rotation_matrix_from_vectors(z_axis, force)

    # 创建表示力的箭头几何体
    geom = viewer.user_scn.geoms[viewer.user_scn.ngeom]
    mujoco.mjv_initGeom(
        geom,
        type=mujoco.mjtGeom.mjGEOM_ARROW,
        size=np.array([0.01, 0.01, np.linalg.norm(force) * scale]),  # 箭头的尺寸
        pos=pos,  # 箭头的起点位置
        mat=rotation_matrix.flatten(),  # 箭头的方向
        rgba=np.array([0, 0, 1, 1])  # 蓝色箭头表示力
    )
    viewer.user_scn.ngeom += 1


def add_force_vector_external(viewer, pos, force, scale=0.01):  # visualize forces
    """
    在指定位置添加一个箭头，表示施加的力。

    参数:
    - viewer: Mujoco viewer 对象
    - pos: 力的施加位置 (3,)
    - force: 力向量 (3,)
    - scale: 缩放因子，用于调整箭头的长度
    """
    # 计算旋转矩阵，将 z 轴对齐到力的方向
    z_axis = np.array([0, 0, 1])
    rotation_matrix = rotation_matrix_from_vectors(z_axis, force)

    # 创建表示力的箭头几何体
    geom = viewer.user_scn.geoms[viewer.user_scn.ngeom]
    mujoco.mjv_initGeom(
        geom,
        type=mujoco.mjtGeom.mjGEOM_ARROW,
        size=np.array([0.05, 0.05, np.linalg.norm(force) * scale]),  # 箭头的尺寸
        pos=pos,  # 箭头的起点位置
        mat=rotation_matrix.flatten(),  # 箭头的方向
        rgba=np.array([0, 1, 0, 1])  # 绿色箭头表示力
    )
    viewer.user_scn.ngeom += 1


# get the rotation against z axis
def get_z_rotation_matrix(xmat):
    """
    从 xmat 提取绕 z 轴的旋转矩阵
    """
    # 提取 xmat 中的旋转分量
    r11 = xmat[0, 0]
    r21 = xmat[1, 0]

    # 使用 atan2 计算旋转角度
    theta = np.arctan2(r21, r11)

    # 构建只包含 z 轴旋转的矩阵
    z_rot_matrix = np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])

    return z_rot_matrix


def convert_matrix_to_6d(rot_matrix):
    # Extract the first and second columns of rotation matrix
    rot_6d = np.concatenate([rot_matrix[:, 0], rot_matrix[:, 1]], axis=-1)
    return rot_6d


def z_axis_rotation_matrix(angle):
    cos_theta = np.cos(angle)
    sin_theta = np.sin(angle)

    # Rotation matrix for z-axis
    rotation_matrix = np.array([
        [cos_theta, -sin_theta, 0],
        [sin_theta, cos_theta, 0],
        [0, 0, 1]
    ])
    return rotation_matrix


def axis_angle_to_rotation_matrix(axis_angle):
    # Compute the rotation angle (magnitude of the axis-angle vector)
    angle = np.linalg.norm(axis_angle)
    if angle < 1e-8:  # If angle is very small, return the identity matrix
        return np.eye(3)

    # Normalize the axis
    axis = axis_angle / angle

    # Rodrigues' rotation formula
    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0]
    ])
    I = np.eye(3)
    R = I + np.sin(angle) * K + (1 - np.cos(angle)) * np.dot(K, K)
    return R


def get_feet_state(model, data):
    feet_list = ['left_foot', 'right_foot']
    feet_pos_list = []
    feet_ori_list = []
    for foot in feet_list:
        foot_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, foot)
        temp_foot_pos = data.geom_xpos[foot_id].copy()
        temp_foot_ori = data.geom_xmat[foot_id].reshape(3, 3).copy()
        temp_foot_ori_z = get_z_rotation_matrix(temp_foot_ori)
        feet_pos_list.append(temp_foot_pos)
        feet_ori_list.append(temp_foot_ori_z)
    return feet_pos_list, feet_ori_list


# this is different from the one used in env
def set_feet_site_this_step(model, data, feet_pos_list_this_step, feet_ori_list_this_step, desired_left_foot_pos,
                            desired_right_foot_pos):
    feet_site_list_SRB = ['left_foot_center', 'left_foot_lf', 'left_foot_rf', 'left_foot_lb', 'left_foot_rb',
                          'right_foot_center', 'right_foot_lf', 'right_foot_rf', 'right_foot_lb', 'right_foot_rb']
    feet_pos_from_action = np.vstack((desired_left_foot_pos, desired_right_foot_pos))
    i = 0
    j = 0
    offset = 5
    site_to_apply_force_on = [-1] * len(feet_site_list_SRB)  # 使用 -1 作为占位符/Use -1 as a placeholder

    # the relative pos betweenn site_center and site_front,site_back
    # rel_lf = np.array([0.25, 0.1, 0])
    # rel_rf = np.array([0.25, -0.1, 0])
    # rel_lb = np.array([-0.25, 0.1, 0])
    # rel_rb = np.array([-0.25, -0.1, 0])
    rel_lf = np.array([0.12, 0.06, 0])
    rel_rf = np.array([0.12, -0.06, 0])
    rel_lb = np.array([-0.12, 0.06, 0])
    rel_rb = np.array([-0.12, -0.06, 0])


    while j < len(feet_site_list_SRB):
        # 如果 z 坐标不为 0，则该foot处于swing state，同样设置该foot pos为action输出，但是标为-1
        # If the z coordinate is not 0, then the foot is in swing state, and the foot pos is also set to the action output, but marked -1
        if feet_pos_list_this_step[i][2] != 0:
            site_ids_to_hide = [
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, feet_site_list_SRB[j]),
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, feet_site_list_SRB[j + 1]),
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, feet_site_list_SRB[j + 2]),
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, feet_site_list_SRB[j + 3]),
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, feet_site_list_SRB[j + 4])
            ]

            for k, site_id in enumerate(site_ids_to_hide):
                if i == 0:
                    model.site_rgba[site_id] = [1, 0, 0, 0.1]
                else:
                    model.site_rgba[site_id] = [0, 1, 0, 0.1]
                site_to_apply_force_on[j + k] = -1
            data.site_xmat[site_ids_to_hide[0]] = feet_ori_list_this_step[i].flatten()
            data.site_xpos[site_ids_to_hide[0]] = feet_pos_from_action[i]
            data.site_xpos[site_ids_to_hide[1]] = feet_pos_from_action[i] + feet_ori_list_this_step[i] @ rel_lf
            data.site_xpos[site_ids_to_hide[2]] = feet_pos_from_action[i] + feet_ori_list_this_step[i] @ rel_rf
            data.site_xpos[site_ids_to_hide[3]] = feet_pos_from_action[i] + feet_ori_list_this_step[i] @ rel_lb
            data.site_xpos[site_ids_to_hide[4]] = feet_pos_from_action[i] + feet_ori_list_this_step[i] @ rel_rb
            j += offset
            i += 1
            continue

        site_ids_to_show = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, feet_site_list_SRB[j]),
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, feet_site_list_SRB[j + 1]),
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, feet_site_list_SRB[j + 2]),
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, feet_site_list_SRB[j + 3]),
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, feet_site_list_SRB[j + 4])
        ]

        for k, site_id in enumerate(site_ids_to_show):
            if i == 0:
                model.site_rgba[site_id] = [1, 0, 0, 1]
            else:
                model.site_rgba[site_id] = [0, 1, 0, 1]
            site_to_apply_force_on[j + k] = site_id
        data.site_xmat[site_ids_to_show[0]] = feet_ori_list_this_step[i].flatten()
        data.site_xpos[site_ids_to_show[0]] = feet_pos_from_action[i]
        data.site_xpos[site_ids_to_show[1]] = feet_pos_from_action[i] + feet_ori_list_this_step[i] @ rel_lf
        data.site_xpos[site_ids_to_show[2]] = feet_pos_from_action[i] + feet_ori_list_this_step[i] @ rel_rf
        data.site_xpos[site_ids_to_show[3]] = feet_pos_from_action[i] + feet_ori_list_this_step[i] @ rel_lb
        data.site_xpos[site_ids_to_show[4]] = feet_pos_from_action[i] + feet_ori_list_this_step[i] @ rel_rb
        j += offset
        i += 1


def return_site_to_apply_force_on(model, feet_pos_list_this_step):
    feet_site_list_SRB = ['left_foot_center', 'left_foot_lf', 'left_foot_rf', 'left_foot_lb', 'left_foot_rb',
                          'right_foot_center', 'right_foot_lf', 'right_foot_rf', 'right_foot_lb', 'right_foot_rb']
    i = 0
    j = 0
    offset = 5
    site_to_apply_force_on = [-1] * len(feet_site_list_SRB)

    while j < len(feet_site_list_SRB):
        if feet_pos_list_this_step[i][2] != 0:
            j += offset
            i += 1
            continue

        site_ids_to_show = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, feet_site_list_SRB[j]),
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, feet_site_list_SRB[j + 1]),
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, feet_site_list_SRB[j + 2]),
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, feet_site_list_SRB[j + 3]),
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, feet_site_list_SRB[j + 4])
        ]
        for k, site_id in enumerate(site_ids_to_show):
            site_to_apply_force_on[j + k] = site_id  # 设置为 site id
        j += offset
        i += 1

    return site_to_apply_force_on


# get jacobian for all sites on the floor
def get_site_jacobians(model, data, site_to_apply_force_on):
    jacp_list = []
    jacr_list = []
    for site_id in site_to_apply_force_on:
        if site_id != -1:
            jacp = np.zeros((3, model.nv))
            jacr = np.zeros((3, model.nv))
            site_pos = data.site_xpos[site_id]
            site_body_id = model.site_bodyid[site_id]
            mujoco.mj_jac(model, data, jacp, jacr, site_pos, site_body_id)
            # mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
            jacp_list.append(jacp)
            jacr_list.append(jacr)
        else:
            jacp_list.append(None)
            jacr_list.append(None)
    return jacp_list, jacr_list


# set force at a site
def apply_force_at_site(model, data, site_id, force):

    # Get the body id to which the site is attached
    body_id = model.site_bodyid[site_id]

    # Get the site's position in the global frame
    site_pos = data.site_xpos[site_id]

    # Create torque=0
    torque = np.zeros(3, dtype=np.float64).reshape(3, 1)

    # Apply the force and torque using mj_applyFT
    mujoco.mj_applyFT(model, data, force, torque, site_pos, body_id, data.qfrc_applied)


def print_mass_matrix(model, data):  # get mass matrix
    full_mass_matrix = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, full_mass_matrix, data.qM)
    return full_mass_matrix


# get contact force from QP
def solve_qp_for_contact_forces(model, data, curr_T_mocap, target_vel, site_to_apply_force_on, mu):
    nv = model.nv
    M = print_mass_matrix(model, data)
    b = data.qfrc_bias.copy()

    cdm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'cdm')

    _, curr_projSRB_ori = global2projected_SRB(data, cdm_id, data.qpos[:3], data.qpos[:3])
    # 接触力基底
    b1 = np.array([1.2*mu, 0, 1])
    b2 = np.array([-1.2*mu, 0, 1])
    b3 = np.array([0, 1.2*mu, 1])
    b4 = np.array([0, -1.2*mu, 1])
    b_list = [b1, b2,  b3, b4]
    B_list = [curr_projSRB_ori@b for b in b_list]
    B = np.vstack(B_list).T
    # B = np.vstack((b1, b2, b3, b4)).T

    # 获取期望加速度 ddq_d
    Tt = transformation_matrix(data.xmat[cdm_id].copy().reshape(3, 3), data.qpos[:3])
    Tt_mocap = curr_T_mocap
    local_twist = log_se3(Tt, Tt_mocap)
    local_twist = np.concatenate([data.xmat[cdm_id].reshape(3, 3)@local_twist[:3], local_twist[3:]])  # global linvel and local angvel
    q_ddot_d = 120 * local_twist + 35 * (target_vel - data.qvel.copy())  # 120, 35

    # local_ang_vel = data.qvel[3:]
    # global_ang_vel = data.xmat[cdm_id].reshape(3, 3)@local_ang_vel
    # data_qvel = np.concatenate([data.qvel[:3].copy(), global_ang_vel])
    # q_ddot_d = 120 * log_se3(Tt, Tt_mocap) + 35 * (target_vel - data_qvel)

    # 存储每个 site 的雅可比矩阵
    jacp_list, _ = get_site_jacobians(model, data, site_to_apply_force_on)
    jacp_list = [j for j in jacp_list if j is not None]

    # 构建目标函数的参数
    w_lambda = 0.001  # 0.001
    G_q = 2 * np.eye(nv)
    G_lambda = 2 * w_lambda * np.eye(4 * len(jacp_list))
    G = block_diag(G_q, G_lambda)

    a_q = -2 * q_ddot_d
    a_lambda = np.zeros(4 * len(jacp_list))
    a = np.concatenate([a_q, a_lambda])

    # 构建等式约束
    C_eq = np.zeros((nv, 4 * len(jacp_list)))
    for i, jacp in enumerate(jacp_list):
        C_eq[:, 4 * i:4 * (i + 1)] = jacp.T @ B
    A_eq = sparse.csc_matrix(np.hstack((M, -C_eq)))
    b_eq = -b

    # 构建不等式约束
    num_lambda = 4 * len(jacp_list)
    A_ineq = sparse.csc_matrix(np.hstack((np.zeros((num_lambda, nv)), np.eye(num_lambda))))
    b_ineq = np.zeros(num_lambda)

    # 合并约束
    A = -sparse.vstack([A_eq, A_ineq]).tocsc()
    b_qp = -np.concatenate([b_eq, b_ineq])

    # 定义 Clarabel 优化问题
    P = sparse.csc_matrix(G)
    P = sparse.triu(P).tocsc()
    q = a

    cones = [clarabel.ZeroConeT(b_eq.shape[0]),
             clarabel.NonnegativeConeT(b_ineq.shape[0])]

    settings = clarabel.DefaultSettings()
    settings.verbose = False
    solver = clarabel.DefaultSolver(P, q, A, b_qp, cones, settings)

    # 解决问题
    # t1 = time.perf_counter()
    solution = solver.solve()
    # t2 = time.perf_counter()
    # print(t2-t1)
    x = np.array(solution.x)  # 将 solution.x 转换为 NumPy 数组
    q_ddot_QP = x[:nv]
    lambda_list_QP = x[nv:].reshape(-1, 4)

    # 计算接触力
    contact_forces_list = [B @ lambda_QP for lambda_QP in lambda_list_QP]

    return q_ddot_QP, lambda_list_QP, contact_forces_list


def visual_set(viewer, lookat):
    """设置视觉参数和相机位置"""
    opt = mujoco.MjvOption()
    opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = False
    viewer.cam.azimuth = 135  # 相机方位角
    viewer.cam.elevation = -20  # 相机仰角
    viewer.cam.distance = 3.0  # 相机距离
    viewer.cam.lookat[:] = lookat  # 相机注视点


def project_to_forward_facing(global_rot_matrix):
    """
    将全局旋转矩阵投射到仅绕竖直轴（z 轴）旋转的forward facing帧。

    参数:
        global_rot_matrix: 3x3 的全局旋转矩阵

    返回:
        3x3 的旋转矩阵，仅包含绕 z 轴的旋转。
    """
    # 将输入的旋转矩阵转换为欧拉角（轴顺序为 xyz）
    # 这里假设欧拉角顺序为 'xyz'，即先绕 x 再绕 y 最后绕 z
    global_transformation_mat = np.eye(4)
    global_transformation_mat[:3, :3] = global_rot_matrix
    euler_xyz = transformations.euler_from_matrix(global_transformation_mat, 'rxyz')

    # 将 roll 和 pitch 清零，只保留 yaw（绕 z 轴旋转）
    roll = 0.0
    pitch = 0.0
    yaw = euler_xyz[2]

    # 根据纯偏航角重新构建旋转矩阵
    forward_facing_matrix = transformations.euler_matrix(roll, pitch, yaw, 'rxyz')[:3, :3]

    return forward_facing_matrix


def rotation_matrix_to_quaternion(R):
    """
    将旋转矩阵（3x3）转换为四元数 [w, x, y, z]
    """
    m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
    m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
    m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]

    trace = m00 + m11 + m22

    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m21 - m12) * s
        y = (m02 - m20) * s
        z = (m10 - m01) * s
    elif (m00 > m11) and (m00 > m22):
        s = 2.0 * np.sqrt(1.0 + m00 - m11 - m22)
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = 2.0 * np.sqrt(1.0 + m11 - m00 - m22)
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m22 - m00 - m11)
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s

    q = np.array([w, x, y, z], dtype=np.float64)
    return q / np.linalg.norm(q)


def quaternion_to_rotation_matrix(q):
    """
    将单位四元数 [w, x, y, z] 转换为旋转矩阵（3x3）
    """
    w, x, y, z = q
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z

    R = np.array([
        [1 - 2 * (yy + zz),     2 * (xy - wz),       2 * (xz + wy)],
        [2 * (xy + wz),         1 - 2 * (xx + zz),   2 * (yz - wx)],
        [2 * (xz - wy),         2 * (yz + wx),       1 - 2 * (xx + yy)]
    ], dtype=np.float64)
    return R


def average_rotation_matrix(R1, R2):
    """
    平均两个旋转矩阵，返回新的旋转矩阵
    """
    # 转为四元数
    q1 = rotation_matrix_to_quaternion(R1)
    q2 = rotation_matrix_to_quaternion(R2)

    # 处理四元数符号一致性（避免取反的四元数干扰平均）
    if np.dot(q1, q2) < 0:
        q2 = -q2

    # 简单平均（也可以自定义权重）
    q_avg = q1 + q2

    # 归一化
    q_avg /= np.linalg.norm(q_avg)

    # 转回旋转矩阵
    R_avg = quaternion_to_rotation_matrix(q_avg)

    return R_avg


def random_quaternion_perturbation(reset_noise_scale=5e-2):
    # 随机旋转轴（归一化向量）
    axis = np.random.randn(3)
    axis /= np.linalg.norm(axis)

    # 随机小角度扰动
    angle = np.random.uniform(-reset_noise_scale, reset_noise_scale)

    # 生成四元数（w, x, y, z）
    w = np.cos(angle / 2.0)
    xyz = axis * np.sin(angle / 2.0)

    return np.array([w, xyz[0], xyz[1], xyz[2]])


def quaternion_multiply(q1, q2):
    """
    Hamilton product of two quaternions.
    q1, q2: [w, x, y, z]
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

    return np.array([w, x, y, z])


def read_contact_npz(mocap_contact_path):
    d = np.load(mocap_contact_path)
    mocap_length = len(d['leftHeel'])

    left_foot_step = np.zeros(mocap_length)
    right_foot_step = np.zeros(mocap_length)
    for i in range(mocap_length):
        if d['leftHeel'][i] or d['leftToe'][i]:
            left_foot_step[i] = 1
        if d['rightHeel'][i] or d['rightToe'][i]:
            right_foot_step[i] = 1

    return left_foot_step, right_foot_step


def terrain_height_ray(model, data, x, y, max_z=100.0, geomgroup=np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)):
    # 起点 (x, y, z)，向下射线
    pnt = np.array([x, y, max_z], dtype=np.float64)
    vec = np.array([0.0, 0.0, -1.0], dtype=np.float64)  # 向下射线
    geomid = np.array([-1], dtype=np.int32)
    cdm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'cdm')
    distance = mujoco.mj_ray(model, data, pnt, vec, geomgroup=geomgroup, flg_static=1, bodyexclude=cdm_id, geomid=geomid)

    if geomid[0] == -1:
        return None, geomid[0]  # 没撞到任何 geom
    else:
        return max_z - distance, geomid[0]

# def terrain_height(model, x, y, hf_id=0):
#     hf_geom_pos = model.geom_pos[hf_id]
#     hf_nrow = model.hfield_nrow[hf_id]
#     hf_ncol = model.hfield_ncol[hf_id]
#     size_x, size_y, size_z, _ = model.hfield_size[hf_id]
#     hf_offset = sum(model.hfield_nrow[i] * model.hfield_ncol[i] for i in range(hf_id))
#
#     # ✅ 把 (x, y) 映射到以地形中心为原点的坐标系
#     local_x = x - hf_geom_pos[0]
#     local_y = y - hf_geom_pos[1]
#
#     # ✅ 映射到灰度图索引坐标
#     u = (local_x + size_x) / (2 * size_x) * (hf_ncol - 1)
#     v = (local_y + size_y) / (2 * size_y) * (hf_nrow - 1)
#
#     # ✅ 边界外采样则直接返回 0（防止误读）
#     if u < 0 or u > hf_ncol - 1 or v < 0 or v > hf_nrow - 1:
#         return 0.0
#
#     u0 = int(np.floor(u))
#     u1 = int(np.clip(u0 + 1, 0, hf_ncol - 1))
#     v0 = int(np.floor(v))
#     v1 = int(np.clip(v0 + 1, 0, hf_nrow - 1))
#
#     def get_z(ui, vi):
#         idx = vi * hf_ncol + ui
#         return model.hfield_data[hf_offset + idx] * size_z
#
#     z00 = get_z(u0, v0)
#     z01 = get_z(u0, v1)
#     z10 = get_z(u1, v0)
#     z11 = get_z(u1, v1)
#
#     s = u - u0
#     t = v - v0
#
#     z = (1 - s) * (1 - t) * z00 + \
#         (1 - s) * t * z01 + \
#         s * (1 - t) * z10 + \
#         s * t * z11
#
#     # ✅ 最终加上地形的 z 偏移量（地形中心）
#     return z + hf_geom_pos[2]


class SRBdata(object):
    def __init__(self, mocap_path):
        self.character_path = "gym_trackSRB/Characters/SRB5_playground.xml"
        self.mocap_path = mocap_path
        self.mocap_contact_pair = {
            'motiondata_mujoco_refined/walk1_subject5.txt': 'ref_contact/walk1_subject5.bvh.constraint.npz',
            'motiondata_mujoco_refined/run1_subject5.txt': 'ref_contact/run1_subject5.bvh.constraint.npz',
            'motiondata_mujoco_refined/jumps1_subject1.txt': 'ref_contact/jumps1_subject1.bvh.constraint.npz',
            'motiondata_mujoco_refined/jumps1_subject5.txt': 'ref_contact/jumps1_subject5.bvh.constraint.npz',
            'motiondata_mujoco_refined/sprint1_subject2.txt': 'ref_contact/sprint1_subject2.bvh.constraint.npz',
            'motiondata_mujoco_refined_mirrored/walk1_subject5_mirrored.txt': 'ref_contact_mirrored/walk1_subject5_mirrored.bvh.constraint.npz',
            'motiondata_mujoco_refined_mirrored/run1_subject5_mirrored.txt': 'ref_contact_mirrored/run1_subject5_mirrored.bvh.constraint.npz',
            'motiondata_mujoco_refined_mirrored/jumps1_subject1_mirrored.txt': 'ref_contact_mirrored/jumps1_subject1_mirrored.bvh.constraint.npz',
            'motiondata_mujoco_refined_mirrored/jumps1_subject5_mirrored.txt': 'ref_contact_mirrored/jumps1_subject5_mirrored.bvh.constraint.npz',
            'motiondata_mujoco_refined_mirrored/sprint1_subject2_mirrored.txt': 'ref_contact_mirrored/sprint1_subject2_mirrored.bvh.constraint.npz',
            'motiondata_mujoco_refined/stand1.txt': 'ref_contact/stand1.bvh.constraint.npz',
            'motiondata_mujoco_refined/aiming1_subject1.txt': 'ref_contact/aiming1_subject1.bvh.constraint.npz',
            'motiondata_mujoco_refined/aiming_show.txt': 'ref_contact/aiming_show.bvh.constraint.npz',
            'motiondata_mujoco_refined/basketball_show.txt': 'ref_contact/basketball_show.bvh.constraint.npz',
            'motiondata_mujoco_refined/volleyball_show.txt': 'ref_contact/volleyball_show.bvh.constraint.npz',
            'motiondata_mujoco_refined/dance_show.txt': 'ref_contact/dance_show.bvh.constraint.npz',
            'motiondata_mujoco_refined/fight_show.txt': 'ref_contact/fight_show.bvh.constraint.npz',
        }
        if self.mocap_path in self.mocap_contact_pair:
            self.mocap_contact_path = 'gym_trackSRB/'+self.mocap_contact_pair[self.mocap_path]
        else:
            self.mocap_contact_path = 'gym_trackSRB/'+self.mocap_path[:-10]+'constraint.npz'
        self.character_path_humanoid = "gym_trackSRB/Characters/humanoid_deepmimic_withpalm_Tpose.xml"
        self.dt = 0.033333
        mocap_path='gym_trackSRB/'+mocap_path
        #self.dt = 0.05
        if mocap_path[-3:]=='npz':
            data=np.load(mocap_path)
            # 如果有 'motion' 这个 key
            if 'motion' in data:
                self.mocap_data = data['motion']
            elif 'arr_0' in data:
                self.mocap_data = data['arr_0']
            else:
                raise KeyError("Neither 'motion' nor 'arr_0' found in file.")
        else:
            with open(mocap_path, 'r') as f:
                mocap_data = f.read()
                self.mocap_data = np.array(eval(mocap_data))
        initial_pose = self.mocap_data[0, :2]
        self.mocap_data[:, :2] -= initial_pose
        self.model = mujoco.MjModel.from_xml_path(self.character_path_humanoid)
        self.data = mujoco.MjData(self.model)

    def get_frame_data(self, frame_idx, prev_cycle_root=None):
        """
        获取指定帧的姿态数据，并处理循环过渡
        Gets attitude data for the specified frame and handles loop transitions
        """
        # （1）计算当前在 [initial_frame, last_frame] 范围内的帧索引
        #  Computes the index of the frame currently in the [initial_frame, last_frame] range
        cycle_frame = self.initial_frame + (frame_idx - self.initial_frame) % self.cycle_length

        # （2）从 mocap_data 中取该帧的绝对姿态
        #  Get the real state of the frame from mocap_data
        current_pose = self.mocap_data[cycle_frame].copy()

        # 如果有上一周期末尾的根节点信息，就叠加偏差
        # If the root node information at the end of the previous period is available, the deviation is superimposed
        if prev_cycle_root is not None:
            prev_pos = prev_cycle_root[:3]
            projected_prev_pos = prev_pos.copy()
            projected_prev_pos[2] = 0
            prev_quat = prev_cycle_root[3:7]
            prev_mat = transformations.quaternion_matrix(prev_quat)[:3, :3]

            # 计算当前帧相对于循环初始帧的“局部”差异
            # Calculates the "local" difference between the current frame and the initial frame of the loop
            start_pos = self.mocap_data[self.initial_frame][:3]
            projected_start_pos = start_pos.copy()
            projected_start_pos[2] = 0
            start_quat = self.mocap_data[self.initial_frame][3:7]
            start_mat = transformations.quaternion_matrix(start_quat)[:3, :3]

            # get global pos
            local_pos = project_to_forward_facing(start_mat).T @ (current_pose[:3] - projected_start_pos)
            final_pos = project_to_forward_facing(prev_mat) @ local_pos + projected_prev_pos

            # get global quat
            current_mocap_quat = current_pose[3:7]
            current_mocap_mat = transformations.quaternion_matrix(current_mocap_quat)[:3, :3]
            local_mat = project_to_forward_facing(start_mat).T @ current_mocap_mat
            global_mat = project_to_forward_facing(prev_mat) @ local_mat

            final_mat = global_mat
            final_transformation_mat = np.eye(4)
            final_transformation_mat[:3, :3] = final_mat
            final_quat = transformations.quaternion_from_matrix(final_transformation_mat)

            # 将叠加结果写回 current_pose
            # Write the result back to current_pose
            current_pose[:3] = final_pos
            current_pose[3:7] = final_quat
        return current_pose

    def humanoid2SRB(self, initial_frame=0, last_frame=None):
        mocap_pos_humanoid = self.mocap_data
        self.initial_frame = initial_frame

        frame_idx = initial_frame  # 总帧数计数器/ Total frame counter
        if last_frame is None:
            last_frame = len(mocap_pos_humanoid)-1
            self.last_frame = len(mocap_pos_humanoid)-1
        self.last_frame = last_frame
        self.cycle_length = last_frame - initial_frame + 1
        prev_cycle_root = None  # 存储上一循环结束时的根节点变换/ Stores the root transform at the end of the previous loop

        # Simulation parameters
        root_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "root")
        chest_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "chest")
        left_toe_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "left_toe")
        left_heel_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "left_heel")
        right_toe_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "right_toe")
        right_heel_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "right_heel")
        # Get the SRB state
        self.feet_pos_list = []
        self.toe_pos_list = []
        self.heel_pos_list = []
        self.feet_ori_list = []
        self.mocap_pos_com = []
        self.mocap_pos_humanoid = []
        self.mocap_vel_com = []
        self.mocap_ori_com = []
        self.mocap_lin_vel_com = []
        self.mocap_ang_vel_com = []
        # self.mocap_all_feet_site_pos = []
        i = 0
        while True:
            current_pose = self.get_frame_data(frame_idx, prev_cycle_root)
            self.mocap_pos_humanoid.append(current_pose)
            self.data.qpos = current_pose
            mujoco.mj_forward(self.model, self.data)
            temp_mocap_pos_com = self.data.subtree_com[0].copy()
            temp_mocap_ori_com_root = self.data.xmat[root_id].copy().reshape(3, 3)  # use root ori as cdm ori
            temp_mocap_ori_com_chest = self.data.xmat[chest_id].copy().reshape(3, 3)  # use chest ori as cdm ori
            temp_mocap_ori_com_average = average_rotation_matrix(temp_mocap_ori_com_root, temp_mocap_ori_com_chest)
            temp_mocap_ori_com = temp_mocap_ori_com_root
            # temp_mocap_quat_com = rotation_matrix_to_quaternion(temp_mocap_ori_com_average)
            # self.mocap_pos_humanoid[i][3:7] = temp_mocap_quat_com
            temp_feet_pos, temp_feet_ori = get_feet_state(self.model, self.data)
            temp_left_toe_pos = self.data.site_xpos[left_toe_id]
            temp_left_heel_pos = self.data.site_xpos[left_heel_id]
            temp_right_toe_pos = self.data.site_xpos[right_toe_id]
            temp_right_heel_pos = self.data.site_xpos[right_heel_id]
            temp_toe_pos = np.array([temp_left_toe_pos, temp_right_toe_pos])
            temp_heel_pos = np.array([temp_left_heel_pos, temp_right_heel_pos])
            if i == 0:
                temp_mocap_lin_vel_com = np.array([np.nan, np.nan, np.nan])
                temp_mocap_ang_vel_com = np.array([np.nan, np.nan, np.nan])
            else:
                temp_mocap_qvel = np.zeros(self.model.nv)
                mujoco.mj_differentiatePos(self.model, temp_mocap_qvel, self.dt, self.mocap_pos_humanoid[i-1], self.mocap_pos_humanoid[i])
                temp_mocap_ang_vel_com = temp_mocap_qvel[3:6]
                temp_mocap_lin_vel_com = (temp_mocap_pos_com - self.mocap_pos_com[i-1]) / self.dt

            self.toe_pos_list.append(temp_toe_pos)
            self.heel_pos_list.append(temp_heel_pos)
            self.feet_pos_list.append(temp_feet_pos)
            self.feet_ori_list.append(temp_feet_ori)
            self.mocap_pos_com.append(temp_mocap_pos_com)
            self.mocap_lin_vel_com.append(temp_mocap_lin_vel_com)
            self.mocap_ang_vel_com.append(temp_mocap_ang_vel_com)
            self.mocap_ori_com.append(temp_mocap_ori_com)  # com rotation matirx = root rotation matrix
            # 在每个循环结束时，存储根节点变换用于下一个循环/ At the end of each loop, the root node transform is stored for the next loop
            if (frame_idx - self.initial_frame) % self.cycle_length == self.cycle_length - 1:
                prev_cycle_root = current_pose.copy()
            frame_idx += 1
            i += 1
            if i >= self.cycle_length*2-1:
                break
        self.mocap_pos_humanoid = np.array(self.mocap_pos_humanoid)
        mocap_ori_com_humanoid = self.mocap_pos_humanoid[:, 3:7]  # com quaternion = root quaternion
        self.mocap_com_humanoid = np.hstack((np.array(self.mocap_pos_com), mocap_ori_com_humanoid))  # com state(com pos and root quat)
        self.mocap_lin_vel_com[0] = self.mocap_lin_vel_com[1].copy()
        self.mocap_ang_vel_com[0] = self.mocap_ang_vel_com[1].copy()
        self.mocap_vel_com = np.hstack((self.mocap_lin_vel_com, self.mocap_ang_vel_com))  # com vel(com lin vel and ang vel)
        self.feet_pos_list = np.array(self.feet_pos_list)
        self.toe_pos_list = np.array(self.toe_pos_list)
        self.heel_pos_list = np.array(self.heel_pos_list)

        # 提取左脚和右脚的z坐标; get the z value of feet pos
        initial_foot_z = min(self.feet_pos_list[0, 0, 2], self.feet_pos_list[0, 1, 2])
        left_foot_z = self.feet_pos_list[:, 0, 2].copy()-initial_foot_z
        right_foot_z = self.feet_pos_list[:, 1, 2].copy()-initial_foot_z

        initial_toe_z = min(self.toe_pos_list[0, 0, 2], self.toe_pos_list[0, 1, 2])
        left_toe_z = self.toe_pos_list[:, 0, 2].copy() - initial_toe_z
        right_toe_z = self.toe_pos_list[:, 1, 2].copy() - initial_toe_z

        initial_heel_z = min(self.heel_pos_list[0, 0, 2], self.heel_pos_list[0, 1, 2])
        left_heel_z = self.heel_pos_list[:, 0, 2].copy() - initial_heel_z
        right_heel_z = self.heel_pos_list[:, 1, 2].copy() - initial_heel_z

        left_foot_xy = self.feet_pos_list[:, 0, :2]
        right_foot_xy = self.feet_pos_list[:, 1, :2]
        left_foot_vel_xy = np.linalg.norm(left_foot_xy[1:, :]-left_foot_xy[:-1, :], axis=-1)/self.dt
        right_foot_vel_xy = np.linalg.norm(right_foot_xy[1:, :]-right_foot_xy[:-1, :], axis=-1)/self.dt
        self.left_foot_vel_xy = np.concatenate([left_foot_vel_xy[0:1], left_foot_vel_xy])
        self.right_foot_vel_xy = np.concatenate([right_foot_vel_xy[0:1], right_foot_vel_xy])

        # 设定一个阈值来区分脚踏在地上和悬在空中（这里假设阈值为0.04）; if z<threshold, it will be considered to be on the floor
        left_threshold = 0.04  # 0.04
        right_threshold = 0.04  # 0.04
        # 创建列表表示脚的状态，踏在地上为1，悬在空中为0; feet on th floor then 1, feet in the air then 0
        left_foot_step_1 = np.zeros(left_foot_z.shape[0])
        right_foot_step_1 = np.zeros(right_foot_z.shape[0])
        for i in range(left_foot_z.shape[0]):
            if (left_toe_z[i] < left_threshold or left_heel_z[i] < left_threshold) and self.left_foot_vel_xy[i] < 0.7:
                left_foot_step_1[i] = 1
                if i >= 1 and left_foot_step_1[i-1] == 0:
                    left_foot_step_1[i-1] = 1
            if (right_toe_z[i] < right_threshold or right_heel_z[i] < right_threshold) and self.right_foot_vel_xy[i] < 0.7:
                right_foot_step_1[i] = 1
                if i >= 1 and right_foot_step_1[i-1] == 0:
                    right_foot_step_1[i-1] = 1

        left_foot_step_2, right_foot_step_2 = read_contact_npz(self.mocap_contact_path)
        left_foot_step_2 = left_foot_step_2[self.initial_frame: self.last_frame].copy()
        right_foot_step_2 = right_foot_step_2[self.initial_frame: self.last_frame].copy()
        while True:
            left_foot_step_2 = np.concatenate([left_foot_step_2, left_foot_step_2], axis=0)
            right_foot_step_2 = np.concatenate([right_foot_step_2, right_foot_step_2], axis=0)
            if left_foot_step_2.shape[0] >= self.mocap_pos_humanoid.shape[0]:
                left_foot_step_2 = left_foot_step_2[:self.mocap_pos_humanoid.shape[0]]
                right_foot_step_2 = right_foot_step_2[:self.mocap_pos_humanoid.shape[0]]
                break
        self.left_foot_step = np.where(left_foot_step_1==left_foot_step_2, left_foot_step_1, 0)
        self.right_foot_step = np.where(right_foot_step_1==right_foot_step_2, right_foot_step_1, 0)
        self.left_foot_step = left_foot_step_2
        self.right_foot_step = right_foot_step_2
        # self.left_foot_step = left_foot_step_1
        # self.right_foot_step = right_foot_step_1
        self.feet_pos_list[:, 0, 2] = self.feet_pos_list[:, 0, 2] * (1 - self.left_foot_step)
        self.feet_pos_list[:, 1, 2] = self.feet_pos_list[:, 1, 2] * (1 - self.right_foot_step)

        # print(f'feet_post_list: {self.feet_pos_list}')
        # get the transformation matrix of mocap data at each timestep
        model = mujoco.MjModel.from_xml_path(self.character_path)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        terrain_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_HFIELD, 'hf_hill')
        cdm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'cdm')

        self.mocap_steps_num = len(self.mocap_com_humanoid)
        self.feet_pos_list_original = self.feet_pos_list.copy()
        self.feet_pos_list[:, :, 2] = 0
        self.mocap_height_com_terrain = []
        for i in range(self.mocap_steps_num):
            com_pos_xy = self.mocap_pos_com[i][:2]
            left_foot_pos_xy = self.feet_pos_list[i, 0, :2]
            right_foot_pos_xy = self.feet_pos_list[i, 1, :2]
            terrain_height_com, _ = terrain_height_ray(model, data, com_pos_xy[0], com_pos_xy[1])
            terrain_height_left_foot, _ = terrain_height_ray(model, data, left_foot_pos_xy[0], left_foot_pos_xy[1])
            terrain_height_right_foot, _ = terrain_height_ray(model, data, right_foot_pos_xy[0], right_foot_pos_xy[1])
            # terrain_height_com = terrain_height(model, com_pos_xy[0], com_pos_xy[1], terrain_id)
            # terrain_height_left_foot = terrain_height(model, left_foot_pos_xy[0], left_foot_pos_xy[1], terrain_id)
            # terrain_height_right_foot = terrain_height(model, right_foot_pos_xy[0], right_foot_pos_xy[1], terrain_id)
            self.mocap_pos_com[i][2] += terrain_height_com
            self.mocap_com_humanoid[i][2] += terrain_height_com
            self.feet_pos_list[i, 0, 2] += terrain_height_left_foot
            self.feet_pos_list[i, 1, 2] += terrain_height_right_foot
            self.mocap_height_com_terrain.append(terrain_height_com)
        del model, data, self.model, self.data

    def play(self, character_path, initial_frame=0, last_frame=None):
        self.humanoid2SRB(initial_frame=initial_frame, last_frame=last_frame)
        # visual setting options in mujoco
        opt = mujoco.MjvOption()
        opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = True

        #  load SRB character
        model = mujoco.MjModel.from_xml_path(character_path)
        data = mujoco.MjData(model)
        friction_coef = model.geom_friction[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'floor')]
        self.mu = friction_coef[0]

        # data.qpos = mocap_pos_cdm[0]
        # data.qvel = mocap_vel_cdm[0]


        #  simulation viewer
        # i = self.last_frame-self.initial_frame-50
        # i = self.cycle_length-100
        i = 0
        data.qpos = self.mocap_com_humanoid[i]
        print(data.qpos[:2])
        data.qvel = self.mocap_vel_com[i]
        mujoco.mj_forward(model, data)
        with (mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False, key_callback=key_callback) as viewer):
            visual_set(viewer, data.qpos[:3]+np.array([0, 0, 0.5]))
            largest_angle_dgrees = 0
            largest_speed = 0
            lowest_height = 10
            while i <= self.mocap_steps_num-2:
                if not paused:
                    curr_feet_pos = self.feet_pos_list[i]
                    curr_feet_ori = self.feet_ori_list[i]
                    curr_T_mocap = transformation_matrix(self.mocap_ori_com[i+1], self.mocap_pos_com[i+1])
                    # target_pos = mocap_pos_cdm[i]
                    target_pos = self.mocap_com_humanoid[i+1]
                    # target_vel = mocap_vel_cdm[i]
                    target_vel = self.mocap_vel_com[i+1]
                    print(i+self.initial_frame)
                    srb_ori = self.mocap_ori_com[i]
                    srb_vertical_axis = srb_ori[:, 2]
                    global_z_axis = np.array([0, 0, 1])  # 全局 z 轴
                    cos_angle = np.dot(srb_vertical_axis, global_z_axis) / (
                            np.linalg.norm(srb_vertical_axis) * np.linalg.norm(global_z_axis))
                    angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))  # 确保值在 -1 到 1 之间，防止浮点数误差
                    angle_degrees = np.degrees(angle)
                    speed = np.linalg.norm(data.qvel[:2].copy())
                    if speed > largest_speed:
                        largest_speed = speed
                    print(largest_speed)
                    if angle_degrees>largest_angle_dgrees:
                        largest_angle_dgrees = angle_degrees
                    if data.qpos[2]<lowest_height:
                        lowest_height = data.qpos[2].copy()
                    print(lowest_height)
                    print(largest_angle_dgrees)
                    print(f'height error: {self.mocap_pos_com[i][2]-data.qpos[2]}')
                    # terrain_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_HFIELD, 'hf_hill')
                    print(f'terrain height ray {terrain_height_ray(model, data, data.qpos[0], data.qpos[1])}')
                    # print(f'terrain height hfield {terrain_height(model, data.qpos[0], data.qpos[1], terrain_id)}')
                    print('----------------')
                    for _ in range(round(self.dt/model.opt.timestep)):

                        mujoco.mj_forward(model, data)
                        data.qfrc_applied.fill(0)
                        viewer.user_scn.ngeom = 0  # 每次渲染前清除所有之前的几何体标记; clear the previous geoms created at last step

                        # 该时刻的site id，在地面上的为真实id，不在地面上的为-1; site id at this moment, on the floor then true id, or -1
                        set_feet_site_this_step(model, data, self.feet_pos_list_original[i], self.feet_ori_list[i], self.feet_pos_list[i, 0, :], self.feet_pos_list[i, 1, :])
                        site_to_apply_force_on = return_site_to_apply_force_on(model, self.feet_pos_list_original[i])

                        if all(site == -1 for site in site_to_apply_force_on):  # if no foot is on the floor, pass
                            pass
                        else:
                            _, _, contact_forces_list = solve_qp_for_contact_forces(model, data, curr_T_mocap,
                                                                                    target_vel, site_to_apply_force_on, self.mu)
                            j = 0
                            for site_id in site_to_apply_force_on:
                                if site_id == -1:
                                    continue
                                else:
                                    apply_force_at_site(model, data, site_id, contact_forces_list[j])  # apply contact forces
                                    add_force_vector(viewer, data.site_xpos[site_id], contact_forces_list[j])  # visualize forces
                                    j += 1
                    data.qpos = target_pos
                    data.qvel = self.mocap_vel_com[i+1]
                    print(data.qvel)
                    viewer.sync()
                    time.sleep(0.01)
                    i += 1
        viewer.close()


if __name__ == "__main__":
    paused = True


    def key_callback(keycode):
        global paused
        if chr(keycode) == ' ':
            paused = not paused
    mocap_path = "motiondata_mujoco_refined/walk1_subject5.txt"
    test = SRBdata(mocap_path)
    model_path = "Characters/SRB5_playground.xml"
    test.play(model_path, 0, None)
