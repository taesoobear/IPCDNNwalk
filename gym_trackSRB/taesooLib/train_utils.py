import numpy as np
import torch
import torch.nn.functional as F


def get_mocap_data_mujoco(skill_intention):
    mocap_path_fallAndGetUp = "motiondata_mujoco_refined/fallAndGetUp1_subject1.txt"
    mocap_path_walk = "motiondata_mujoco_refined/walk1_subject5.txt"
    mocap_path_run = "motiondata_mujoco_refined/run1_subject5.txt"
    mocap_path_jump = "motiondata_mujoco_refined/jumps1_subject1.txt"

    # use tuple as skill intention
    mocap_dict = {}
    mocap_dict[(1, 0, 0)] = mocap_path_walk
    mocap_dict[(0, 1, 0)] = mocap_path_run
    mocap_dict[(0, 0, 1)] = mocap_path_jump
    mocap_dict[(1, 1, 1)] = mocap_path_fallAndGetUp

    # reference mocap data
    skill_intention_tuple = tuple(skill_intention)
    mocap_path = mocap_dict[skill_intention_tuple]
    with open(mocap_path, 'r') as f:
        mocap_data = f.read()
    mocap_data = np.array(eval(mocap_data))

    return mocap_data


def get_mocap_data_mujoco_mirrored(skill_intention):
    mocap_path_fallAndGetUp = "motiondata_mujoco_refined_mirrored/fallAndGetUp1_subject1_mirrored.txt"
    mocap_path_walk = "motiondata_mujoco_refined_mirrored/walk1_subject5_mirrored.txt"
    mocap_path_run = "motiondata_mujoco_refined_mirrored/run1_subject5_mirrored.txt"
    mocap_path_jump = "motiondata_mujoco_refined_mirrored/jumps1_subject1_mirrored.txt"

    # use tuple as skill intention
    mocap_dict = {}
    mocap_dict[(1, 0, 0)] = mocap_path_walk
    mocap_dict[(0, 1, 0)] = mocap_path_run
    mocap_dict[(0, 0, 1)] = mocap_path_jump
    mocap_dict[(1, 1, 1)] = mocap_path_fallAndGetUp

    # reference mocap data
    skill_intention_tuple = tuple(skill_intention)
    mocap_path = mocap_dict[skill_intention_tuple]
    with open(mocap_path, 'r') as f:
        mocap_data = f.read()
    mocap_data = np.array(eval(mocap_data))

    return mocap_data


def get_reference_state(skill_intention):
    mocap_path_fallAndGetUp = "reference_state_refined/fallAndGetUp1_subject1_state.txt"
    mocap_path_walk = "reference_state_refined/walk1_subject5_state.txt"
    mocap_path_run = "reference_state_refined/run1_subject5_state.txt"
    mocap_path_jump = "reference_state_refined/jumps1_subject1_state.txt"

    # use tuple as skill intention
    mocap_dict = {}
    mocap_dict[(1, 0, 0)] = mocap_path_walk
    mocap_dict[(0, 1, 0)] = mocap_path_run
    mocap_dict[(0, 0, 1)] = mocap_path_jump
    mocap_dict[(1, 1, 1)] = mocap_path_fallAndGetUp

    # reference mocap data
    skill_intention_tuple = tuple(skill_intention)
    mocap_path = mocap_dict[skill_intention_tuple]
    with open(mocap_path, 'r') as f:
        mocap_data = f.read()
    mocap_data = np.array(eval(mocap_data))

    return mocap_data


def get_reference_state_mirrored(skill_intention):
    mocap_path_fallAndGetUp = "reference_state_refined_mirrored/fallAndGetUp1_subject1_state_mirrored.txt"
    mocap_path_walk = "reference_state_refined_mirrored/walk1_subject5_state_mirrored.txt"
    mocap_path_run = "reference_state_refined_mirrored/run1_subject5_state_mirrored.txt"
    mocap_path_jump = "reference_state_refined_mirrored/jumps1_subject1_state_mirrored.txt"

    # use tuple as skill intention
    mocap_dict = {}
    mocap_dict[(1, 0, 0)] = mocap_path_walk
    mocap_dict[(0, 1, 0)] = mocap_path_run
    mocap_dict[(0, 0, 1)] = mocap_path_jump
    mocap_dict[(1, 1, 1)] = mocap_path_fallAndGetUp

    # reference mocap data
    skill_intention_tuple = tuple(skill_intention)
    mocap_path = mocap_dict[skill_intention_tuple]
    with open(mocap_path, 'r') as f:
        mocap_data = f.read()
    mocap_data = np.array(eval(mocap_data))

    return mocap_data


def get_reference_global_state(skill_intention):
    mocap_path_fallAndGetUp = "reference_global_state/fallAndGetUp1_subject1_global_state.txt"
    mocap_path_walk = "reference_global_state/walk1_subject5_global_state.txt"
    mocap_path_run = "reference_global_state/run1_subject5_global_state.txt"
    mocap_path_jump = "reference_global_state/jumps1_subject1_global_state.txt"

    # use tuple as skill intention
    mocap_dict = {}
    mocap_dict[(1, 0, 0)] = mocap_path_walk
    mocap_dict[(0, 1, 0)] = mocap_path_run
    mocap_dict[(0, 0, 1)] = mocap_path_jump
    mocap_dict[(1, 1, 1)] = mocap_path_fallAndGetUp

    # reference mocap data
    skill_intention_tuple = tuple(skill_intention)
    mocap_path = mocap_dict[skill_intention_tuple]
    with open(mocap_path, 'r') as f:
        mocap_data = f.read()
    mocap_data = np.array(eval(mocap_data))

    return mocap_data


def get_reference_global_state_mirrored(skill_intention):
    mocap_path_fallAndGetUp = "reference_global_state_mirrored/fallAndGetUp1_subject1_global_state_mirrored.txt"
    mocap_path_walk = "reference_global_state_mirrored/walk1_subject5_global_state_mirrored.txt"
    mocap_path_run = "reference_global_state_mirrored/run1_subject5_global_state_mirrored.txt"
    mocap_path_jump = "reference_global_state_mirrored/jumps1_subject1_global_state_mirrored.txt"

    # use tuple as skill intention
    mocap_dict = {}
    mocap_dict[(1, 0, 0)] = mocap_path_walk
    mocap_dict[(0, 1, 0)] = mocap_path_run
    mocap_dict[(0, 0, 1)] = mocap_path_jump
    mocap_dict[(1, 1, 1)] = mocap_path_fallAndGetUp

    # reference mocap data
    skill_intention_tuple = tuple(skill_intention)
    mocap_path = mocap_dict[skill_intention_tuple]
    with open(mocap_path, 'r') as f:
        mocap_data = f.read()
    mocap_data = np.array(eval(mocap_data))

    return mocap_data


def get_mocap_qvel(skill_intention):
    mocap_path_fallAndGetUp = "reference_qvel_mujoco_refined/fallAndGetUp1_subject1_qvel.txt"
    mocap_path_walk = "reference_qvel_mujoco_refined/walk1_subject5_qvel.txt"
    mocap_path_run = "reference_qvel_mujoco_refined/run1_subject5_qvel.txt"
    mocap_path_jump = "reference_qvel_mujoco_refined/jumps1_subject1_qvel.txt"

    # use tuple as skill intention
    mocap_dict = {}
    mocap_dict[(1, 0, 0)] = mocap_path_walk
    mocap_dict[(0, 1, 0)] = mocap_path_run
    mocap_dict[(0, 0, 1)] = mocap_path_jump
    mocap_dict[(1, 1, 1)] = mocap_path_fallAndGetUp

    # reference mocap data
    skill_intention_tuple = tuple(skill_intention)
    mocap_path = mocap_dict[skill_intention_tuple]
    with open(mocap_path, 'r') as f:
        mocap_qvel = f.read()
    mocap_qvel = np.array(eval(mocap_qvel))

    return mocap_qvel


def get_mocap_qvel_mirrored(skill_intention):
    mocap_path_fallAndGetUp = "reference_qvel_mujoco_refined_mirrored/fallAndGetUp1_subject1_qvel_mirrored.txt"
    mocap_path_walk = "reference_qvel_mujoco_refined_mirrored/walk1_subject5_qvel_mirrored.txt"
    mocap_path_run = "reference_qvel_mujoco_refined_mirrored/run1_subject5_qvel_mirrored.txt"
    mocap_path_jump = "reference_qvel_mujoco_refined_mirrored/jumps1_subject1_qvel_mirrored.txt"

    # use tuple as skill intention
    mocap_dict = {}
    mocap_dict[(1, 0, 0)] = mocap_path_walk
    mocap_dict[(0, 1, 0)] = mocap_path_run
    mocap_dict[(0, 0, 1)] = mocap_path_jump
    mocap_dict[(1, 1, 1)] = mocap_path_fallAndGetUp

    # reference mocap data
    skill_intention_tuple = tuple(skill_intention)
    mocap_path = mocap_dict[skill_intention_tuple]
    with open(mocap_path, 'r') as f:
        mocap_qvel = f.read()
    mocap_qvel = np.array(eval(mocap_qvel))

    return mocap_qvel


def get_intention_goal_state(skill_intention):
    """
    根据 skill_intention 加载正确的 intention goal state 文件，并存储在字典中以便快速访问。
    :param skill_intention: 一个包含技能意图的列表或元组，用于选择正确的文件。
    :return: intention goal states 的字典，key 是 frame_id，value 是 goal state。
    """
    intention_goal_path_walk = "IntentionGoalState/walk1_subject5_intention_goal_state.txt"
    intention_goal_path_run = "IntentionGoalState/run1_subject5_intention_goal_state.txt"
    intention_goal_path_jump = "IntentionGoalState/jumps1_subject1_intention_goal_state.txt"
    intention_goal_path_fallAndGetUp = "IntentionGoalState/fallAndGetUp1_subject1_intention_goal_state.txt"

    # 用元组来映射技能意图到对应的文件路径
    intention_goal_dict = {}
    intention_goal_dict[(1, 0, 0)] = intention_goal_path_walk
    intention_goal_dict[(0, 1, 0)] = intention_goal_path_run
    intention_goal_dict[(0, 0, 1)] = intention_goal_path_jump
    intention_goal_dict[(1, 1, 1)] = intention_goal_path_fallAndGetUp

    # 根据 skill_intention 选择文件路径
    skill_intention_tuple = tuple(skill_intention)
    file_path = intention_goal_dict[skill_intention_tuple]

    # 读取文件内容
    with open(file_path, 'r') as f:
        intention_goal_data = f.read()

    # 解析文件内容为 numpy 数组
    intention_goal_data = np.array(eval(intention_goal_data))

    return intention_goal_data


def get_intention_goal_state_mirrored(skill_intention):
    """
    根据 skill_intention 加载正确的 intention goal state 文件，并存储在字典中以便快速访问。
    :param skill_intention: 一个包含技能意图的列表或元组，用于选择正确的文件。
    :return: intention goal states 的字典，key 是 frame_id，value 是 goal state。
    """
    intention_goal_path_walk = "IntentionGoalState_mirrored/walk1_subject5_intention_goal_state_mirrored.txt"
    intention_goal_path_run = "IntentionGoalState_mirrored/run1_subject5_intention_goal_state_mirrored.txt"
    intention_goal_path_jump = "IntentionGoalState_mirrored/jumps1_subject1_intention_goal_state_mirrored.txt"
    intention_goal_path_fallAndGetUp = "IntentionGoalState_mirrored/fallAndGetUp1_subject1_intention_goal_state_mirrored.txt"

    # 用元组来映射技能意图到对应的文件路径
    intention_goal_dict = {}
    intention_goal_dict[(1, 0, 0)] = intention_goal_path_walk
    intention_goal_dict[(0, 1, 0)] = intention_goal_path_run
    intention_goal_dict[(0, 0, 1)] = intention_goal_path_jump
    intention_goal_dict[(1, 1, 1)] = intention_goal_path_fallAndGetUp

    # 根据 skill_intention 选择文件路径
    skill_intention_tuple = tuple(skill_intention)
    file_path = intention_goal_dict[skill_intention_tuple]

    # 读取文件内容
    with open(file_path, 'r') as f:
        intention_goal_data = f.read()

    # 解析文件内容为 numpy 数组
    intention_goal_data = np.array(eval(intention_goal_data))

    return intention_goal_data


def get_normalized_obs(mocap_state):
    global device
    mocap_observation = mocap_state[96: -64]

    # 使用 NumPy 计算均值和标准差
    obs_mean = np.round(np.mean(mocap_observation, axis=0), 8)
    obs_std = np.round(np.std(mocap_observation, axis=0), 8)

    # 只对标准差为 0 的位置加上 eps
    eps = 1e-8
    obs_std[obs_std == 0.0] += eps  # 只修改 std 为 0 的位置

    # 转换为 PyTorch tensor
    obs_mean = torch.tensor(obs_mean, dtype=torch.float32, device=device)
    obs_std = torch.tensor(obs_std, dtype=torch.float32, device=device)

    statistics = {
        'obs_mean': obs_mean,
        'obs_std': obs_std
    }
    return statistics


def normalize(data, mean, std):
    return (data-mean)/std


def unnormalize(data, mean, std):
    return data*std+mean


def collect_trajectory(env, trajectory_buffer):
    """
    完成一个 batch 的 trajectory collection：
    - 填充 temporary buffer
    - temporary buffer 满后将其内容合并到 main buffer

    :param env: 仿真环境对象，负责模拟动作
    :param trajectory_buffer: TrajectoryBuffer 对象，管理 temporary buffer 和 main buffer
    """
    # Step 1: 填充 temporary buffer，直到其达到容量上限
    trajectory_buffer.fill_buffer(env)

    # Step 2: 当 temporary buffer 满了，进行合并操作
    if trajectory_buffer.buffer_full_signal:
        trajectory_buffer.merge_buffers()


# def collect_trajectory_parallel(envs, trajectory_buffer):
#     """
#     完成一个 batch 的 trajectory collection：
#     - 使用多个环境并行填充 temporary buffer
#     - 当 temporary buffer 满后将其内容合并到 main buffer
#
#     :param envs: 包含多个仿真环境对象的列表，用于并行模拟
#     :param trajectory_buffer: TrajectoryBuffer 对象，管理 temporary buffer 和 main buffer
#     """
#     # Step 1: 填充 temporary buffer，直到其达到容量上限
#     trajectory_buffer.fill_buffer_parallel(envs)
#
#     # Step 2: 当 temporary buffer 满了，进行合并操作
#     if trajectory_buffer.buffer_full_signal:
#         trajectory_buffer.merge_buffers()


# def update_6d_rotation(rotation_6d, angular_velocity, dt):
#     """
#     根据角速度使用欧拉法更新 6D 旋转。
#
#     :param rotation_6d: 当前的 6D 旋转表示 (num_bodies, 6)
#     :param angular_velocity: 角速度 (num_bodies, 3)
#     :param dt: 时间步长
#     :return: 更新后的 6D 旋转表示 (num_bodies, 6)
#     """
#     # 计算旋转矩阵增量
#     delta_rot_matrix = angular_velocity_to_rotation_matrix(angular_velocity, dt)
#
#     # 将 6D 旋转表示转为旋转矩阵
#     rot_matrix = rotation_6d_to_matrix(rotation_6d)
#
#     # 更新旋转矩阵
#     next_rot_matrix = torch.bmm(delta_rot_matrix.unsqueeze(0), rot_matrix.unsqueeze(0)).squeeze(0)
#
#     # 将旋转矩阵转回 6D 表示
#     next_rotation_6d = matrix_to_6d(next_rot_matrix)
#
#     return next_rotation_6d

#
# def angular_velocity_to_rotation_matrix(angular_velocity, dt):
#     """
#     将角速度转换为旋转矩阵增量。
#
#     :param angular_velocity: 角速度 (3,)
#     :param dt: 时间步长
#     :return: 旋转矩阵增量 (3, 3)
#     """
#     theta = torch.norm(angular_velocity) * dt
#     if theta == 0:
#         return torch.eye(3)  # 没有角速度时返回单位矩阵
#
#     axis = angular_velocity / torch.norm(angular_velocity)
#
#     # 计算 Rodrigues 公式
#     K = torch.tensor([
#         [0, -axis[2], axis[1]],
#         [axis[2], 0, -axis[0]],
#         [-axis[1], axis[0], 0]
#     ])
#
#     delta_rot_matrix = torch.eye(3) + torch.sin(theta) * K + (1 - torch.cos(theta)) * torch.matmul(K, K)
#
#     return delta_rot_matrix
#
#
# def rotation_6d_to_matrix(rotation_6d):
#     """
#     将 6D 旋转表示转为旋转矩阵。
#
#     :param rotation_6d: 6D 旋转表示 (6,)
#     :return: 旋转矩阵 (3, 3)
#     """
#     col1 = rotation_6d[:3]
#     col2 = rotation_6d[3:]
#
#     # 计算第三列为前两列的叉积
#     col3 = torch.cross(col1, col2)
#
#     # 拼接成旋转矩阵
#     rot_matrix = torch.stack([col1, col2, col3], dim=1)
#
#     return rot_matrix
#
#
# def rotation_6d_to_matrix_batch(rotation_6d):
#     """
#     将批处理的 6D 旋转表示转为旋转矩阵 (批处理版本)。
#
#     :param rotation_6d: 批处理 6D 旋转表示，形状 (batch_size, 6)
#     :return: 批处理旋转矩阵，形状 (batch_size, 3, 3)
#     """
#     col1 = rotation_6d[:, :3]  # 提取前3个元素作为第一列，形状 (batch_size, 3)
#     col2 = rotation_6d[:, 3:]  # 提取后3个元素作为第二列，形状 (batch_size, 3)
#
#     # 计算第三列为前两列的叉积，形状 (batch_size, 3)
#     col3 = torch.cross(col1, col2, dim=1)
#
#     # 拼接成旋转矩阵，形状 (batch_size, 3, 3)
#     rot_matrix = torch.stack([col1, col2, col3], dim=1)
#
#     return rot_matrix


def matrix_to_6d(rot_matrix):
    """
    将旋转矩阵转为 6D 旋转表示。

    :param rot_matrix: 旋转矩阵 (batch_size, 3, 3)
    :return: 6D 旋转表示 (batch_size, 6)
    """
    col1 = rot_matrix[..., 0]
    col2 = rot_matrix[..., 1]

    rotation_6d = torch.cat([col1, col2], dim=-1)

    return rotation_6d


def set_goals(vector):
    goals = []
    n = len(vector)

    if n == 0:
        return goals

    i = 0
    while i < n:
        if vector[i] == 1:
            # 如果当前是1，检查是否接下来是0
            if i + 1 < n and vector[i + 1] == 0:
                goals.append(i + 1)  # 将第一个0设为goal

        elif vector[i] == 0:
            # 如果当前是0，检查是否有连续的0
            start = i
            while i < n and vector[i] == 0:
                i += 1
            # i到达的是第一个非0的位置，或者已经到达末尾
            if start > 0 or (start == 0 and i > 1):  # 确保不是第一位，但第二位的情况应考虑
                goals.append(i - 1)
            continue

        i += 1

    # 处理最后一位是1的情况
    if vector[-1] == 1:
        goals.append(n - 1)

    # 去重并排序
    return sorted(set(goals))


# def state2obs_projected(state):
#     """
#     将全局状态转换为局部观测，使用批处理方式进行所有计算，并返回扁平化的向量。
#     :param state: 全局状态，shape (batch_size, num_bodies, 15)包含每个刚体的全局位置、姿态、线速度、角速度。
#     :return: 局部观测 obs，shape (batch_size, num_bodies * 16 + 3)
#     """
#     batch_size, num_body, _ = state.shape
#
#     pos = state[..., 0:3]
#     rot = state[..., 3:9]
#     vel = state[..., 9:12]
#     avel = state[..., 12:15]
#
#     root_pos = pos[:, 0:1, :].view(-1, 1, 3)
#     projected_root_pos = root_pos
#     projected_root_pos[..., 2] = 0
#     root_rot_6D = rot[:, 0:1, :].view(-1, 1, 6)
#     root_rot_matrix = convert_6d_to_matrix(root_rot_6D)  # (batch_size, 1, 3, 3)
#     projected_root_rot_matrix = root_frame2projected_forwardfacing_frame(root_rot_matrix)  # (batch_size, 1, 3, 3)
#
#     # 计算局部坐标 (修正后)
#     projected_root_rot_matrix_inv = projected_root_rot_matrix.transpose(-1, -2)
#     local_pos = torch.matmul(projected_root_rot_matrix_inv, (pos - projected_root_pos).unsqueeze(-1)).squeeze(-1)  # (batch_size, num_bodies, 3)
#     local_vel = torch.matmul(projected_root_rot_matrix_inv, vel.unsqueeze(-1)).squeeze(-1)  # (batch_size, num_bodies, 3)
#     local_avel = torch.matmul(projected_root_rot_matrix_inv, avel.unsqueeze(-1)).squeeze(-1)  # (batch_size, num_bodies, 3)
#
#     # 处理旋转，将6D旋转转换为3x3矩阵，然后转换为相对旋转矩阵
#     rot_matrix = convert_6d_to_matrix(rot)  # (batch_size, num_bodies, 3, 3)
#     rel_rot_matrix = torch.matmul(projected_root_rot_matrix_inv, rot_matrix)  # (batch_size, num_body, 3, 3)
#     local_rot_6D = convert_matrix_to_6d(rel_rot_matrix)  # 转换回6D表示 (batch_size, num_body, 6)
#
#     # 提取全局高度（y 轴上的值）
#     global_height = pos[..., 2:3]  # (batch_size, num_bodies, 1)
#
#     # 根body up axis
#     root_up_axis = root_rot_matrix[:, 0, :, 2]  # z-up (batch_size, 3)
#
#     # 将每个属性的局部状态分别拼接
#     all_local_pos = local_pos.view(batch_size, -1)  # (batch_size, num_bodies * 3)
#     all_local_rot_6D = local_rot_6D.view(batch_size, -1)  # (batch_size, num_bodies * 6)
#     all_local_vel = local_vel.view(batch_size, -1)  # (batch_size, num_bodies * 3)
#     all_local_avel = local_avel.view(batch_size, -1)  # (batch_size, num_bodies * 3)
#     all_global_height = global_height.view(batch_size, -1)  # (batch_size, num_bodies * 1)
#
#     # 现在将这些属性拼接在一起
#     obs_per_body = torch.cat([all_local_pos, all_local_rot_6D, all_local_vel, all_local_avel, all_global_height], dim=-1)  # (batch_size, num_bodies * 16)
#
#     obs = torch.cat([obs_per_body, root_up_axis], dim=-1)  # (batch_size, num_bodies*16+3)
#
#     return obs

def rotation_matrix_x(theta):
    cos_theta = torch.cos(theta)
    sin_theta = torch.sin(theta)
    R_x = torch.stack([
        torch.stack([torch.ones_like(theta), torch.zeros_like(theta), torch.zeros_like(theta)], dim=-1),
        torch.stack([torch.zeros_like(theta), cos_theta, -sin_theta], dim=-1),
        torch.stack([torch.zeros_like(theta), sin_theta, cos_theta], dim=-1)
    ], dim=-2)
    return R_x


def rotation_matrix_y(theta):
    cos_theta = torch.cos(theta)
    sin_theta = torch.sin(theta)
    R_y = torch.stack([
        torch.stack([cos_theta, torch.zeros_like(theta), sin_theta], dim=-1),
        torch.stack([torch.zeros_like(theta), torch.ones_like(theta), torch.zeros_like(theta)], dim=-1),
        torch.stack([-sin_theta, torch.zeros_like(theta), cos_theta], dim=-1)
    ], dim=-2)
    return R_y


def rotation_matrix_z(theta):
    cos_theta = torch.cos(theta)
    sin_theta = torch.sin(theta)
    R_z = torch.stack([
        torch.stack([cos_theta, -sin_theta, torch.zeros_like(theta)], dim=-1),
        torch.stack([sin_theta, cos_theta, torch.zeros_like(theta)], dim=-1),
        torch.stack([torch.zeros_like(theta), torch.zeros_like(theta), torch.ones_like(theta)], dim=-1)
    ], dim=-2)
    return R_z


def euler_angle_action2rot_mat_action(action):
    """
    将euler_angle的action转化为旋转矩阵的6D表示
    :param action: (batch_size, action_size)
    :return: 6D表示的action, (batch_size, num_joints, 3, 3)
    """
    device = action.device
    batch_size, _ = action.shape

    actuators_dict = {
        "chest": ['x', 'y', 'z'],
        "neck": ['x', 'y', 'z'],
        "right_shoulder": ['x', 'y', 'z'],
        "right_elbow": ['z', 'y'],
        "right_wrist": ['x', 'z'],
        "left_shoulder": ['x', 'y', 'z'],
        "left_elbow": ['z', 'y'],
        "left_wrist": ['x', 'z'],
        "right_hip": ['x', 'y', 'z'],
        "right_knee": ['-y'],
        "right_ankle": ['x', 'y', 'z'],
        "left_hip": ['x', 'y', 'z'],
        "left_knee": ['-y'],
        "left_ankle": ['x', 'y', 'z']
    }
    num_joints = len(actuators_dict)
    # 初始化旋转矩阵为单位矩阵 (batch_size, num_joints, 3, 3)
    rot_matrices = torch.eye(3, device=device).repeat(batch_size, num_joints, 1, 1)

    # 将 actuators_dict 的键和值转换为列表，保证遍历顺序一致
    joint_axes_list = list(actuators_dict.values())

    # 按顺序处理每个关节的动作
    current_index = 0
    for i, axes in enumerate(joint_axes_list):
        num_axes = len(axes)
        joint_actions = action[:, current_index:current_index + num_axes]
        current_index += num_axes

        # 初始化该关节的旋转矩阵为单位矩阵
        R = torch.eye(3, device=device).unsqueeze(0).repeat(batch_size, 1, 1)

        # 根据固定的四种旋转轴配置生成旋转矩阵
        if axes == ['x', 'y', 'z']:
            theta_x, theta_y, theta_z = joint_actions[:, 0], joint_actions[:, 1], joint_actions[:, 2]
            R_x = rotation_matrix_x(theta_x)
            R_y = rotation_matrix_y(theta_y)
            R_z = rotation_matrix_z(theta_z)
            R = torch.matmul(torch.matmul(R_x, R_y), R_z)

        elif axes == ['z', 'y']:
            theta_z, theta_y = joint_actions[:, 0], joint_actions[:, 1]
            R_z = rotation_matrix_z(theta_z)
            R_y = rotation_matrix_y(theta_y)
            R = torch.matmul(R_z, R_y)

        elif axes == ['x', 'z']:
            theta_x, theta_z = joint_actions[:, 0], joint_actions[:, 1]
            R_x = rotation_matrix_x(theta_x)
            R_z = rotation_matrix_z(theta_z)
            R = torch.matmul(R_x, R_z)

        elif axes == ['-y']:
            theta_y = -joint_actions[:, 0]
            R = rotation_matrix_y(theta_y)

        # 将计算得到的旋转矩阵存入结果张量的第 i 个关节位置
        rot_matrices[:, i] = R

    return rot_matrices


def matrix_to_axis_angle(rotation_matrix, default_axis=torch.tensor([0., 0., 1.]), eps=1e-6):
    """
    将旋转矩阵转换为轴角表示
    Args:
        rotation_matrix: torch.Tensor, 旋转矩阵 (batch_size, 3, 3)
        default_axis: torch.Tensor, 默认旋转轴 (3,)
        eps: float, 数值稳定性阈值
    Returns:
        axis: torch.Tensor, 旋转轴 (batch_size, 3)
        theta: torch.Tensor, 旋转角度 (batch_size,)
    """
    # 验证输入
    assert rotation_matrix.dim() == 3 and rotation_matrix.size(1) == 3 and rotation_matrix.size(2) == 3, \
        f"Expected rotation matrix of shape (batch_size, 3, 3), got {rotation_matrix.shape}"

    batch_size = rotation_matrix.size(0)
    default_axis = default_axis.to(dtype=rotation_matrix.dtype, device=rotation_matrix.device).expand(batch_size, -1)

    # Step 1: 计算旋转角度
    # trace = rotation_matrix.diagonal(dim1=-2, dim2=-1).sum(-1)
    # theta = torch.acos(torch.clamp((trace - 1) / 2, -1.0, 1.0))
    trace = rotation_matrix[..., 0, 0] + rotation_matrix[..., 1, 1] + rotation_matrix[..., 2, 2]
    costheta = torch.clamp((trace - 1) / 2, -1.0, 1.0)
    theta = torch.acos(costheta)

    # Step 2: 创建掩码
    zero_angle_mask = theta < eps
    pi_angle_mask = torch.abs(theta - torch.pi) < eps
    normal_mask = ~(zero_angle_mask | pi_angle_mask)

    # Step 3: 计算旋转轴
    # 一般情况
    r21 = rotation_matrix[..., 2, 1] - rotation_matrix[..., 1, 2]
    r02 = rotation_matrix[..., 0, 2] - rotation_matrix[..., 2, 0]
    r10 = rotation_matrix[..., 1, 0] - rotation_matrix[..., 0, 1]
    axis_normal = torch.stack([r21, r02, r10], dim=-1)
    sin_theta = torch.sin(theta.unsqueeze(-1))
    axis_normal = torch.where(
        (sin_theta > eps) & normal_mask.unsqueeze(-1),
        axis_normal / (2 * sin_theta),
        torch.zeros_like(axis_normal)
    )

    # θ ≈ π 的情况
    diag = rotation_matrix.diagonal(dim1=-2, dim2=-1)  # (batch_size, 3)
    axis_pi = torch.sqrt(torch.clamp((diag + 1) / 2, min=0))
    # 使用非对角元素确定符号
    off_diag = torch.stack([
        rotation_matrix[..., 2, 1],  # r21
        rotation_matrix[..., 0, 2],  # r02
        rotation_matrix[..., 1, 0]  # r10
    ], dim=-1)
    axis_pi = axis_pi * torch.sign(off_diag)

    # 组合所有情况
    axis = torch.where(
        normal_mask.unsqueeze(-1), axis_normal,
        torch.where(pi_angle_mask.unsqueeze(-1), axis_pi, default_axis)
    )

    # Step 4: 归一化
    norm = torch.norm(axis, dim=-1, keepdim=True)
    axis = torch.where(norm > eps, axis / norm, default_axis)

    return axis, theta


def matrix_to_angular_velocity(R1, R2, dt, eps=1e-6):
    """
    通过两个相邻帧的旋转矩阵和时间间隔，计算相应的角速度。

    Args:
        R1 (torch.Tensor): 初始旋转矩阵，形状为 (batch_size, 3, 3)
        R2 (torch.Tensor): 下一个旋转矩阵，形状为 (batch_size, 3, 3)
        dt (torch.Tensor): 时间步长，形状为 (batch_size,)
        eps (float, optional): 数值稳定性阈值。默认为 1e-6
    Returns:
        omega (torch.Tensor): 角速度，形状为 (batch_size, 3)
    """
    dt = dt.view(-1, 1, 1)  # (batch_size, 1, 1)

    # Step 1: 计算旋转矩阵的导数 R_dot = (R2 - R1) / dt
    R_dot = (R2 - R1) / dt  # (batch_size, 3, 3)

    # Step 2: 计算角速度矩阵 Omega = (R_dot * R1^T - (R_dot * R1^T)^T) / 2
    Omega = (torch.matmul(R_dot, R1.transpose(-2, -1)) - torch.matmul(R_dot, R1.transpose(-2, -1)).transpose(-2, -1)) / 2  # (batch_size, 3, 3)

    # Step 3: 提取角速度向量 omega = [Omega_32, Omega_13, Omega_21]
    omega_x = Omega[:, 2, 1]
    omega_y = Omega[:, 0, 2]
    omega_z = Omega[:, 1, 0]

    omega = torch.stack([omega_x, omega_y, omega_z], dim=-1)  # (batch_size, 3)

    # Step 4: 处理旋转角度接近零的情况，避免数值不稳定
    omega_norm = torch.norm(omega, dim=1, keepdim=True)  # (batch_size, 1)
    omega = torch.where(
        omega_norm > eps,
        omega / omega_norm * omega_norm.clamp_min(eps),
        torch.zeros_like(omega)
    )

    return omega


# def get_rec_loss(curr_reconstructed_fullbody_obs, curr_ref_fullbody_obs, last_reconstructed_fullbody_obs, last_ref_fullbody_obs):
#     batch_size = curr_ref_fullbody_obs.shape[0]
#     weights = {
#         'pos': 0.2,
#         'rot': 0.1,
#         'delta_p': 0.5,
#         'delta_r': 0.5
#     }
#     curr_reconstructed_pos, curr_reconstructed_ori = decompose_reconstructed_obs(curr_reconstructed_fullbody_obs)
#     curr_ref_pos, curr_ref_ori, curr_ref_lvel, curr_ref_avel = decompose_obs(curr_ref_fullbody_obs)
#     # last_reconstructed_pos, last_reconstructed_ori = decompose_reconstructed_obs(last_reconstructed_fullbody_obs)
#     # last_ref_pos, last_ref_ori, last_ref_lvel, last_ref_avel = decompose_obs(last_ref_fullbody_obs)
#
#     # curr_reconstructed_ori_mat = mat6d2mat_vector_batch(curr_reconstructed_ori)
#     # last_reconstructed_ori_mat = mat6d2mat_vector_batch(last_reconstructed_ori)
#     # curr_ref_ori_mat = mat6d2mat_vector_batch(curr_ref_ori)
#     # last_ref_ori_mat = mat6d2mat_vector_batch(last_ref_ori)
#
#     # reconstructed_delta_r = torch.matmul(last_reconstructed_ori_mat.transpose(-1, -2), curr_reconstructed_ori_mat)
#     # reconstructed_delta_r_6d = mat3x3_to_6d_batch(reconstructed_delta_r).view(batch_size, -1)
#     # ref_delta_r = torch.matmul(last_ref_ori_mat.transpose(-1, -2), curr_ref_ori_mat)
#     # ref_delta_r_6d = mat3x3_to_6d_batch(ref_delta_r).view(batch_size, -1)
#
#     # Compute the L1 differences between next_observation and target_observation for each component
#     loss_pos = weights['pos'] * torch.norm(curr_reconstructed_pos - curr_ref_pos, p=1, dim=-1)
#     loss_rot = weights['rot'] * torch.norm(curr_reconstructed_ori - curr_ref_ori, p=1, dim=-1)
#     # loss_delta_p = weights['delta_p'] * torch.norm((curr_reconstructed_pos - last_reconstructed_pos)-(curr_ref_pos - last_ref_pos), p=1, dim=-1)
#     # loss_delta_r = weights['delta_r'] * torch.norm(reconstructed_delta_r_6d-ref_delta_r_6d, p=1, dim=-1)
#
#     # Sum the weighted L1 losses for all components
#     # loss = loss_pos + loss_rot + loss_delta_p + loss_delta_r
#     loss = loss_pos + loss_rot
#     # print(f'loss pos: {loss_pos}')
#     # print(f'loss rot: {loss_rot}')
#     # print(f'loss delta p: {loss_delta_p}')
#     # print(f'loss delta r: {loss_delta_r}')
#     # print('---------------------------')
#     return torch.mean(loss)

def get_rec_loss(curr_reconstructed_fullbody_obs, curr_ref_fullbody_obs):
    num_frames = curr_ref_fullbody_obs.shape[0]
    weights = {
        'pos': 0.2,
        'rot': 0.1,
    }

    loss1= torch.norm(curr_reconstructed_fullbody_obs[:, :-4*5] - curr_ref_fullbody_obs[:, :-4*5], p=1, dim=-1)
    #pred_con=(curr_reconstructed_fullbody_obs[:, -4:]>0.5).float()
    pred_con=curr_reconstructed_fullbody_obs[:, -4*5:]

    
    bce_loss=torch.nn.BCEWithLogitsLoss() # sigmoid + cross entropy
    #loss2=bce_loss(curr_ref_fullbody_obs[:, -4:]*4-2,curr_ref_fullbody_obs[:, -4:])
    #loss2=torch.sum(bce_loss(pred_con, curr_ref_fullbody_obs[:, -4:]),dim=-1)
    loss2=bce_loss(pred_con, curr_ref_fullbody_obs[:, -4*5:])

    return torch.mean(loss1)+loss2*5
    #curr_ref_pos = curr_ref_fullbody_obs.view(num_frames, 15, -1)[:, :, :3].contiguous().view(num_frames, -1)
    #curr_ref_ori = curr_ref_fullbody_obs.view(num_frames, 15, -1)[:, :, 3:9].contiguous().view(num_frames, -1)

    #curr_reconstructed_pos = curr_reconstructed_fullbody_obs.view(num_frames, 15, -1)[:, :, :3].contiguous().view(num_frames, -1)
    #curr_reconstructed_ori = curr_reconstructed_fullbody_obs.view(num_frames, 15, -1)[:, :, 3:9].contiguous().view(num_frames, -1)

    ## Compute the L1 differences between next_observation and target_observation for each component
    #loss_pos = weights['pos'] * torch.norm(curr_reconstructed_pos - curr_ref_pos, p=1, dim=-1)
    #loss_rot = weights['rot'] * torch.norm(curr_reconstructed_ori - curr_ref_ori, p=1, dim=-1)

    ## Sum the weighted L1 losses for all components
    #loss = loss_pos + loss_rot
    # print(f'loss pos: {loss_pos}')
    # print(f'loss rot: {loss_rot}')
    # print('---------------------------')
    #return torch.mean(loss)


# def mat6d2mat_vector_batch(v):
#     """
#     将6D向量表示转换为3x3旋转矩阵 (批处理版本)
#     :param v: 6D向量批，形如 [v1, v2, v3, v4, v5, v6]，形状 (batch_size, 6)
#     :return: 3x3旋转矩阵批，形状 (batch_size, 3, 3)
#     """
#     if v.shape[1] != 6:
#         raise ValueError("每个6D向量必须具有6个元素")
#
#     # 从6D向量中提取两个3D向量
#     r1 = v[:, :3]  # 第一个3D向量，形状 (batch_size, 3)
#     r2 = v[:, 3:]  # 第二个3D向量，形状 (batch_size, 3)
#
#     # 对第一个向量进行归一化处理
#     x = r1 / r1.norm(dim=-1, keepdim=True)  # 归一化，形状 (batch_size, 3)
#
#     # 计算第二个正交向量
#     z = torch.cross(x, r2, dim=-1)
#     z = z / z.norm(dim=-1, keepdim=True)  # 归一化
#
#     # 计算第三个正交向量
#     y = torch.cross(z, x, dim=-1)
#
#     # 拼接3个向量构成旋转矩阵
#     rotation_matrices = torch.stack([x, y, z], dim=-1)  # 形状 (batch_size, 3, 3)
#
#     return rotation_matrices


def mat6d2mat_vector_batch(v):
    """
    将6D向量表示转换为3x3旋转矩阵 (批处理版本)
    使用 Gram-Schmidt 正交化
    :param v: 6D向量批，形如 [v1, v2, v3, v4, v5, v6]，形状 (batch_size, 6)
    :return: 3x3旋转矩阵批，形状 (batch_size, 3, 3)
    """
    batch_size = v.shape[0]
    v = v.view(batch_size, -1, 6)
    if v.shape[-1] != 6:
        raise ValueError("每个6D向量必须具有6个元素")

    # 从6D向量中提取两个3D向量
    a = v[..., :3]  # 第一个3D向量，形状 (..., 3)
    b = v[..., 3:]  # 第二个3D向量，形状 (..., 3)

    # Gram-Schmidt 正交化
    # 1. 归一化第一个向量
    c1 = a / torch.norm(a, dim=-1, keepdim=True)  # 形状 (..., 3)

    # 2. 计算第二个向量的投影并减去
    proj = torch.sum(c1 * b, dim=-1, keepdim=True) * c1  # 投影分量
    c2 = b - proj  # 减去投影分量
    c2 = c2 / torch.norm(c2, dim=-1, keepdim=True)  # 归一化

    # 3. 计算第三个向量（叉积）
    c3 = torch.cross(c1, c2, dim=-1)  # 形状 (..., 3)

    # 拼接3个向量构成旋转矩阵
    rotation_matrices = torch.stack([c1, c2, c3], dim=-1)  # 形状 (..., 3, 3)

    return rotation_matrices


def global2projected_SRB_mocap_batch(SRB_rot_mats, SRB_origins, global_pos):
    """
    将全局状态转换为投影的 SRB 观测 (批处理版本)
    :param SRB_rot_mats: 旋转矩阵批，形状 (batch_size, 3, 3)
    :param SRB_origins: SRB原点批，形状 (batch_size, 3)
    :param global_pos: 全局位置，形状 (batch_size, 3)
    :return: 投影位置和旋转矩阵，形状分别为 (batch_size, 3) 和 (batch_size, 3, 3)
    """
    cdm_ori = SRB_rot_mats  # 旋转矩阵，形状 (batch_size, 3, 3)
    SRB_ori = cdm_ori.clone()

    x_SRB = SRB_ori[:, 0]  # 提取旋转矩阵的第一列，形状 (batch_size, 3)
    x_proj = torch.stack([x_SRB[:, 0], x_SRB[:, 1], torch.zeros_like(x_SRB[:, 0])], dim=1)  # 形状 (batch_size, 3)

    # 归一化处理
    norm_x_proj = x_proj.norm(dim=1, keepdim=True)
    x_ffSRB = torch.where(norm_x_proj != 0, x_proj / norm_x_proj, x_proj)  # 形状 (batch_size, 3)

    z_ffSRB = torch.tensor([0, 0, 1], dtype=x_ffSRB.dtype, device=x_ffSRB.device)  # z方向
    y_ffSRB = torch.cross(z_ffSRB.repeat(x_ffSRB.shape[0], 1), x_ffSRB, dim=1)  # y方向，形状 (batch_size, 3)

    ffSRB_ori = torch.stack([x_ffSRB, y_ffSRB, z_ffSRB.repeat(x_ffSRB.shape[0], 1)], dim=1)  # 形状 (batch_size, 3, 3)

    # 将SRB原点投影到平面
    SRB_origin_proj = SRB_origins.clone()
    SRB_origin_proj[:, 2] = 0  # 高度为0，形状 (batch_size, 3)

    # 计算投影位置
    local_pos_SRB_proj = torch.bmm(ffSRB_ori.transpose(1, 2), (global_pos - SRB_origin_proj).unsqueeze(-1)).squeeze(-1)

    return local_pos_SRB_proj, ffSRB_ori


def global2forward_facing_SRB_mocap_batch(SRB_rot_mats, SRB_origins, global_pos):
    """
    将全局状态转换为前向面 SRB 观测 (批处理版本)
    :param SRB_rot_mats: 旋转矩阵批，形状 (batch_size, 3, 3)
    :param SRB_origins: SRB原点批，形状 (batch_size, 3)
    :param global_pos: 全局位置，形状 (batch_size, 3)
    :return: 前向面位置和旋转矩阵，形状分别为 (batch_size, 3) 和 (batch_size, 3, 3)
    """
    cdm_ori = SRB_rot_mats  # 旋转矩阵，形状 (batch_size, 3, 3)
    SRB_ori = cdm_ori.clone()

    # 提取旋转矩阵的第一列，形状 (batch_size, 3)
    x_SRB = SRB_ori[:, 0]

    # 创建投影方向并归一化，形状 (batch_size, 3)
    x_proj = torch.stack([x_SRB[:, 0], x_SRB[:, 1], torch.zeros_like(x_SRB[:, 0])], dim=1)
    norm_x_proj = x_proj.norm(dim=1, keepdim=True)
    x_ffSRB = torch.where(norm_x_proj != 0, x_proj / norm_x_proj, x_proj)  # 形状 (batch_size, 3)

    # 定义z方向并计算y方向，形状 (batch_size, 3)
    z_ffSRB = torch.tensor([0, 0, 1], dtype=x_ffSRB.dtype, device=x_ffSRB.device)
    y_ffSRB = torch.cross(z_ffSRB.repeat(x_ffSRB.shape[0], 1), x_ffSRB, dim=1)  # y方向，形状 (batch_size, 3)

    # 构造前向面旋转矩阵，形状 (batch_size, 3, 3)
    ffSRB_ori = torch.stack([x_ffSRB, y_ffSRB, z_ffSRB.repeat(x_ffSRB.shape[0], 1)], dim=-1)

    # 将SRB原点投影到平面，形状 (batch_size, 3)
    SRB_origin_proj = SRB_origins.clone()
    SRB_origin_proj[:, 2] = 0  # 高度为0，形状 (batch_size, 3)

    # 计算前向面位置，形状 (batch_size, 3)
    local_pos_ffSRB = torch.bmm(ffSRB_ori.transpose(1, 2), (global_pos - SRB_origin_proj).unsqueeze(-1)).squeeze(-1)

    return local_pos_ffSRB, ffSRB_ori


def mat3x3_to_6d_batch(rotation_matrices):
    """
    将 (batch_size, numbodies, 3, 3) 的旋转矩阵批转换为 6D 表示 (批处理版本)
    :param rotation_matrices: 一个形状为 (batch_size, numbodies, 3, 3) 的 PyTorch 张量
    :return: 一个形状为 (batch_size, numbodies*6) 的张量
    """
    batch_size = rotation_matrices.shape[0]
    numbodies = rotation_matrices.shape[1]
    # 提取每个旋转矩阵的前两列
    result_6d = torch.zeros((batch_size, numbodies, 6), dtype=rotation_matrices.dtype, device=rotation_matrices.device)

    # 提取第一列和第二列
    result_6d[..., :3] = rotation_matrices[..., 0]  # 第一列，形状 (batch_size, 3)
    result_6d[..., 3:] = rotation_matrices[..., 1]  # 第二列，形状 (batch_size, 3)

    return result_6d


def z_axis_rotation_matrix_batch(angles):
    """
    批处理版本的 z 轴旋转矩阵计算。

    :param angles: 角度张量，形状为 (batch_size,)，每个元素为一个旋转角度
    :return: 旋转矩阵张量，形状为 (batch_size, 3, 3)
    """
    batch_size = angles.shape[0]
    cos_theta = torch.cos(angles)
    sin_theta = torch.sin(angles)

    # 初始化旋转矩阵，形状为 (batch_size, 3, 3)
    rotation_matrix = torch.zeros(batch_size, 3, 3, dtype=angles.dtype, device=angles.device)

    # 填充旋转矩阵的每一行
    rotation_matrix[:, 0, 0] = cos_theta  # 第一行第一列
    rotation_matrix[:, 0, 1] = -sin_theta  # 第一行第二列
    rotation_matrix[:, 1, 0] = sin_theta  # 第二行第一列
    rotation_matrix[:, 1, 1] = cos_theta  # 第二行第二列
    rotation_matrix[:, 2, 2] = 1.0  # 第三行第三列

    return rotation_matrix


def get_z_rotation_angles(rotation_matrices):
    """
    Given a batch of rotation matrices, calculate the rotation angle around the global Z-axis.

    :param rotation_matrices: A tensor of shape (batch_size, 3, 3) representing rotation matrices.
    :return: A tensor of shape (batch_size,) representing the rotation angles around the Z-axis in radians.
    """
    # Extract the relevant components from the rotation matrices
    R_11 = rotation_matrices[:, 0, 0]  # (batch_size,)
    R_21 = rotation_matrices[:, 1, 0]  # (batch_size,)

    # Calculate the angle using atan2
    angles = torch.atan2(R_21, R_11)  # (batch_size,)

    return angles


def convert_6d_to_matrix(rot_6d):
    """
    将6D旋转表示转换为3x3旋转矩阵
    :param rot_6d: shape (..., 6), 6D旋转表示
    :return: shape (..., 3, 3), 3x3旋转矩阵, 按列堆叠
    """
    # 限制输入范围
    rot_6d = torch.clamp(rot_6d, min=-1e6, max=1e6)
    a1 = rot_6d[..., :3]  # 取6D表示的前3个数
    a2 = rot_6d[..., 3:]  # 取6D表示的后3个数

    b1 = torch.nn.functional.normalize(a1, dim=-1)
    a2_proj = torch.sum(b1*a2, dim=-1, keepdim=True) * b1
    b2 = torch.nn.functional.normalize(a2 - a2_proj, dim=-1)

    b3 = torch.cross(b1, b2, dim=-1)

    matrix = torch.stack((b1, b2, b3), dim=-1)
    return matrix


def global_state2obs(global_state):
    SRB_pos = global_state[:, 0:3]
    SRB_ori_6D = global_state[:, 3:9]
    SRB_lin_vel = global_state[:, 9:12]
    SRB_ang_vel = global_state[:, 12:15]
    left_foot_pos = global_state[:, 15:18]
    left_foot_ori_6D = global_state[:, 18:24]
    right_foot_pos = global_state[:, 24:27]
    right_foot_ori_6D = global_state[:, 27:33]
    feet_contact_one_hot = global_state[:, 33:37]

    # get local SRB obs, in projected SRB frame
    SRB_ori_mat = convert_6d_to_matrix(SRB_ori_6D)
    SRB_pos_proj, projSRB_ori = global2projected_SRB_mocap_batch(SRB_ori_mat, SRB_pos, SRB_pos)
    height_proj = SRB_pos_proj[:, 2:3]
    SRB_ori_proj = torch.matmul(projSRB_ori.transpose(-1, -2), SRB_ori_mat)
    SRB_ori_6D_proj = matrix_to_6d(SRB_ori_proj)
    SRB_lin_vel_proj = torch.matmul(projSRB_ori.transpose(-1, -2), SRB_lin_vel.unsqueeze(-1)).squeeze(-1)
    SRB_ang_vel_proj = torch.matmul(projSRB_ori.transpose(-1, -2), SRB_ang_vel.unsqueeze(-1)).squeeze(-1)

    # get local feet obs, in forwardfacing SRB frame
    left_foot_pos_ff, ffSRB_ori = global2forward_facing_SRB_mocap_batch(SRB_ori_mat, SRB_pos, left_foot_pos)
    left_foot_ori_mat = convert_6d_to_matrix(left_foot_ori_6D)
    left_foot_ori_mat_ff = torch.matmul(ffSRB_ori.transpose(-1, -2), left_foot_ori_mat)
    left_foot_ori_z_ff = get_z_rotation_angles(left_foot_ori_mat_ff).unsqueeze(-1)

    right_foot_pos_ff, _ = global2forward_facing_SRB_mocap_batch(SRB_ori_mat, SRB_pos, right_foot_pos)
    right_foot_ori_mat = convert_6d_to_matrix(right_foot_ori_6D)
    right_foot_ori_mat_ff = torch.matmul(ffSRB_ori.transpose(-1, -2), right_foot_ori_mat)
    right_foot_ori_z_ff = get_z_rotation_angles(right_foot_ori_mat_ff).unsqueeze(-1)

    up_dir = SRB_ori_mat.transpose(-1, -2)[:, :, 2]  # (batch_size, 3)
    obs = torch.cat([height_proj, SRB_ori_6D_proj, SRB_lin_vel_proj, SRB_ang_vel_proj,
                     left_foot_pos_ff, left_foot_ori_z_ff, right_foot_pos_ff, right_foot_ori_z_ff, up_dir,
                     feet_contact_one_hot], dim=-1)

    return obs  # (batch_size, obs_dim)


def get_contact_loss(contact_logits, target_observation):
    ref_contact_state = target_observation[:, 24:28]
    ref_contact_indices = torch.argmax(ref_contact_state, dim=-1)
    contact_loss = F.cross_entropy(contact_logits, ref_contact_indices)
    return contact_loss

def findDevice():
    device=None
    # mocap SRB state and mocap full body state
    if torch.cuda.is_available():
        device = torch.device("cuda")
    elif torch.mps.is_available():
        device = torch.device("mps")
    else:
        device = torch.device("cpu")
    return device

def train_FullbodyVAE(ref_SRB_obs, ref_full_body_obs, FullbodyVAE_encoder, FullbodyVAE_decoder, optimizer, beta=0.01):
    device=findDevice()

    FullbodyVAE_encoder.train()
    FullbodyVAE_decoder.train()

    # 定义设备（如果有 GPU 可用，则使用 'cuda'，否则使用 'cpu'）
    num_frames = ref_SRB_obs.shape[0]
    # 初始化总损失
    total_loss = torch.mean(torch.zeros(num_frames, device=device))  # (1,)

    # target observation from reference states
    SRB_obs = ref_SRB_obs
    fullbody_obs = ref_full_body_obs

    latent_code, mu_q = FullbodyVAE_encoder(SRB_obs, fullbody_obs)

    # 计算kl loss
    kl_loss = FullbodyVAE_encoder.kl_loss(mu_q)

    # 使用 policy decoder 采样 action a_t，使用 observation 和 z_t
    reconstructed_fullbody_obs = FullbodyVAE_decoder(SRB_obs, latent_code)  # shape: (batch_size, num_bodies*9)

    # 计算reconstruction loss
    rec_loss = get_rec_loss(reconstructed_fullbody_obs, fullbody_obs)
    # rec_loss = get_rec_loss(reconstructed_fullbody_obs, curr_fullbody_obs, reconstructed_fullbody_obs_list[t-1], last_fullbody_obs)

    total_loss += rec_loss + beta * kl_loss
    # reconstructed_fullbody_obs_list.append(reconstructed_fullbody_obs)

    # mean_loss = torch.mean(total_loss)
    mean_loss = total_loss
    # ---------------- 反向传播和权重更新 ----------------

    # 清除之前的梯度
    optimizer.zero_grad()

    # 反向传播计算梯度
    mean_loss.backward()

    # 对 SRBVAE的梯度进行裁剪，防止梯度爆炸/消失
    torch.nn.utils.clip_grad_norm_(FullbodyVAE_encoder.parameters(), 1.0)
    torch.nn.utils.clip_grad_norm_(FullbodyVAE_decoder.parameters(), 1.0)

    # 使用优化器更新模型参数
    optimizer.step()

    # 返回 batch 的平均损失
    return mean_loss.item()
