import math

from gymnasium import spaces
from gymnasium import Env
import mujoco
import numpy as np
import random
from scipy.spatial.transform import Rotation as R
from .testSRB_mujoco_original_viewer_terrain  import *
from .testSRB_mujoco_original_viewer_terrain  import SRBdata as SRBDataProcess
"""
    RL env
"""
import work.controlmodule as control # only for LQR filters
from . import Policy_prior_posterior_MoE_window

def set_feet_site_this_step_env(model, data, feet_pos_list_this_step, feet_ori_list_this_step, desired_left_foot_pos,
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


class SRBEnv(Env):
    def __init__(self, mocap_path_list):
        super(SRBEnv, self).__init__()
        model_path = 'gym_trackSRB/Characters/SRB5_playground.xml'
        self.model_path=model_path
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.minLogQ=9
        # 初始化当前帧数为0
        # Initialize the current frame count to 0
        self.current_frame = 0
        self.window_size = 5
        self.SRB_data_num = len(mocap_path_list)
        self.mocap_path_list=mocap_path_list
        self.SRB_data_list = []
        for i, mocap_path in enumerate(mocap_path_list):
            self.SRB_data_list.append(None)
            #self.loadMocap(i, mocap_path) lazy laoding
        self.left_foot_in_contact = False
        self.right_foot_in_contact = False
        self.left_foot_contact_ori = None
        self.right_foot_contact_ori = None
        self.left_foot_contact_pos_global = None
        self.right_foot_contact_pos_global = None
        self.next_predicted_left_foot_pos = None
        self.next_predicted_right_foot_pos = None
        self.next_contact_one_hot = None
        self.random_initial_frame = None
        # self.next_contact_prob = None
        self.site_to_apply_force_on = [-1, -1, -1, -1, -1, -1]
        self.action = None
        # 加载模型和数据/ Load the model
        self.cdm_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'cdm')
        self.left_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "left_foot_center")
        self.right_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "right_foot_center")
        self.site_id_list = [self.left_site_id, self.right_site_id]
        friction_coef = self.model.geom_friction[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, 'floor')]
        self.mu = 1.2 * friction_coef[0]
        self.SRB_pos_list = []
        self.SRB_ori_list = []
        self.simulated_feet_pos_list = []
        self.simulated_feet_ori_list = []
        self.contact_signal_list = []

        # 定义连续部分和分类部分/ Define continuous and categorical parts
        low_continuous = np.array([-3.14] * 18, dtype=np.float32)  # 连续部分
        low_class = np.array([-3.14] * 4, dtype=np.float32)  # 分类部分
        low = np.concatenate([low_continuous, low_class])

        high_continuous = np.array([3.14] * 18, dtype=np.float32)  # 连续部分
        high_class = np.array([3.14] * 4, dtype=np.float32)  # 分类部分
        high = np.concatenate([high_continuous, high_class])
        # 定义动作空间和观测空间
        self.action_space = spaces.Box(low=low, high=high, dtype=np.float32)
        self.observation_space = spaces.Box(low=-float('inf'), high=float('inf'), shape=(25*6,),
                                            dtype=np.float32)

        # 初始化环境状态
        self.testing_mode = False

        # add exteranal force
        self.frame_counter = 0  # 累计帧数
        #self.impulse_interval = 360  # 每360帧施加一次外力
        self.impulse_interval = 1e9  # 每360帧施加一次外力
        self.impulse_duration = 24  # 每次施加24帧
        self.impulse = 0  # 初始时不施加力
        self.force = [700, 700, -350]
        self.f = np.zeros(6)

    def update_impulse(self):
        # 每帧调用一次这个函数
        if self.frame_counter % self.impulse_interval == self.impulse_interval-1:
            self.impulse = self.impulse_duration  # 开始施加外力

        if self.impulse > 0:
            self.impulse -= 1
            f = np.zeros(6)
            f[0:len(self.force)] = self.force

            self.data.xfrc_applied[1] = f
        else:
            f = np.zeros(6)
            self.data.xfrc_applied[1] = f
        self.f = f
        self.frame_counter += 1

    def step(self, action):
        _, ffSRB_ori = global2forward_facing_SRB(self.data, self.cdm_id, self.data.qpos[:3].copy(), self.data.qpos[:3].copy())
        # SRB_pos_proj, projSRB_ori = global2projected_SRB(self.data, self.cdm_id, self.data.qpos[:3].copy(), self.data.qpos[:3].copy())
        SRB_ori = self.data.xmat[self.cdm_id].reshape(3, 3)
        ffSRB_origin = self.data.qpos[:3].copy()
        ffSRB_origin[2] = 0

        desired_com_vel_global = np.concatenate([SRB_ori @ action[4:7], action[7:10]])

        # # relative to current state
        delta = action[10:16]
        target_pos_com_local = delta[:3].copy()
        target_pos_com_global = SRB_ori @ target_pos_com_local + self.data.qpos[:3]

        target_axisangle_rot_com_local = delta[3:].copy()
        # 如果角度非常小，直接返回单位矩阵/ If the Angle is very small, return the identity matrix directly
        if np.linalg.norm(target_axisangle_rot_com_local) < 1e-6:
            target_rot_com_local = np.eye(3)
        else:
            target_rot_com_local = axis_angle_to_rotation_matrix(target_axisangle_rot_com_local)
        target_rot_com_global = SRB_ori @ target_rot_com_local

        next_T = transformation_matrix(target_rot_com_global, target_pos_com_global)
        target_vel = desired_com_vel_global

        # predicted contact signal at next step
        self.next_contact_one_hot = action[18:22]
        #     L R  LR
        # 00 01 10 11
        next_contact_signal = np.argmax(self.next_contact_one_hot)

        contact_prob=Policy_prior_posterior_MoE_window._last_contact_prob

        left_foot_currently_in_contact = False
        right_foot_currently_in_contact = False


        if contact_prob[0]==None:
            if next_contact_signal == 0:
                left_foot_currently_in_contact = False
                right_foot_currently_in_contact = False
            elif next_contact_signal == 1:
                left_foot_currently_in_contact = True
                right_foot_currently_in_contact = False
            elif next_contact_signal == 2:
                left_foot_currently_in_contact = False
                right_foot_currently_in_contact = True
            else:
                left_foot_currently_in_contact = True
                right_foot_currently_in_contact = True
        else:
            import work.rendermodule as RE
            RE.output('contactProb', contact_prob)
            # taesoo
            confidence=0.7
            if self.left_foot_in_contact  and contact_prob[0]<(1.0-confidence):
                left_foot_currently_in_contact = False
            elif not self.left_foot_in_contact  and contact_prob[0]>confidence:
                left_foot_currently_in_contact = True
            else:
                left_foot_currently_in_contact = self.left_foot_in_contact  

            if self.right_foot_in_contact  and contact_prob[1]<(1.0-confidence):
                right_foot_currently_in_contact = False
            elif not self.right_foot_in_contact  and contact_prob[1]>confidence:
                right_foot_currently_in_contact = True
            else:
                right_foot_currently_in_contact = self.right_foot_in_contact  



        last_left_foot_in_contact = self.left_foot_in_contact
        last_right_foot_in_contact = self.right_foot_in_contact

        # # add swing window_size
        # window_size = 4
        # left_foot_swing_window_size = 0
        # right_foot_swing_window_size = 0
        # if left_foot_currently_in_contact and self.current_frame >= window_size - 1:
        #     for i in range(self.current_frame, self.current_frame - window_size, -1):
        #         if self.contact_signal_list[i] == 0 or self.contact_signal_list[i] == 2:
        #             left_foot_swing_window_size += 1
        #         else:
        #             break
        # if right_foot_currently_in_contact and self.current_frame >= window_size - 1:
        #     for i in range(self.current_frame, self.current_frame - window_size, -1):
        #         if self.contact_signal_list[i] == 0 or self.contact_signal_list[i] == 1:
        #             right_foot_swing_window_size += 1
        #         else:
        #             break
        # if 0 < left_foot_swing_window_size < window_size:
        #     left_foot_currently_in_contact = False
        # if 0 < right_foot_swing_window_size < window_size:
        #     right_foot_currently_in_contact = False
        # next_contact_signal = np.argmax(
        #     foot_contact_to_one_hot([left_foot_currently_in_contact, right_foot_currently_in_contact]))

        if self.left_foot_in_contact and not left_foot_currently_in_contact:
            self.left_foot_in_contact = False
        elif not self.left_foot_in_contact and left_foot_currently_in_contact:
            self.left_foot_in_contact = True

        if self.right_foot_in_contact and not right_foot_currently_in_contact:
            self.right_foot_in_contact = False
        elif not self.right_foot_in_contact and right_foot_currently_in_contact:
            self.right_foot_in_contact = True

        #assert(len(self.simulated_feet_pos_list)==self.current_frame+1) #tested ok.
        last_left_foot_pos_xy = self.simulated_feet_pos_list[-1][0][:2]
        last_right_foot_pos_xy = self.simulated_feet_pos_list[-1][1][:2]
        curr_com_lin_vel_xy = self.data.qvel[:2]
        foot_speed_upper_bound = max(5 * np.linalg.norm(curr_com_lin_vel_xy), 5)  # the mannual least upper limit is 5

        # 检查左脚的接触状态，如果下一帧contact，z=0，否则z=np.nan（仅用于判定site_to_apply_force_on）
        # Check the contact status of the left foot, if the next frame is contact, z=0, otherwise z=np.nan (only used to determine site_to_apply_force_on)
        if self.contactFilter[0]==None:
            self.initFilter(0, action[16], action[0:2], ffSRB_ori, ffSRB_origin) 
            self.initFilter(1, action[17], action[2:4], ffSRB_ori, ffSRB_origin) 



        if not self.left_foot_in_contact:
            #filterFoot0 outputs self.left_foot_contact_pos_global
            self.filterFoot(0, action[16], action[0:2], ffSRB_ori, ffSRB_origin)


            # desired_left_foot_speed_xy = np.linalg.norm(
            #     self.left_foot_contact_pos_global[:2] - last_left_foot_pos_xy) / self.mocap_dt
            # desired_left_foot_moving_direction_xy = (self.left_foot_contact_pos_global[
            #                                          :2] - last_left_foot_pos_xy) / np.linalg.norm(
            #     self.left_foot_contact_pos_global[:2] - last_left_foot_pos_xy)
            # true_left_foot_speed_xy = min(desired_left_foot_speed_xy, foot_speed_upper_bound)
            # self.left_foot_contact_pos_global[
            # :2] = last_left_foot_pos_xy + desired_left_foot_moving_direction_xy * true_left_foot_speed_xy * self.mocap_dt

            left_foot_height_terrain, _ = terrain_height_ray(self.model, self.data, self.left_foot_contact_pos_global[0], self.left_foot_contact_pos_global[1])
            self.left_foot_contact_pos_global[2] = left_foot_height_terrain
            self.left_foot_contact_ori = ffSRB_ori @ z_axis_rotation_matrix(self.contactFilter[0])
            self.next_predicted_left_foot_pos = self.left_foot_contact_pos_global.copy()
            self.next_predicted_left_foot_pos[2] = np.nan
        else:
            left_foot_height_terrain, geom_id = terrain_height_ray(self.model, self.data, self.left_foot_contact_pos_global[0], self.left_foot_contact_pos_global[1])
            geom_bodyid = self.model.geom_bodyid[geom_id]
            body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, geom_bodyid)
            # when stepping on pyramid
            if body_name.startswith("pyramid") and mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, geom_bodyid+1).startswith("pyramid") and not last_left_foot_in_contact:
                foot_xy = self.left_foot_contact_pos_global[:2]
                geom_size = self.model.geom_size[geom_id]
                next_geom_size = self.model.geom_size[geom_id+1]
                center = self.data.xpos[geom_bodyid]
                half_size = (geom_size[:2] + next_geom_size[:2]) / 2
                x_min = center[0] - half_size[0]
                x_max = center[0] + half_size[0]
                y_min = center[1] - half_size[1]
                y_max = center[1] + half_size[1]

                # 判断哪个边最近，并把脚点投影到那条边上
                dx_min = abs(foot_xy[0] - x_min)
                dx_max = abs(foot_xy[0] - x_max)
                dy_min = abs(foot_xy[1] - y_min)
                dy_max = abs(foot_xy[1] - y_max)

                clamped_x = np.clip(foot_xy[0], x_min, x_max)
                clamped_y = np.clip(foot_xy[1], y_min, y_max)
                min_dist = min(dx_min, dx_max, dy_min, dy_max)

                if min_dist == dx_min:
                    projected_pos = np.array([x_min, clamped_y])
                elif min_dist == dx_max:
                    projected_pos = np.array([x_max, clamped_y])
                elif min_dist == dy_min:
                    projected_pos = np.array([clamped_x, y_min])
                else:
                    projected_pos = np.array([clamped_x, y_max])
                self.left_foot_contact_pos_global[:2] = projected_pos

            self.updateFilter(0, self.left_foot_contact_ori, ffSRB_ori, self.left_foot_contact_pos_global, ffSRB_origin) 
            self.next_predicted_left_foot_pos = self.left_foot_contact_pos_global.copy()
            self.next_predicted_left_foot_pos[2] = 0

        # 检查右脚的接触状态，如果下一帧contact，z=0，否则z=np.nan（仅用于判定site_to_apply_force_on）
        # Check the contact status of the right foot, if the next frame is contact, z=0, otherwise z=np.nan (only used to determine site_to_apply_force_on)
        if not self.right_foot_in_contact:
            #filterFoot1 outputs self.right_foot_contact_pos_global
            self.filterFoot(1, action[17], action[2:4], ffSRB_ori, ffSRB_origin)


            # desired_right_foot_speed_xy = np.linalg.norm(
            #     self.right_foot_contact_pos_global[:2] - last_right_foot_pos_xy) / self.mocap_dt
            # desired_right_foot_moving_direction_xy = (self.right_foot_contact_pos_global[
            #                                           :2] - last_right_foot_pos_xy) / np.linalg.norm(
            #     self.right_foot_contact_pos_global[:2] - last_right_foot_pos_xy)
            # true_right_foot_speed_xy = min(desired_right_foot_speed_xy, foot_speed_upper_bound)
            # self.right_foot_contact_pos_global[
            # :2] = last_right_foot_pos_xy + desired_right_foot_moving_direction_xy * true_right_foot_speed_xy * self.mocap_dt

            right_foot_height_terrain, _ = terrain_height_ray(self.model, self.data, self.right_foot_contact_pos_global[0], self.right_foot_contact_pos_global[1])
            self.right_foot_contact_pos_global[2] = right_foot_height_terrain
            self.right_foot_contact_ori = ffSRB_ori @ z_axis_rotation_matrix(action[17])
            self.next_predicted_right_foot_pos = self.right_foot_contact_pos_global.copy()
            self.next_predicted_right_foot_pos[2] = np.nan
        else:
            right_foot_height_terrain, geom_id = terrain_height_ray(self.model, self.data, self.right_foot_contact_pos_global[0], self.right_foot_contact_pos_global[1])
            geom_bodyid = self.model.geom_bodyid[geom_id]
            body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, geom_bodyid)
            # when stepping on pyramid
            if body_name!=None and body_name.startswith("pyramid") and mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, geom_bodyid+1).startswith("pyramid") and not last_right_foot_in_contact:
                foot_xy = self.right_foot_contact_pos_global[:2]
                geom_size = self.model.geom_size[geom_id]
                next_geom_size = self.model.geom_size[geom_id+1]
                center = self.data.xpos[geom_bodyid]
                half_size = (geom_size[:2] + next_geom_size[:2]) / 2
                x_min = center[0] - half_size[0]
                x_max = center[0] + half_size[0]
                y_min = center[1] - half_size[1]
                y_max = center[1] + half_size[1]

                clamped_x = np.clip(foot_xy[0], x_min, x_max)
                clamped_y = np.clip(foot_xy[1], y_min, y_max)

                # 判断哪个边最近，并把脚点投影到那条边上
                dx_min = abs(foot_xy[0] - x_min)
                dx_max = abs(foot_xy[0] - x_max)
                dy_min = abs(foot_xy[1] - y_min)
                dy_max = abs(foot_xy[1] - y_max)

                min_dist = min(dx_min, dx_max, dy_min, dy_max)

                if min_dist == dx_min:
                    projected_pos = np.array([x_min, clamped_y])
                elif min_dist == dx_max:
                    projected_pos = np.array([x_max, clamped_y])
                elif min_dist == dy_min:
                    projected_pos = np.array([clamped_x, y_min])
                else:
                    projected_pos = np.array([clamped_x, y_max])
                self.right_foot_contact_pos_global[:2] = projected_pos

            self.updateFilter(1, self.right_foot_contact_ori, ffSRB_ori, self.right_foot_contact_pos_global, ffSRB_origin) 
            self.next_predicted_right_foot_pos = self.right_foot_contact_pos_global.copy()
            self.next_predicted_right_foot_pos[2] = 0

        next_predicted_feet_pos = np.stack((self.next_predicted_left_foot_pos, self.next_predicted_right_foot_pos))
        self.site_to_apply_force_on = return_site_to_apply_force_on(self.model, next_predicted_feet_pos)

        # 使用接触地面时由action得到的foot global position/ Use the foot global position obtained by action when touching the ground
        desired_left_foot_pos_global = self.left_foot_contact_pos_global
        desired_right_foot_pos_global = self.right_foot_contact_pos_global
        curr_simulated_feet_ori = np.stack((self.left_foot_contact_ori, self.right_foot_contact_ori))
        self.contact_forces_total_list = []
        # 一次rl step中，在mujoco中sim 4次/ In one rl step, sim 4 times in mujoco
        for _ in range(round(self.mocap_dt/self.model.opt.timestep)):
            self.data.qfrc_applied.fill(0)
            mujoco.mj_step1(self.model, self.data)
            # add external forces
            # # taesoo begin
            if hasattr(self, 'impulse'):
                self.update_impulse()
            # taesoo end
            # get which foot is in contact state, -1 means swing state, otherwise is the global foot position
            set_feet_site_this_step_env(self.model, self.data, next_predicted_feet_pos, curr_simulated_feet_ori,
                                    desired_left_foot_pos_global, desired_right_foot_pos_global)
            if not self.left_foot_in_contact and not self.right_foot_in_contact:  # if no foot is on the floor, pass
                pass
            else:
                _, _, self.contact_forces_list = solve_qp_for_contact_forces(self.model, self.data, next_T,
                                                                             target_vel, self.site_to_apply_force_on, self.mu)
                self.contact_forces_total_list.append(self.contact_forces_list)
                j = 0
                for site_id in self.site_to_apply_force_on:
                    if site_id == -1:
                        continue
                    else:
                        apply_force_at_site(self.model, self.data, site_id, self.contact_forces_list[j])  # apply contact forces
                        # if self.testing_mode:
                        #     add_force_vector(viewer=)
                        j += 1
            mujoco.mj_step2(self.model, self.data)
        set_feet_site_this_step_env(self.model, self.data, next_predicted_feet_pos, curr_simulated_feet_ori,
                                desired_left_foot_pos_global, desired_right_foot_pos_global)
        temp_SRB_pos = self.data.qpos[:3].copy()
        temp_SRB_ori = self.data.xmat[self.cdm_id].copy().reshape(3, 3)
        temp_feet_pos = [self.data.site_xpos[site_id].copy() for site_id in self.site_id_list]
        temo_feet_ori = [self.data.site_xmat[site_id].copy().reshape(3, 3) for site_id in self.site_id_list]
        temp_contact_signal = next_contact_signal
        self.SRB_pos_list.append(temp_SRB_pos)
        self.SRB_ori_list.append(temp_SRB_ori)
        self.simulated_feet_pos_list.append(temp_feet_pos)
        self.simulated_feet_ori_list.append(temo_feet_ori)
        self.contact_signal_list.append(temp_contact_signal)

        maxHistory=20
        if len(self.SRB_pos_list)>maxHistory:
            #prevent consuming too much memory
            self.SRB_pos_list.pop(0)
            self.SRB_ori_list.pop(0)
            self.simulated_feet_pos_list.pop(0)
            self.simulated_feet_ori_list.pop(0)
            self.contact_signal_list.pop(0)


        self.action = action
        # get observation
        observation = self._get_obs()

        # get reward
        reward, reward_info = self._get_reward()

        # get done
        done = self._get_done()

        truncated = False

        # 将奖励详情加入 info/ Add reward details to info
        info = {'reward_info': reward_info}

        self.current_frame += 1

        return observation, reward, done, truncated, info

    def _get_reward(self):
        return 0, {}

    def reset(self, seed=None, options=None, mocap_id=0, initial_pos_planar=np.array([0.0, 0.0])):
        # 重置环境状态/ Reset environment state
        self.contactFilter=[None]*4
        if not self.testing_mode:  # Random state initialization
            # Randomly choose a SRBdata as reference
            random_SRBdata_id = random.randrange(self.SRB_data_num)
            self.loadMocap(random_SRBdata_id )
            self.SRBdata = self.SRB_data_list[random_SRBdata_id]["SRBdata"]
            self.mocap_dt = self.SRB_data_list[random_SRBdata_id]["mocap_dt"]
            self.cycle_length = self.SRB_data_list[random_SRBdata_id]["cycle_length"]
            self.mocap_qpos = self.SRB_data_list[random_SRBdata_id]["mocap_qpos"]
            self.mocap_pos_com = self.SRB_data_list[random_SRBdata_id]["mocap_pos_com"]
            self.mocap_ori_com = self.SRB_data_list[random_SRBdata_id]["mocap_ori_com"]
            self.mocap_vel_com = self.SRB_data_list[random_SRBdata_id]["mocap_vel_com"]
            self.feet_pos_list = self.SRB_data_list[random_SRBdata_id]["feet_pos_list"]
            self.feet_ori_list = self.SRB_data_list[random_SRBdata_id]["feet_ori_list"]
            self.feet_pos_list_original = self.SRB_data_list[random_SRBdata_id]["feet_pos_list_original"]
            self.mocap_height_com_terrain = self.SRB_data_list[random_SRBdata_id]["mocap_height_com_terrain"]
            # Randomly choose an initial frame
            self.random_initial_frame = random.randint(0, self.cycle_length-2)
            self.current_frame = 0
            self.set_state(self.random_initial_frame)
        else:
            # use the chosen motion with mocap_id
            self.loadMocap(mocap_id, initial_pos_planar)
            self.SRBdata = self.SRB_data_list[mocap_id]["SRBdata"]
            self.mocap_dt = self.SRB_data_list[mocap_id]["mocap_dt"]
            self.cycle_length = self.SRB_data_list[mocap_id]["cycle_length"]
            self.mocap_qpos = self.SRB_data_list[mocap_id]["mocap_qpos"]
            self.mocap_pos_com = self.SRB_data_list[mocap_id]["mocap_pos_com"]
            self.mocap_ori_com = self.SRB_data_list[mocap_id]["mocap_ori_com"]
            self.mocap_vel_com = self.SRB_data_list[mocap_id]["mocap_vel_com"]
            self.feet_pos_list = self.SRB_data_list[mocap_id]["feet_pos_list"]
            self.feet_ori_list = self.SRB_data_list[mocap_id]["feet_ori_list"]
            self.feet_pos_list_original = self.SRB_data_list[mocap_id]["feet_pos_list_original"]
            self.mocap_height_com_terrain = self.SRB_data_list[mocap_id]["mocap_height_com_terrain"]

            # Set initial frame as 0
            self.random_initial_frame = 0
            self.current_frame = 0
            self.set_state(self.random_initial_frame)

        info = {}
        return self._get_obs(), info

    def set_state(self, initial_frame):

        self.SRB_pos_list = []
        self.SRB_ori_list = []
        self.simulated_feet_pos_list = []
        self.simulated_feet_ori_list = []
        self.contact_signal_list = []

        # self.next_contact_prob = None
        self.site_to_apply_force_on = [-1, -1, -1, -1, -1, -1]
        # set SRB
        if not self.testing_mode:
            reset_noise_scale = 5e-2
            random_noise = np.random.uniform(
                low=-reset_noise_scale,
                high=reset_noise_scale,
                size=3
            )
            vel_noise = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            vel_noise[:3] = random_noise
            noise_quat = random_quaternion_perturbation(reset_noise_scale)
            perturbed_quat = quaternion_multiply(noise_quat, self.mocap_qpos[initial_frame][3:7].copy())
            perturbed_quat /= np.linalg.norm(perturbed_quat)
            self.data.qpos = np.concatenate([self.mocap_qpos[initial_frame][:3], perturbed_quat])
            self.data.qvel = self.mocap_vel_com[initial_frame] + vel_noise
        else:
            self.data.qpos = self.mocap_qpos[initial_frame]
            self.data.qvel = self.mocap_vel_com[initial_frame]
        # set feet
        curr_mocap_feet_pos_original = self.feet_pos_list_original[initial_frame]
        curr_mocap_feet_pos_terrain = self.feet_pos_list[initial_frame]
        curr_mocap_feet_ori = self.feet_ori_list[initial_frame]

        self.left_foot_contact_ori = curr_mocap_feet_ori[0]
        self.right_foot_contact_ori = curr_mocap_feet_ori[1]

        self.site_to_apply_force_on = return_site_to_apply_force_on(self.model, curr_mocap_feet_pos_original)
        self.left_foot_contact_pos_global = curr_mocap_feet_pos_terrain[0]
        self.right_foot_contact_pos_global = curr_mocap_feet_pos_terrain[1]
        # 根据curr_mocap_feet_pos更新当前脚的接触状态，标记为-1的为swing，标记为site id的为contact
        # Update the contact status of the current foot according to curr_mocap_feet_pos. Those marked -1 are swing and those marked site id are contact
        self.left_foot_in_contact = not all(site == -1 for site in self.site_to_apply_force_on[:3])
        self.right_foot_in_contact = not all(site == -1 for site in self.site_to_apply_force_on[3:])
        set_feet_site_this_step_env(self.model, self.data, curr_mocap_feet_pos_original, curr_mocap_feet_ori,
                                self.left_foot_contact_pos_global, self.right_foot_contact_pos_global)
        mujoco.mj_forward(self.model, self.data)
        set_feet_site_this_step_env(self.model, self.data, curr_mocap_feet_pos_original, curr_mocap_feet_ori,
                                self.left_foot_contact_pos_global, self.right_foot_contact_pos_global)
        temp_SRB_pos = self.data.qpos[:3].copy()
        temp_SRB_ori = self.data.xmat[self.cdm_id].copy().reshape(3, 3)
        temp_feet_pos = [self.data.site_xpos[site_id].copy() for site_id in self.site_id_list]
        temo_feet_ori = [self.data.site_xmat[site_id].copy().reshape(3, 3) for site_id in self.site_id_list]
        if self.left_foot_in_contact == False and self.right_foot_in_contact == False:
            temp_contact_signal = 0
        elif self.left_foot_in_contact == True and self.right_foot_in_contact == False:
            temp_contact_signal = 1
        elif self.left_foot_in_contact == False and self.right_foot_in_contact == True:
            temp_contact_signal = 2
        else:
            temp_contact_signal = 3
        self.SRB_pos_list.append(temp_SRB_pos)
        self.SRB_ori_list.append(temp_SRB_ori)
        self.simulated_feet_pos_list.append(temp_feet_pos)
        self.simulated_feet_ori_list.append(temo_feet_ori)
        self.contact_signal_list.append(temp_contact_signal)

    def _get_obs(self):
        return self.get_obs_window(self.window_size)

    def _get_done(self):
        # 定义何时结束一回合
        done = False
        # if self.testing_mode:
        #     return done

        # ET
        height = self.data.qpos[2]
        ref_height = self.mocap_pos_com[self.current_frame + 1 + self.random_initial_frame, 2].copy()
        srb_ori = self.data.xmat[self.cdm_id].reshape(3, 3)
        _, projsrb_ori = global2projected_SRB_mocap(srb_ori, self.data.qpos[:3], self.data.qpos[:3])
        srb_vertical_axis_proj = (projsrb_ori.T @ srb_ori)[:, 2]
        srb_vertical_axis = srb_ori[:, 2]  # 垂直轴（局部 z 轴）

        global_z_axis = np.array([0, 0, 1])  # 全局 z 轴
        cos_angle = np.dot(srb_vertical_axis, global_z_axis) / (
                np.linalg.norm(srb_vertical_axis) * np.linalg.norm(global_z_axis))
        angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))  # 确保值在 -1 到 1 之间，防止浮点数误差
        angle_degrees = np.degrees(angle)

        ref_srb_ori = self.mocap_ori_com[self.current_frame + 1 + self.random_initial_frame]
        _, ref_projsrb_ori = global2projected_SRB_mocap(ref_srb_ori, self.mocap_pos_com[self.current_frame + 1 + self.random_initial_frame].copy(), self.mocap_pos_com[self.current_frame + 1 + self.random_initial_frame].copy())
        ref_srb_vertical_axis_proj = (ref_projsrb_ori.T@ref_srb_ori)[:, 2]
        ref_srb_vertical_axis = ref_srb_ori[:, 2]
        # cos_angle_from_ref = np.dot(srb_vertical_axis, ref_srb_vertical_axis) / (
        #         np.linalg.norm(srb_vertical_axis) * np.linalg.norm(ref_srb_vertical_axis))
        # angle_from_ref = np.arccos(np.clip(cos_angle_from_ref, -1.0, 1.0))
        # angle_degrees_from_ref = np.degrees(angle_from_ref)
        #
        cos_angle_from_ref_proj = np.dot(srb_vertical_axis_proj, ref_srb_vertical_axis_proj) / (
                np.linalg.norm(srb_vertical_axis_proj) * np.linalg.norm(ref_srb_vertical_axis_proj))
        angle_from_ref_proj = np.arccos(np.clip(cos_angle_from_ref_proj, -1.0, 1.0))
        angle_degrees_from_ref_proj = np.degrees(angle_from_ref_proj)

        # #  feet pos terminate condition
        # next_ref_mocap_pos = self.mocap_pos_com[self.current_frame + 1 + self.random_initial_frame]
        # next_ref_mocap_ori = self.mocap_ori_com[self.current_frame + 1 + self.random_initial_frame]
        # next_mocap_projSRB_origin = next_ref_mocap_pos.copy()
        # next_mocap_projSRB_origin[2] = 0
        # _, next_mocap_projSRB_ori = global2projected_SRB_mocap(next_ref_mocap_ori, next_ref_mocap_pos,
        #                                                        next_ref_mocap_pos)
        # _, curr_projSRB_ori = global2projected_SRB(self.data, self.cdm_id, self.data.qpos[:3].copy(),
        #                                            self.data.qpos[:3].copy())
        # curr_projSRB_origin = self.data.qpos[:3].copy()
        # curr_projSRB_origin[2] = 0
        #
        # left_foot_pos_local = curr_projSRB_ori.T @ (self.data.site_xpos[self.left_site_id]-curr_projSRB_origin)
        # right_foot_pos_local = curr_projSRB_ori.T @ (self.data.site_xpos[self.right_site_id]-curr_projSRB_origin)
        #
        # next_left_foot_pos = self.feet_pos_list[self.current_frame+1+self.random_initial_frame, 0, :]
        # next_left_foot_pos_xy = next_left_foot_pos.copy()
        # next_left_foot_pos_xy[2] = 0
        # next_right_foot_pos = self.feet_pos_list[self.current_frame+1+self.random_initial_frame, 1, :]
        # next_right_foot_pos_xy = next_right_foot_pos.copy()
        # next_right_foot_pos_xy[2] = 0
        # ref_left_foot_pos_local = next_mocap_projSRB_ori.T @ (next_left_foot_pos_xy-next_mocap_projSRB_origin)
        # ref_right_foot_pos_local = next_mocap_projSRB_ori.T @ (next_right_foot_pos_xy-next_mocap_projSRB_origin)
        #
        # left_foot_pos_error = np.linalg.norm((ref_left_foot_pos_local-left_foot_pos_local)[:2])
        # right_foot_pos_error = np.linalg.norm((ref_right_foot_pos_local-right_foot_pos_local)[:2])

        # # Dynamic terminate threshould
        decay_rate = self.current_frame/self.cycle_length
        # if ref_height - height > 0.1+0.2*(1.0-decay_rate) or height - ref_height > 0.1+0.2*(1.0-decay_rate):  # 0.2, 0.3
        velocity_condition = False
        if self.current_frame >= 5:
            #assert(len(self.SRB_pos_list)==self.current_frame+2)
            delta_sim = self.data.qpos[:2]-self.SRB_pos_list[-2-5][:2]
            delta_mocap = self.mocap_pos_com[self.current_frame+1+self.random_initial_frame][:2] - self.mocap_pos_com[self.current_frame+self.random_initial_frame-5][:2]
            if np.linalg.norm(delta_sim) >= 1.5 * np.linalg.norm(delta_mocap):
                velocity_condition = True
        if ref_height - height > 0.2 or height - ref_height > 0.2 or height < 0.50 or height > 1.2:  # 0.2, 0.3
        # if height < 0.50 or height > 1.2:  # 0.2, 0.3
            # print(f'height done {ref_height-height}')
            done = True
        elif angle_degrees > 65:
            # print(f'angle done {angle_degrees}')
            # elif angle_degrees_from_ref > 30:
        # elif angle_degrees_from_ref_proj > 30:
            done = True
        # elif velocity_condition:
        #     done = True
        # elif np.linalg.norm(self.data.qpos[:2] - self.mocap_pos_com[self.current_frame+1+self.random_initial_frame][:2]) >= 2:
        #     done = True
        elif self.current_frame >= 512 - 1:
            done = True

        return done

    def getFullState(self):
        return {
            'current_frame': self.current_frame,
            'qpos': self.data.qpos.copy(),
            'qvel': self.data.qvel.copy(),
            'left_foot_pos': self.data.site_xpos[self.left_site_id].copy(),
            'right_foot_pos': self.data.site_xpos[self.right_site_id].copy(),
            'left_foot_mat': self.data.site_xmat[self.left_site_id].reshape(3, 3).copy(),
            'right_foot_mat': self.data.site_xmat[self.right_site_id].reshape(3, 3).copy(),
            'left_foot_in_contact': self.left_foot_in_contact,
            'right_foot_in_contact': self.right_foot_in_contact,
            'SRB_pos_list': self.SRB_pos_list.copy(),
            'SRB_ori_list': self.SRB_ori_list.copy(),
            'simulated_feet_pos_list': self.simulated_feet_pos_list.copy(),
            'simulated_feet_ori_list': self.simulated_feet_ori_list.copy(),
            'contact_signal_list': self.contact_signal_list.copy()
        }

    def restoreFullState(self, FullState):
        self.current_frame = FullState["current_frame"]
        self.data.qpos = FullState["qpos"]
        self.data.qvel = FullState["qvel"]
        self.left_foot_in_contact = FullState["left_foot_in_contact"]
        self.right_foot_in_contact = FullState["right_foot_in_contact"]
        self.SRB_pos_list = FullState['SRB_pos_list']
        self.SRB_ori_list = FullState['SRB_ori_list']
        self.simulated_feet_pos_list = FullState['simulated_feet_pos_list']
        self.simulated_feet_ori_list = FullState['simulated_feet_ori_list']
        self.contact_signal_list = FullState['contact_signal_list']

        self.left_foot_contact_pos_global = FullState["left_foot_pos"]
        self.right_foot_contact_pos_global = FullState["right_foot_pos"]
        self.left_foot_contact_ori = FullState["left_foot_mat"]
        self.right_foot_contact_ori = FullState["right_foot_mat"]
        curr_feet_contact = np.stack((FullState["left_foot_pos"], FullState["right_foot_pos"]))
        if not self.left_foot_in_contact:
            curr_feet_contact[0, 2] = np.nan
        else:
            curr_feet_contact[0, 2] = 0
        if not self.right_foot_in_contact:
            curr_feet_contact[1, 2] = np.nan
        else:
            curr_feet_contact[1, 2] = 0
        curr_feet_ori = np.stack((FullState["left_foot_mat"], FullState["right_foot_mat"]))
        set_feet_site_this_step_env(self.model, self.data, curr_feet_contact, curr_feet_ori,
                                    FullState["left_foot_pos"], FullState["right_foot_pos"])
        mujoco.mj_forward(self.model, self.data)
        set_feet_site_this_step_env(self.model, self.data, curr_feet_contact, curr_feet_ori,
                                    FullState["left_foot_pos"], FullState["right_foot_pos"])

    def get_obs_window(self, window_size: int) -> np.ndarray:
        assert window_size >= 1, "window size must be >= 1"
        least_episode_length = max(self.cycle_length - 2, 512)
        assert self.current_frame + window_size < least_episode_length * 2, f'will pass upper cycle length limit in {window_size}frames later'

        curr_height_terrain, _ = terrain_height_ray(self.model, self.data, self.data.qpos[0], self.data.qpos[1])
        SRB_pos_proj, projSRB_ori = global2projected_SRB(self.data, self.cdm_id, self.data.qpos[:3].copy(),
                                                         self.data.qpos[:3].copy())
        SRB_pos_proj[2] -= curr_height_terrain
        # height
        height_proj = np.array([SRB_pos_proj[2]])

        # quaternion
        world_mat = self.data.xmat[self.cdm_id].reshape(3, 3).copy()
        projSRB_mat = projSRB_ori.T @ world_mat
        # projSRB_mat_6D = convert_matrix_to_6d(projSRB_mat)
        projSRB_mat_6D = projSRB_mat[:, :2].flatten(order='F')
        # projSRB_quat = R.from_matrix(projSRB_mat).as_quat()

        # generalized velocity
        local_vel_lin = projSRB_ori.T @ self.data.qvel[:3].copy()
        local_vel_ang = projSRB_ori.T @ (world_mat @ self.data.qvel[3:].copy())

        # left foot 3D pos and 1D ori against vertical axis
        left_foot_pos_global = self.data.site_xpos[self.left_site_id]
        left_foot_pos_ff, ffSRB_ori = global2forward_facing_SRB(self.data, self.cdm_id, self.data.qpos[:3].copy(),
                                                                left_foot_pos_global)
        left_foot_mat = self.data.site_xmat[self.left_site_id].reshape(3, 3)
        left_foot_mat_ff = ffSRB_ori.T @ left_foot_mat
        left_foot_angle_vertical = np.array([np.arctan2(left_foot_mat_ff[1, 0], left_foot_mat_ff[0, 0])])

        # right foot 3D pos and 1D ori against vertical axis
        right_foot_pos_global = self.data.site_xpos[self.right_site_id]
        right_foot_pos_ff, _ = global2forward_facing_SRB(self.data, self.cdm_id, self.data.qpos[:3].copy(),
                                                         right_foot_pos_global)
        right_foot_mat = self.data.site_xmat[self.right_site_id].reshape(3, 3)
        right_foot_mat_ff = ffSRB_ori.T @ right_foot_mat
        right_foot_angle_vertical = np.array([np.arctan2(right_foot_mat_ff[1, 0], left_foot_mat_ff[0, 0])])

        foot_in_contact = [self.left_foot_in_contact, self.right_foot_in_contact]
        contact_signal = foot_contact_to_one_hot(foot_in_contact)

        window_obs = []
        for window_id in range(1, window_size + 1):
            # next mocap obs
            next_mocap_height_terrain = self.mocap_height_com_terrain[self.current_frame + window_id + self.random_initial_frame]
            next_ref_mocap_pos = self.mocap_pos_com[self.current_frame + window_id + self.random_initial_frame].copy()
            next_ref_mocap_ori = self.mocap_ori_com[self.current_frame + window_id + self.random_initial_frame]
            next_ref_mocap_linvel = self.mocap_vel_com[self.current_frame + window_id + self.random_initial_frame, :3]
            next_ref_mocap_angvel = self.mocap_vel_com[self.current_frame + window_id + self.random_initial_frame, 3:]

            next_SRB_pos_proj, next_projSRB_ori = global2projected_SRB_mocap(next_ref_mocap_ori, next_ref_mocap_pos,
                                                                             next_ref_mocap_pos)
            # next_SRB_pos_global_xy = np.array(next_ref_mocap_pos[:2])
            # next_SRB_pos_relative2current_step_xy = next_SRB_pos_global_xy - self.data.qpos[:2].copy()
            next_SRB_pos_proj[2] -= next_mocap_height_terrain
            next_height_proj = np.array([next_SRB_pos_proj[2]])

            # next_pos_proj = projSRB_ori.T @ (next_ref_mocap_pos-self.data.qpos[:3].copy())
            # next_projSRB_mat = projSRB_ori.T @ next_ref_mocap_ori

            next_ref_mocap_pos_plane = next_ref_mocap_pos.copy()
            next_ref_mocap_pos_plane[2] = 0
            # next_pos = next_projSRB_ori.T@(next_ref_mocap_pos - next_ref_mocap_pos_plane)
            next_mat = (next_projSRB_ori.T@next_ref_mocap_ori)[:, :2].flatten(order='F')
            # next_projSRB_mat_6D = convert_matrix_to_6d(next_projSRB_mat)
            # next_projSRB_mat_6D = next_projSRB_mat[:, :2].flatten(order='F')
            # next_vel_lin_proj = projSRB_ori.T @ next_ref_mocap_linvel
            # next_vel_ang_proj = projSRB_ori.T @ (next_ref_mocap_ori @ next_ref_mocap_angvel)

            next_vel_lin = next_projSRB_ori.T @ next_ref_mocap_linvel
            next_vel_ang = next_projSRB_ori.T @ (next_ref_mocap_ori @ next_ref_mocap_angvel)

            next_left_foot_pos_global_planar = self.feet_pos_list_original[self.current_frame + window_id + self.random_initial_frame, 0, :]
            next_left_foot_pos_global = self.feet_pos_list[self.current_frame + window_id + self.random_initial_frame, 0, :]
            next_left_foot_pos_xy = next_left_foot_pos_global.copy()
            next_left_foot_pos_xy[2], _ = terrain_height_ray(self.model, self.data, next_left_foot_pos_xy[0], next_left_foot_pos_xy[1])
            if next_left_foot_pos_global_planar[2] == 0:
                next_left_foot_in_contact = True
            else:
                next_left_foot_in_contact = False
            next_left_foot_pos_ff, next_ffSRB_ori = global2forward_facing_SRB_mocap(next_ref_mocap_ori, next_ref_mocap_pos,
                                                                                    next_left_foot_pos_xy)
            next_left_foot_mat = self.feet_ori_list[self.current_frame + window_id + self.random_initial_frame][0]
            next_left_foot_mat_ff = next_ffSRB_ori.T @ next_left_foot_mat
            next_left_foot_angle_vertical = np.array([np.arctan2(next_left_foot_mat_ff[1, 0], next_left_foot_mat_ff[0, 0])])

            next_right_foot_pos_global_planar = self.feet_pos_list_original[self.current_frame + window_id + self.random_initial_frame, 1, :]
            next_right_foot_pos_global = self.feet_pos_list[self.current_frame + window_id + self.random_initial_frame, 1, :]
            next_right_foot_pos_xy = next_right_foot_pos_global.copy()
            next_right_foot_pos_xy[2], _ = terrain_height_ray(self.model, self.data, next_right_foot_pos_xy[0], next_right_foot_pos_xy[1])
            if next_right_foot_pos_global_planar[2] == 0:
                next_right_foot_in_contact = True
            else:
                next_right_foot_in_contact = False
            next_right_foot_pos_ff, _ = global2forward_facing_SRB_mocap(next_ref_mocap_ori, next_ref_mocap_pos,
                                                                        next_right_foot_pos_xy)
            next_right_foot_mat = self.feet_ori_list[self.current_frame + window_id + self.random_initial_frame][1]
            next_right_foot_mat_ff = next_ffSRB_ori.T @ next_right_foot_mat
            next_right_foot_angle_vertical = np.array(
                [np.arctan2(next_right_foot_mat_ff[1, 0], next_right_foot_mat_ff[0, 0])])

            next_foot_in_contact = [next_left_foot_in_contact, next_right_foot_in_contact]
            next_contact_signal = foot_contact_to_one_hot(next_foot_in_contact)

            next_mocap_obs = np.concatenate(
                [next_height_proj, next_mat, next_vel_lin, next_vel_ang,
                 next_left_foot_pos_ff, next_left_foot_angle_vertical, next_right_foot_pos_ff, next_right_foot_angle_vertical,
                 next_contact_signal], dtype=np.float32)
            window_obs.append(next_mocap_obs)
        window_mocap_obs = np.concatenate(window_obs, axis=-1)

        obs = np.concatenate([height_proj, projSRB_mat_6D, local_vel_lin, local_vel_ang, left_foot_pos_ff,
                              left_foot_angle_vertical, right_foot_pos_ff, right_foot_angle_vertical, contact_signal,
                              window_mocap_obs], dtype=np.float32)
        return obs


    def loadMocap(self, i, initial_pos_planar=np.array([0.0, 0.0])):
        if self.SRB_data_list[i]!=None:
            return
        mocap_path=self.mocap_path_list[i]
        SRBdata = SRBDataProcess(mocap_path)
        mocap_dt = SRBdata.dt
        initial_frame = 0  # 96 walk  # 225 jump
        last_frame = None  # 952 walk  # 929 jump
        SRBdata.mocap_data[:, :2] += initial_pos_planar
        SRBdata.humanoid2SRB(initial_frame=initial_frame, last_frame=last_frame)
        cycle_length = SRBdata.cycle_length
        mocap_qpos = SRBdata.mocap_com_humanoid
        mocap_pos_com = SRBdata.mocap_com_humanoid[:, :3]
        mocap_ori_com = SRBdata.mocap_ori_com  # rotation matrix
        mocap_vel_com = SRBdata.mocap_vel_com  # generalized velocity
        feet_pos_list = SRBdata.feet_pos_list
        feet_ori_list = SRBdata.feet_ori_list
        feet_ori_list_original = SRBdata.feet_pos_list_original
        mocap_height_com_terrain = SRBdata.mocap_height_com_terrain
        self.SRB_data_list[i]={
            'SRBdata': SRBdata,
            'mocap_dt': mocap_dt,
            'cycle_length': cycle_length,
            'mocap_qpos': mocap_qpos,
            'mocap_pos_com': mocap_pos_com,
            'mocap_ori_com': mocap_ori_com,
            'mocap_vel_com': mocap_vel_com,
            'feet_pos_list': feet_pos_list,
            'feet_ori_list': feet_ori_list,
            'feet_pos_list_original': feet_ori_list_original,
            'mocap_height_com_terrain': mocap_height_com_terrain
        }
    def getCurrentGlobalFootPos(self, index):
        if index==0:
            return self.left_foot_contact_pos_global
        return self.right_foot_contact_pos_global
    def setCurrentGlobalFootPos(self, index, pos):
        if index==0:
            self.left_foot_contact_pos_global=pos
        else:
            self.right_foot_contact_pos_global=pos

    # after reset
    def initFilter(self, index, action_ori, action_pos, ffSRB_ori, ffSRB_origin):
        self.contactFilter[index]=action_ori

        self.useLocalPosFilter=False

        if self.useLocalPosFilter:
            # local filter
            self.contactFilter[index+2]=action_pos.copy()

        else:
            # global filter
            desired_foot_pos_ff= np.concatenate([action_pos, np.array([0])])
            desired_foot_pos_global=ffSRB_ori @ desired_foot_pos_ff + ffSRB_origin

            B=0.001
            k=1

            logQ=9
            Q=math.pow(10, logQ)
            ballX=control.SDS(60.0, B, k, Q, 0, 1/120.0,True) # timestep is 1/120 because I am using 4 substeps for this.
            ballY=control.SDS(60.0, B, k, Q, 0, 1/120.0,True)
            ballX.x.set(0, 0, desired_foot_pos_global[0])
            ballX.xd.set(0, 0, desired_foot_pos_global[0])
            ballY.x.set(0, 0, desired_foot_pos_global[1])
            ballY.xd.set(0, 0, desired_foot_pos_global[1])
            self.contactFilter[index+2]=[ballX, ballY]


    # swing phase
    def filterFoot(self, index, action_ori, action_pos, ffSRB_ori, ffSRB_origin):
        contactFilterAlpha=0.5
        #contactFilterAlpha=0 # to turn the filtering off
        self.contactFilter[index]=self.contactFilter[index]*contactFilterAlpha+action_ori*(1.0-contactFilterAlpha)
        #if(index==0): print('filter', self.contactFilter[index], action_ori)

        if self.useLocalPosFilter: # for feet positions, global filter looks much better.
            #local filter
            self.contactFilter[index+2]=self.contactFilter[index+2]*contactFilterAlpha+action_pos*(1.0-contactFilterAlpha)
            desired_foot_pos_ff = np.concatenate([self.contactFilter[index+2], np.array([0])])
            self.setCurrentGlobalFootPos(index, ffSRB_ori @ desired_foot_pos_ff + ffSRB_origin)

        else:
            #global filter (which provides the impression of heavy legs, but makes the control problem significantly more difficult)
            desired_foot_pos_ff= np.concatenate([action_pos, np.array([0])])
            desired_foot_pos_global=ffSRB_ori @ desired_foot_pos_ff + ffSRB_origin
            ballX, ballY=self.contactFilter[index+2]
            # desired position
            ballX.xd.set(0, 0, desired_foot_pos_global[0])
            ballY.xd.set(0, 0, desired_foot_pos_global[1])

            B=0.001
            k=1
            curr_com_lin_speed = math.sqrt(self.data.qvel[:2].dot(self.data.qvel[:2]))
            #import work.rendermodule as RE
            #RE.output(f'contactFilter{index}', self.contactFilter[index], action_ori, curr_com_lin_speed)


            logQ=None
            if curr_com_lin_speed<3:
                logQ=9
            elif curr_com_lin_speed>6:
                logQ=11
            else:
                logQ=9.0+((curr_com_lin_speed-3.0)/3.0)*(11.0-9.0)

            logQ=max(self.minLogQ, logQ)
            assert(logQ>=5 and logQ<=11.000001) # only this range is supported on windows.
            Q=math.pow(10, logQ) 
            ballX.updateCoef(B, k,Q)
            ballY.updateCoef(B, k,Q)
            for i in range(4):
                ballX.singleStep()
                ballY.singleStep()


            filtered_foot_pos_global=np.array([ballX.x(0,0), ballY.x(0,0), 0.0])
            self.setCurrentGlobalFootPos(index, filtered_foot_pos_global)


    # support phase
    def updateFilter(self, index, foot_contact_ori, ffSRB_ori, foot_contact_pos_global, ffSRB_origin):
        # local filter for orientation
        #if(index==0): print('filter', self.contactFilter[index])

        if not self.useLocalPosFilter:
            self.contactFilter[index]=angle_between_vectors(foot_contact_ori[0,:], ffSRB_ori[0,:], np.array([0.,0.,1.0]))
            # set zero vel.
            self.contactFilter[index+2][0].x.set(1,0,0)
            self.contactFilter[index+2][1].x.set(1,0,0)
            self.contactFilter[index+2][0].xd.set(1,0,0)
            self.contactFilter[index+2][1].xd.set(1,0,0)


def angle_between_vectors(v1, v2, normal):
    # Normalize input vectors
    v1 /= np.sqrt(v1.dot(v1))
    v2 /= np.sqrt(v2.dot(v2))
    normal /= np.sqrt(normal.dot(normal))

    # Compute angle using atan2
    cross = np.cross(v1, v2)
    dot = np.dot(v1, v2)
    det = np.dot(normal, cross)  # projection of cross product on normal
    angle = np.arctan2(det, dot)  # signed angle
    return angle
