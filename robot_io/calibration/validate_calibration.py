# define a function that reads the teleop_data and draws the robot_tcp in the camera frame
# 
import numpy as np
import time
import cv2
import hydra
import numpy as np
import os
import pathlib
from numpy import dot, eye, zeros, outer
from numpy.linalg import inv
import itertools
from datetime import datetime
from scipy.optimize import least_squares
from robot_io.utils.utils import pos_orn_to_matrix, matrix_to_pos_orn
from scipy.spatial.transform.rotation import Rotation as R

import matplotlib.pyplot as plt

map = ["x", "y", "z"]


def merge_dicts(dicts):
    merged = {}
    for d in dicts:
        for k, v in d.items():
            if isinstance(v, tuple):
                # convert v to a dict
                v = {'x': v[0], 'y': v[1], 'z': v[2]}

            if isinstance(v, dict):
                if k not in merged:
                    merged[k] = {}
                merged[k] = merge_dicts([merged[k], v])
            else:
                if k not in merged:
                    merged[k] = []
                merged[k].append(v)
    return merged


def plot_coordinate_frame(image, cam_matrix, frame, length=0.1, thickness=2, wait = True):
    pose = cam_matrix @ frame
    # rescale the pose
    # Extract the rotation and translation from the 4x4 pose matrix
    rotation = pose[:3, :3]
    translation = pose[:3, 3]

    # scale the axes and do a translation to get end points
    rotation = rotation * length + translation[:, None]

    start = translation[:2]/translation[2]
    end = rotation[:2]/rotation[2]
    # Convert the start and end points to integers
    x_start, y_start = start.astype(int)
    x_x_end, x_y_end = end[:, 0].astype(int)
    y_x_end, y_y_end = end[:, 1].astype(int)
    z_x_end, z_y_end = end[:, 2].astype(int)

    # Plot the x-axis in red
    cv2.line(image, (x_start, y_start), (x_x_end, x_y_end), (0, 0, 255), thickness)

    # Plot the y-axis in green
    cv2.line(image, (x_start, y_start), (y_x_end, y_y_end), (0, 255, 0), thickness)

    # Plot the z-axis in blue
    cv2.line(image, (x_start, y_start), (z_x_end, z_y_end), (255, 0, 0), thickness)
    cv2.imshow("object in cam", image[:, :, ::-1])
    cv2.waitKey(0)
    return image

# create a data wrapper class fom the teleop_data
class TeleopData:
    def __init__(self, directory):

        self.directory = directory
        self.frames_paths = sorted(pathlib.Path(self.directory).expanduser().glob('frame_*'))
        self.ep_start_end_ids = None
        self.episodes = None

        # load the camera_info 
        self.get_camera_info()
        # load the teleop_data 
        self.get_teleop_data()

    def build_intrinsics(self, intrinsics):
        # build the intrinisic matrix for each camera

        # create the camera matrix from the intrinsics
        intrinsics["camera_matrix"] = np.array([[intrinsics["fx"], 0, intrinsics["cx"]],
                                                [0, intrinsics["fy"], intrinsics["cy"]],
                                                [0, 0, 1]])

        # add the projection matrix from camera matrix
        intrinsics["projection_matrix"] = np.concatenate((intrinsics["camera_matrix"], np.zeros((3, 1))), axis=1)

        return intrinsics
        # repeat for the dynamic camera

    def get_camera_info(self):
        self.cam_info = np.load(self.directory + '/camera_info.npz', allow_pickle=True)
        # convert from files to a dict 
        self.cam_info = dict(self.cam_info)
        # build the intrinisic matrix for each camera
        static_intrinsic = self.cam_info["static_intrinsics"]
        self.cam_info["static_intrinsics"] = self.build_intrinsics(static_intrinsic[None][0])
        # create the camera matrix from the intrinsics

        # repeat for the dynamic camera
        #dynamic_intrinsic = self.cam_info["gripper_intrinsics"]
        #self.cam_info["gripper_intrinsics"] = self.build_intrinsics(dynamic_intrinsic[None][0])

        return self.cam_info

    def get_teleop_data(self):
        # load the ep_start_end_ids 
        self.ep_start_end_ids = np.load(self.directory + '/ep_start_end_ids.npy', allow_pickle=True)
        #self.ep_start_end_ids = [(0, 507), ]
        # split the frames into episodes
        self.episodes = [self.frames_paths[start:end] for start, end in self.ep_start_end_ids]
        # load the episodes from npz files
        episodes = []
        for ep in self.episodes:
            ep_processed = []
            for i, path in enumerate(ep):
                # skip in valid data
                try:
                    ep_processed.append(dict(np.load(str(path), allow_pickle=True)))
                except:
                    print("skipping invalid data", path, i)
                    continue
            episodes.append(ep_processed)
        self.episodes = episodes
        # convert the episodes to a dict of arrays
        episodes = []
        for ep in self.episodes:
            episode = {}
            for k in ep[0]:
                if k == 'robot_state':
                    if k not in episode:
                        episode[k] = {}

                    for k2 in ep[0][k][None][0]:
                        episode[k][k2] = np.array([d[k][None][0][k2] for d in ep])

                elif k == "action":
                    if k not in episode:
                        episode[k] = {}

                    for k2 in ep[0][k][None][0]:
                        if k2 == "motion":
                            if k2 not in episode[k]:
                                episode[k][k2] = {}
                            for i, k3 in enumerate(map):
                                episode[k][k2][k3] = np.array([d[k][None][0][k2][i] for d in ep])
                        else:
                            episode[k][k2] = np.array([d[k][None][0][k2] for d in ep])
                else:
                    episode[k] = np.array([d[k] for d in ep])
            episodes.append(episode)
        self.episodes = episodes

def main():
    # create a data wrapper class fom the teleop_data
    teleop_data = TeleopData('robot_io/teleop_data/2023-02-04/21-03-15')
    print(teleop_data.cam_info)

    # find timestep with the minimum 'gripper_opening_width'
    min_gripper_opening_width = np.argmin(teleop_data.episodes[0]["robot_state"]['gripper_opening_width'])
    #min_gripper_opening_width = 450
    # show image at that timestep
    # loop over multiple timesteps
    for i in range(0, 500, 50):

        rgb = teleop_data.episodes[0]['rgb_static'][min_gripper_opening_width]
        # get the gripper pose at that timestep
        gripper_pos = teleop_data.episodes[0]['robot_state']['tcp_pos'][min_gripper_opening_width]
        gripper_orn = teleop_data.episodes[0]['robot_state']['tcp_orn'][min_gripper_opening_width]
        # create gripper pose matrix
        gripper_pose = np.eye(4)
        gripper_pose[:3, :3] = R.from_quat(gripper_orn).as_matrix()
        gripper_pose[:3, 3] = gripper_pos

        # get the static camera calibration
        static_intrinsics = teleop_data.cam_info['static_intrinsics']['projection_matrix']
        # get the dynamic camera calibration
        static_extrinsics = teleop_data.cam_info['static_extrinsic_calibration']
        # get the gripper pose in the static camera frame
        cam_matrix = static_intrinsics @ np.linalg.inv(static_extrinsics)
        rgb = plot_coordinate_frame(rgb, cam_matrix, gripper_pose, length=0.1, thickness=3)
        # save the image
        cv2.imwrite(f'rgb_{min_gripper_opening_width}.png', rgb)
        min_gripper_opening_width = i


# write the main function
if __name__ == "__main__":
    main()
