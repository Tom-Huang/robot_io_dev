import json
from omegaconf import OmegaConf
import open3d as o3d
import numpy as np
import cv2
import time
import threading
from pathlib import Path

from robot_io.cams.camera import Camera
from contextlib import redirect_stdout
import io
import argparse


class Kinect4(Camera):
    def __init__(self,
                 device=0,
                 align_depth_to_color=True,
                 resolution='1080p',
                 undistort_image=True,
                 resize_resolution=None,
                 crop_coords=None,
                 fps=30):


        super().__init__(resolution=resolution, crop_coords=crop_coords, resize_resolution=resize_resolution,
                         name="azure_kinect")
        config_path = "config/config_kinect4_{}.json".format(resolution)
        config = self.load_config_path(config_path)
        print("device is: ", device)
        self.sensor = o3d.io.AzureKinectSensor(config)
        f = io.StringIO()
        with redirect_stdout(f):
            if not self.sensor.connect(device):
                raise RuntimeError('Failed to connect to sensor')
        device_info = f.getvalue()
        self.serial_number = self.get_kinect_serial_number(device_info)
        data = self.load_config_data(self.serial_number, resolution)
        

        #data = self.load_config_data(self.serial_number, resolution)
        self.resize_resolution = resize_resolution

        self.intrinsics.update({"crop_coords": self.crop_coords,
                                "resize_resolution": self.resize_resolution,
                                "dist_coeffs": self.dist_coeffs})

        self.map1, self.map2 = cv2.initUndistortRectifyMap(self.camera_matrix, self.dist_coeffs, R=np.eye(3), \
                                                           newCameraMatrix=self.camera_matrix,
                                                           size=(self.intrinsics['width'], self.intrinsics['height']),
                                                           m1type=cv2.CV_16SC2)
        self.device = device
        self.align_depth_to_color = align_depth_to_color
        self.config_path = config_path
        self.undistort_image = undistort_image
        self.fps = fps
        self.align_depth_to_color = align_depth_to_color

    def load_config_path(self, config_path):
        if config_path is not None:
            full_path = (Path(__file__).parent / config_path).as_posix()
            config = o3d.io.read_azure_kinect_sensor_config(full_path)
        else:
            config = o3d.io.AzureKinectSensorConfig()

        return config

    def get_kinect_serial_number(self, device_info):
        """Get the serial number of the kinect device"""
        serial_number = [int(s) for s in device_info.splitlines()[2].split() if s.isdigit()][0]
        return serial_number

    def load_config_data(self, serial_number, resolution='1080p'):
        """load the calibration parameters for the kinect device from yaml file"""
        config_path = f"config/{serial_number}.yaml"
        full_path = (Path(__file__).parent / config_path).as_posix()
        conf = OmegaConf.to_container(OmegaConf.load(full_path)[resolution])
        self.intrinsics = conf["intrinsics"]
        self.resolution = (self.intrinsics["width"], self.intrinsics["height"])
        # create the camera matrix from the intrinsics
        self.camera_matrix = np.array([[self.intrinsics["fx"], 0, self.intrinsics["cx"]],
                                  [0, self.intrinsics["fy"], self.intrinsics["cy"]],
                                  [0, 0, 1]])

        self.projection_matrix = np.array([[self.intrinsics["fx"], 0, self.intrinsics["cx"], 0],
                                      [0, self.intrinsics["fy"], self.intrinsics["cy"], 0],
                                      [0, 0, 1, 0]])

        #convert the distortion coefficients to a numpy array
        self.dist_coeffs = np.array(conf["dist_coeffs"])
        self.crop_coords = None
        if "crop_coords" in conf:
            self.crop_coords = np.array(conf["crop_coords"])

        # create the data dictionary
        data = {'camera_matrix': self.camera_matrix,
                'projection_matrix': self.projection_matrix,
                'dist_coeffs': self.dist_coeffs,
               'crop_coords': self.crop_coords,
                'resolution': resolution,
                'intrinsics': self.intrinsics}

        return data

    def get_intrinsics(self):
        return self.intrinsics

    def get_projection_matrix(self):
        return self.projection_matrix

    def get_camera_matrix(self):
        return self.camera_matrix

    def get_dist_coeffs(self):
        return self.dist_coeffs

    def _get_image(self):
        rgbd = None
        while rgbd is None:
            rgbd = self.sensor.capture_frame(self.align_depth_to_color)
        rgb = np.asarray(rgbd.color)
        depth = (np.asarray(rgbd.depth)).astype(np.float32) / 1000
        if self.undistort_image:
            rgb = cv2.remap(rgb, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)
            depth = cv2.remap(depth, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)

        return rgb, depth


def run_camera(device = 0, resolution = '1080p'):
    cam = Kinect4(device=device, resolution=resolution)
    print(cam.get_intrinsics())
    while True:
        rgb, depth = cam.get_image()
        cv2.imshow("depth", depth)
        cv2.imshow("rgb", rgb[:, :, ::-1])
        cv2.waitKey(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--device', type=int, default=1, help='device number')
    parser.add_argument('--resolution', type=str, default='1080p', help='resolution of the camera')
    args = parser.parse_args()
    run_camera(device=args.device, resolution=args.resolution)
