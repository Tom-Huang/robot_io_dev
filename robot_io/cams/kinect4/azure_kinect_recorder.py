# ----------------------------------------------------------------------------
# -                        Open3D: www.open3d.org                            -
# ----------------------------------------------------------------------------
# The MIT License (MIT)
#
# Copyright (c) 2018-2021 www.open3d.org
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
# IN THE SOFTWARE.
# ----------------------------------------------------------------------------

# examples/python/reconstruction_system/sensors/azure_kinect_recorder.py

import argparse
import datetime
import open3d as o3d
import io
import os
import json
from omegaconf import OmegaConf
from contextlib import redirect_stdout


class RecorderWithCallback:

    def __init__(self, config, device, filename, align_depth_to_color):
        # Global flags
        self.flag_exit = False
        self.flag_record = False
        self.filename = filename
        self.device = device
        self.align_depth_to_color = align_depth_to_color
 
        self.recorder = o3d.io.AzureKinectRecorder(config, device)
        f = io.StringIO()
        with redirect_stdout(f):
            if not self.recorder.init_sensor():
                raise RuntimeError('Failed to connect to sensor')
        device_info = f.getvalue()
        self.serial_number = self.get_kinect_serial_number(device_info)
        print(self.serial_number)

    def escape_callback(self, vis):
        self.flag_exit = True
        if self.recorder.is_record_created():
            print('Recording finished.')
        else:
            print('Nothing has been recorded.')
        return False

    def space_callback(self, vis):
        if self.flag_record:
            print('Recording paused. '
                  'Press [Space] to continue. '
                  'Press [ESC] to save and exit.')
            self.flag_record = False

        elif not self.recorder.is_record_created():
            if self.recorder.open_record(self.filename):
                print('Recording started. '
                      'Press [SPACE] to pause. '
                      'Press [ESC] to save and exit.')
                self.flag_record = True

        else:
            print('Recording resumed, video may be discontinuous. '
                  'Press [SPACE] to pause. '
                  'Press [ESC] to save and exit.')
            self.flag_record = True

        return False

    def run(self):
        glfw_key_escape = 256
        glfw_key_space = 32
        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.register_key_callback(glfw_key_escape, self.escape_callback)
        vis.register_key_callback(glfw_key_space, self.space_callback)

        vis.create_window('recorder', 1920, 540)
        print("Recorder initialized. Press [SPACE] to start. "
              "Press [ESC] to save and exit.")

        vis_geometry_added = False
        while not self.flag_exit:
            rgbd = self.recorder.record_frame(self.flag_record,
                                              self.align_depth_to_color)
            if rgbd is None:
                continue

            if not vis_geometry_added:
                vis.add_geometry(rgbd)
                vis_geometry_added = True

            vis.update_geometry(rgbd)
            vis.poll_events()
            vis.update_renderer()

        self.recorder.close_record()
        
    def get_kinect_serial_number(self, device_info):
        """Get the serial number of the kinect device"""
        serial_number = [int(s) for s in device_info.splitlines()[2].split() if s.isdigit()][0]
        return serial_number

    #create a function that saves a file with n frames
    def save_n_frames(self, n):
        #start the recording 
        if self.recorder.open_record(self.filename):
                print('Recording started. '
                      'Press [SPACE] to pause. '
                      'Press [ESC] to save and exit.')
                self.flag_record = True
        #save n frames
        for i in range(n):
            rgbd = self.recorder.record_frame(self.flag_record,
                                              self.align_depth_to_color)
            if rgbd is None:
                continue
        
        if self.recorder.is_record_created():
            print('Recording finished.')
        else:
            print('Nothing has been recorded.')
        #save the file
        self.recorder.close_record()

    def get_intrinsics(self):
        print("Getting intrinsics ...")
        self.save_n_frames(3)
        reader = o3d.io.AzureKinectMKVReader()
        reader.open(self.filename)
        if not reader.is_opened():
            raise RuntimeError("Unable to open file {}".format(self.filename))
        metadata = reader.get_metadata()
        o3d.io.write_azure_kinect_mkv_metadata(
                f'config/{self.serial_number}.json',metadata)

        #delete mkv file
        os.remove(self.filename)
        # load json file and convert it to an omegaconf yaml
        with open(f'config/{self.serial_number}.json') as f:
            data = json.load(f)

        
        #convert to omegaconf
        cfg = OmegaConf.create(data)
        #create an empty cfg
        cfg_new = OmegaConf.create({f"{cfg.height}p": {}})
        # update the intrinisics from list to a dictionary with cx, cy, fx, fy, width, height
        cfg_new[f"{cfg.height}p"].intrinsics = {'fx': cfg.intrinsic_matrix[0],
         'fy': cfg.intrinsic_matrix[4], 
         'cx': cfg.intrinsic_matrix[6], 
         'cy': cfg.intrinsic_matrix[7], 
         'width': cfg.width,
          'height': cfg.height}
        
        # get config file if exists
        if os.path.exists(f'config/{self.serial_number}.yaml'):
            cfg_old = OmegaConf.load(f'config/{self.serial_number}.yaml')
            cfg_new = OmegaConf.merge(cfg_old, cfg_new)
            
        OmegaConf.save(cfg_new, f'config/{self.serial_number}.yaml')
        #delete json file
        os.remove(f'config/{self.serial_number}.json')



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Azure kinect mkv recorder.')
    parser.add_argument('--config', type=str, help='input json kinect config')
    parser.add_argument('--output', type=str, help='output mkv filename')
    parser.add_argument( '-i', "--intrinsics", action="store_true", help="save intrinsic yaml file")
    parser.add_argument('--list',
                        action='store_true',
                        help='list available azure kinect sensors')
    parser.add_argument('--device',
                        type=int,
                        default=1,
                        help='input kinect device id')
    parser.add_argument('-a',
                        '--align_depth_to_color',
                        action='store_true',
                        help='enable align depth image to color')

    # add parser for number of frames to save
    parser.add_argument('--n_frames', type=int, help='number of frames to save')
    args = parser.parse_args()

    if args.list:
        o3d.io.AzureKinectSensor.list_devices()
        exit()

    if args.config is not None:
        config = o3d.io.read_azure_kinect_sensor_config(args.config)
    else:
        config = o3d.io.AzureKinectSensorConfig()

    if args.output is not None:
        filename = args.output
    else:
        filename = '{date:%Y-%m-%d-%H-%M-%S}.mkv'.format(
            date=datetime.datetime.now())
    print('Prepare writing to {}'.format(filename))

    device = args.device
    if device < 0 or device > 255:
        print('Unsupported device id, fall back to 0')
        device = 0

    r = RecorderWithCallback(config, device, filename,
                             args.align_depth_to_color)

    
    # if n_frames is specified, save n_frames
    if args.n_frames is not None:
        r.save_n_frames(args.n_frames)
    elif args.intrinsics is not None:
        r.get_intrinsics()
    else:
        r.run()