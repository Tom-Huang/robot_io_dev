# Camera Calibration

### Kinect Azure Camera Intrinsics
- Clone the Kinect Azure official SDK.
    ```bash
    git clone https://github.com/microsoft/Azure-Kinect-Sensor-SDK.git
    cd Azure-Kinect-Sensor-SDK
    ```
- Build the Kinect Azure official SDK. Before building, you need to modify `Azure-Kinect-Sensor-SDK/examples/calibration/main.cpp` Line 62 to `deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_720P;` or `deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_1080P;`(depending on the resolution of the intrinsics you need) and Line 76 to `auto calib = calibration.color_camera_calibration;`. 
    ```bash
    # under Azure-Kinect-Sensor-SDK folder
    mkdir build && cd build && cmake ..
    make
    ```
- Run the `./calibration_info`.
    ```
    cd bin
    ./calibration_info
    ```
- Create a `.yaml` file under the `robot_io/cams/kinect4/config/{cam_serial_number}.yaml` and fill in intrinsics number following template of the existing files there.
### Static Camera
- Get the dist coeff parameters through the camera calibration script in the [Azure Kinect SDK](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/tree/develop/examples/calibration)
- Get the intrinsics through running the viewer in the kinect4 directory
- Add parameters to the a config file with the corressponding serial number in `robot_io/cams/kinect4/config`
- Stick the marker to the robot end-effector
- Run `python robot_io/calibration/static_cam_calibration.py --config-name=[panda_calibrate_static_cam|iiwa_calibrate_static_cam]`
- If you set `record_traj: true`, then you should use vr controller to move the robot. Press the record button (on top) to sample poses, and hold record button to finish the pose sampling.
- If you set `record_traj: false` and `play_traj: true`, the robot will move to the previously recorded poses and captures the marker pose. This option is helpful in case the camera is moved slightly.


### Gripper Camera
- Place Aruco Marker in front of robot
- Run `python robot_io/calibration/gripper_cam_calibration.py --config-name=[panda_calibrate_gripper_cam|kuka_calibrate_gripper_cam]`

------------------

### Teleoperation
Make sure to set workspace limits appropriately in `robot_io/conf/robot/<robot_interface.yaml>
```
$ python robot_io/control/teleop_robot.py --config-name=[panda_teleop|kuka_teleop]
```