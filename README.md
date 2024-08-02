# orb_slam2_ros
A ROS wrapper for [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2)

## Prerequisite
```shell
cd orb_slam2_core/Vocabulary
tar -xf ORBvoc.txt.tar.gz
```


## Typical Use (monocular fisheye)
1. Build map prior to the deployment. (offline)
2. Deploy with localization mode enabled. (online)

### 1. Launch with remapped image_topic

```shell
roslaunch orb_slam2_ros mono_fisheye.launch image_topic:=<fisheye_image_topic> fisheye_calibration_file:=fisheye_starling2_front_modalai.yaml resize_factor:=1.0
```
- On **Starling2 (Max) Voxl2**: `fisheye_image_topic:=/ifs/tracking_front`
- Calibration files are located at `./config/`
- The size of image is resized (divided) by the factor of `resize_factor`.

### 2. Turn mapping mode on
```shell
rqt --standalone configure
```
- **Explicitly** turn `enable_localization_mode` off.

### 3. Build map
- **DO NOT** initialize SLAM by moving the vehicle/camera around without 2.
- You will need **sufficient and consistent disparity** in order to initialize monocular SLAM.
- Move the vehicle around, taking care not to lose track.
- Make sure closing loop(s).

### 4. Save map
```shell
rosservice call /orb_slam2_ros/save_map <map_file_name>
```
- Don't include the extension, i.e., .bin. `rosservice call /orb_slam2_ros/save_map ftrc_indoor` is a good example.
- The map and the initial vehicle pose will be saved at `./resource/`

    1. map: `./resource/<map_file_name>.bin`
    2. vehicle pose (when the SLAM is initialized): `./resource/<map_file_name>_initial_tf.bin`

> [!NOTE]
> the ORB_SLAM2 itself has nothing to do with the vehicle pose and the reference frame of ORB_SLAM2 is the camera frame (right-down-forward) **when the system is initialized**.

### 5. Reboot and relaunch with map file name
```shell
roslaunch orb_slam2_ros mono_fisheye.launch image_topic:=<fisheye_image_topic> fisheye_calibration_file:=fisheye_starling2_front_modalai.yaml resize_factor:=2.0 load_map:=true map_file_name:=<map_file_name>
```
- The SLAM system is initiated with **tracking mode** by default, 
    - unless you dynamically (and explicitly) configured it not to do so. (see 2)
- larger `resize_factor` reduces the tracking burden.
    - ORB SLAM is robust to the moderate resolution difference (between mapping and tracking).
    - Adapt the value according to your application.