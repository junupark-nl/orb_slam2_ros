# orb_slam2_ros
A ROS wrapper for [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2).

## Prerequisite
```shell
cd orb_slam2_core/Vocabulary
tar -xf ORBvoc.txt.tar.gz
```

---

## Typical Use (monocular fisheye)
1. Build map prior to the deployment. (offline)
2. Deploy with localization mode enabled. (online)


### 1. Launch with remapped image_topic

```shell
roslaunch orb_slam2_ros mono_fisheye.launch image_topic:=<fisheye_image_topic> fisheye_calibration_file:=fisheye_starling2_front_modalai.yaml resize_factor:=2.0
```
- On **Starling2 (Max) Voxl2**: `fisheye_image_topic:=/ifs/tracking_front`
- Calibration files are located at `./config/`
- The size (and thus resolution) of an image is resized (divided) by the factor of `resize_factor`.

### 2. Turn mapping mode on
- SLAM is initiated as SLAM (localization+mapping) mode by default.

### 3. Build map
- One can initialize SLAM by moving the vehicle/camera around.
- You will need **sufficient and consistent disparity** in order to initialize monocular SLAM.
- Move the vehicle around, taking care not to lose track.
- Make sure closing loop(s) and diversifying viewpoints.

### 4. Save map
```shell
rosservice call /orb_slam2_<type>/save_map <map_file_name>
```
- Don't include the extension, i.e., .bin. 
    - `rosservice call /orb_slam2_ros/save_map ftrc_indoor` is a good example.
- The map and the initial vehicle pose will be saved at `./resource/`

    1. map: `./resource/<map_file_name>.bin`
    2. vehicle pose (when the SLAM is initialized): `./resource/<map_file_name>_initial_tf.bin`

> [!NOTE]
> the ORB_SLAM2 itself has nothing to do with the vehicle pose and the reference frame of ORB_SLAM2 is the camera frame (right-down-forward) **when the system is initialized**.

### 5. Relaunch with map file name
```shell
roslaunch orb_slam2_ros mono_fisheye.launch image_topic:=<fisheye_image_topic> fisheye_calibration_file:=fisheye_starling2_front_modalai.yaml resize_factor:=2.0 load_map:=true map_file_name:=<map_file_name>
```
- if you want localization-only with prior map: 
    - `rosservice call /orb_slam2_<type>/set_localization_mode true`
    - The system is initiated with **mapping mode** by default, thus one must explicitly turn localization-only mode on.
    - Note that you must have launched with `load_map:=true map_file_name:=<map_file_name>`
    - `type`: mono/stereo/rgbd


### 6. Utilities
1. Adjust **scale error** of monocular vision SLAM. defaults are 1.
   - `rosservice call /orb_slam2_<type>/rescale <scale_x> <scale_y> <scale_z>`
   - Scale might be heterogeneous.
2. Adjust **offset of origin** with respect to which the estimate is resolved.
   - `rosservice call /orb_slam2_<type>/set_offset <x> <y> <z> <roll_deg> <pitch_deg> <yaw_deg>`
   - This shifts origin.
3. **Filter** map point cloud. 
   - `rosservice call /orb_slam2_<type>/set_mopp <minimum_observation_per_point>`
   - Only keyframes with larger than `minimum_observation_per_point` map points will be included in the published map point cloud.
   - This may filter out outliers.

---
### Currently supported types
1. monocular
2. monocular fisheye (rectified via fisheye adaptor)
3. stereo
4. stereo fisheye (rectified via two fisheye adaptors)