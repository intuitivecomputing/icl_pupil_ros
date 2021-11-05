# icl_gaze_tracker
Very simple ROS package to interact with the Pupil invisible gaze tracker. 

Connects to the tracker through ndsi interface and publishes the fpv image on `fpv_image/compressed` in `sensor_msgs/CompressedImage` format and the gaze point on `gaze_pt` in `geometry_msgs/PointStamped` fromat.
Parameter `~draw_gaze` enables drawing of the gaze point on the fpv image.

## Usage
```
roslaunch icl_pupil_ros pupil.launch ns:=pupil draw_gaze:=true
```

## Todo
- [ ] IMU if needed

## refs
- [Offical documents](https://docs.pupil-labs.com/developer/invisible/#network-api)
- [Gopika's script](https://github.com/intuitivecomputing/demo_data_processing/blob/master/scripts/gaze_data_pub.py)

