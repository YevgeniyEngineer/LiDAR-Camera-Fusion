# Lidar-Camera-Fusion
Processing detections from LiDAR and camera based on Kitti Dataset.

Work in progress...

## Dependencies

### ROS2 Humble
https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html

Build: `colcon build --packages-select lidar_camera_fusion`

Source: `source ./install/setup.bash`

Launch:

### OpenCV5
https://docs.opencv.org/5.x/d7/d9f/tutorial_linux_install.html

## Example Visualization
The node reads sensor data and publishes synchronously

![complete_video](https://github.com/YevgeniyEngineer/LiDAR-Camera-Fusion/blob/main/images/visualisation.gif)
