# Lidar-Camera-Fusion
Processing detections from LiDAR and camera based on Kitti Dataset.

Work in progress...

## Dependencies

### ROS2 Humble
https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html

### OpenCV5
https://docs.opencv.org/5.x/d7/d9f/tutorial_linux_install.html

or `sudo apt-get install -y libopencv-dev`

### Eigen
`sudo apt install libeigen3-dev`

### Nanoflann
`sudo apt install libnanoflann-dev`

### PCL
`sudo apt install libpcl-dev`

## Build and Launch
See node configuration for more information (YAML file).

Clean cache generated by CMake and Colcon: `./clear.sh`

Build the project: `./build.sh`

Launch data processing pipelines and visualization: `./launch.sh`

## Example Visualization
The node reads sensor data and publishes synchronously

![complete_video](https://github.com/YevgeniyEngineer/LiDAR-Camera-Fusion/blob/main/images/visualization.gif)

## Segmentation

![complete_video](https://github.com/YevgeniyEngineer/LiDAR-Camera-Fusion/blob/main/images/example_segmentation.gif)
