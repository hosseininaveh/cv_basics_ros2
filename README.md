# cv_basics_ros2
The cv_basics includes a package for working with OpenCV functions in ROS 2 iron.

# Installation
to install the cv_basics, the following command should be run in Ubuntu 22.04 after installing ROS 2 Iron 

  ``` 
  mkdir -p ros2_ws/src
  cd ros2_ws/src
  git clone https://github.com/hosseininaveh/cv_basics_ros2.git
  cd ..
  rosdep install --from-paths src --ignore-src -y
  source install/setup.bash
  ```

# Test 
In order to test the package, simply run the following code to read a video file stored in the resource directory and publish the frames in video_frames topic:
  
  ``` 
  cd ~/ros2_ws/src/cv_basics_ros2/resource/
  ros2 run cv_basics frame_publisher ./camera_calib.mp4 ./camera_calibration_params.yaml
  ```
It is possible to see the publishing frames using different tools including rviz2 or rqt_image_view. 
Moreover, dispite available packages for reading data from video file, this package can be used for camera calibration. Use the following to see the camera calibration parameters stored in resource/camera_calibration_params.yaml file
  ```
  # this command shows the list of topics that the node is publishing 
  ros2 topic list
  ros2 topic echo /camera_info 
  ```

# Camera Calibration
Online Calibration
First, we have to collect the ROS 2 package for the calibration:

  ```
  sudo apt install ros-iron-camera-calibration
  ```

We also need a pattern to use during the calibration. 
You can generate one easily here: https://calib.io/pages/camera-calibration-pattern-generator. 
To fit your pattern on an A4/US letter page, I suggest an 8x10 board with 25mm squares. 
Print it on thick paper to ensure it doesn’t bend during the calibration process.
Then, record a video similar to the one is stored in resoures folder and run the previous node for your video file.  
 
To start calibration in one console, start the following command:
  ```
  ros2 run camera_calibration cameracalibrator --size 7x9 --square 0.025 --ros-args -r image:=/video_frames
  ```
The size option here denotes interior corners (e.g. a standard chessboard is 7x7), so for an 8x10 checkerboard, we go with 7x9.

