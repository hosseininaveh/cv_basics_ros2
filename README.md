# cv_basics_ros2
The cv_basics includes a package for working with OpenCV functions in ROS 2 iron.

# installation
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
