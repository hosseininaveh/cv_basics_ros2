# cv_basics_ros2
The cv_basics includes a package for working with OpenCV functions in ROS 2 iron.

# installation
to install the cv_basics, the following command should be run in Ubuntu 22.04 after installing ROS 2 Iron 

  mkdir -p ros2_ws/src
  cd ros2_ws/src
  git clone https://github.com/hosseininaveh/cv_basics_ros2.git
  cd ..
  rosdep install --from-paths src --ignore-src -y
  
  
