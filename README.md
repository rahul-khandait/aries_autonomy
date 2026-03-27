# Aries-Mars Rover workspace description and instructions for Pure Pursuit

## Pre-requisites

ROS2 Jazzy/Humble  
Gazebo Harmonic or up

## Sensor used

imu  
Realsense D435i Camera  
Lidar  


## installation

1. Source workspace

   ```
   cd aries_autonomy
   colcon build
   source install/setup.bash
   ```

2. command for auto detect dependency and install to run file
   
	  ```
	  rosdep install --from-paths src -r -y
	  ```
> [!IMPORTANT]
> Change mesh location in common_properties.xacro which is in urdf files (To location in your pc)


### Gazebo (Rviz optional in code)
  
Terminal 1:  ros2 launch aries my_robot.launch.xml
  
### Global path generator

Terminal 2: ros2 run pure_pursuit_controller path_generator

### Pure Pursuit

Terminal 3: ros2 run pure_pursuit_controller pure_pursuit

### Validator IMU vs ODOM data

Terminal 4: ros2 run pure_pursuit_controller imu_odom_validator


