## To run SLAM with ROS2 and Jetrover 
```bash
# Stop auto start service (Jetrover)
sudo systemctl stop start_app_node.service
```

```bash
# Start robot hardware i.e. camera, lidar, motors and etc. (Jetrover) 
ros2 launch bringup bringup.launch.py
```

```bash
# Ensure that ROS2 slam_toolbox is installed (Host Machine) 
sudo apt install ros-humble-slam-toolbox 
```

#### Note: Download the yaml file and change path_to_config to the actual path of the yaml file
```bash
# To run Online Asynchronous SLAM (Host Machine) 
ros2 launch slam_toolbox online_async_launch.py params_file:=path_to_config/mapper_params_online_async.yaml
```
