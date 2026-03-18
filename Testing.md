# ROS2 Controller repository
ROS2 Workspace for car controller

## Setup Instructions

Start reco fiit docker-compose on a linux machine

### Prerequisities

chance directory to tp5_ros

```bash
cd new_ros/tp5_ros
```

### Steps for carla ros (linux)

Check config files to make upd_client_config

```
*reco fiit ip*
4040
C4RF117S7U0000002
0
```


- Source ros2  `source /opt/ros/humble/setup.bash`.

- Source the tp5_ros ros module `source install/setup.bash`

- Build the ROS2 after you make a change in the code. `colcon build`

- Run the carla module `ros2 launch tp5_ros carla_sim_launch.xml`

### Steps for other computer

- Running the Docker. `docker compose up -d --build`
- Running the Docker. `docker exec -it ros2_ws-sim-1 /bin/bash`

- Run gps sim node. `ros2 launch tp5_ros gps_sim_launch.xml`