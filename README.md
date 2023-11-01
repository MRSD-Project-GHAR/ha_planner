# ha_planner
This planner plans the hardware-optimized path between two way points

# Installation:
## Locobot Navigation Stack
```sudo apt install curl```

```curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_rovers/main/interbotix_ros_xslocobots/install/xslocobot_remote_install.sh' > xslocobot_remote_install.sh```

```chmod +x xslocobot_remote_install.sh```

```./xslocobot_remote_install.sh -d noetic -b kobuki```

## Planning Stack
- Make a workspace (ghar_ws) and clone the ha_planner and elevation_mapping repos from gour github.
- Build them

## Network configuration
- connect to the same network as locobot
- add ip address of locobot (192.168.0.124) in etc/hostnames with the name as vfa
- run   
```sudo service network-manager restart```

### To test
- ssh into locobot
- run in locobot terminal   
```roslaunch interbotix_xslocobot_control xslocobot_control.launch robot_model:=locobot_wx200 use_base:=true use_camera:=true use_lidar:=false```
- run in own computer   
```roslaunch interbotix_xslocobot_descriptions remote_view.launch```
You should see rviz with the robot.

# To keyop the locobot:
- On the locobot terminal   
```roslaunch interbotix_xslocobot_control xslocobot_control.launch robot_model:=locobot_wx200 use_base:=true use_camera:=true use_lidar:=false```
- Your own terminal   
```roslaunch kobuki_keyop keyop.launch __ns:=locobot```

# To launch the stack on locobot:
### Start the navstack on locobot
```roslaunch interbotix_xslocobot_nav xslocobot_nav.launch robot_model:=locobot_wx200 use_lidar:=false rtabmap_args:=-d```

### On the personal computer's terminal launch the planner
```roslaunch mbf_rrts_planner planner_move_base.launch```

### To generate the plan:
    - Load the map from dynamic reconfigure
    - Give the start pos ```start_topic```
    - Give the goal pos ```goal_topic```
    - Call service to generate the plan 
    - Call service to execute the plan