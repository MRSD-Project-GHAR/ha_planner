# ha_planner
This planner plans the hardware-optimized path between two way points


## Dependencies

### ROS
ROS needs to be installed, this code has been tested on ROS Noetic. Install the desktop-full version by following the instructions on [this](https://wiki.ros.org/noetic/Installation) site. Create a catkin workspace, and clone this repository in the src folder of your catkin workspace.

### Qt
Qt for open-source needs to be installed by downloading the online installer from [this](https://www.qt.io/download-open-source) link. Any version of Qt6 will work. Only Qt-6.x.x needs to be installed, the rest of the addons (like Qt designer or Qt creator) don't need to be installed. Please set the environment variable QT_DIR to the gcc folder inside your Qt installation directory. For example, if I have Qt installed in the `/home/kshitij/Qt/6.5.0` directory, QT_DIR needs to be set to `/home/kshitij/Qt/6.5.0/gcc_64`.

### Grid Map
Please also clone [this](https://github.com/MRSD-Project-GHAR/grid_map) repository in the src folder of the catkin workspace.

### Move Base Flex

Can be installed using the command - `sudo apt install ros-noetic-move-base-flex`

## Running instructions
Build the catkin workspace using  `catkin_make -DCMAKE_BUILD_TYPE=Release` command. After building the workspace, source it using the command `source devel/setup.bash`. This needs to be done in every new terminal that is opened. 

Open 3 terminals, and run the following commands in them
- Terminal 1: `roslaunch rand_grid_map_gen map_publisher.launch`

- Terminal 2: `roslaunch rand_grid_map_gen map_filter.launch robot_name:=locobot`

- Terminal 3: `roslaunch mbf_rrts_planner planner_move_base.launch robot_name:=locobot`

Terminal 1 and Terminal 3 open up two GUIs that can be used for testing the planner. The terminal 1 GUI can be used to publish a map of your choice, and the terminal 2 GUI can be used to generate a global plan. The terminal 2 GUI takes some time to start (about 5 seconds). Terminal 2 opens up RViz. 

To load and publish a map, the Obstacle Control Panel GUI needs to be used. Please enter the name of the map in the `Map Name` field, and click `Load Map` button followed by `Publish Map` button. The map name choices that can be used are:

- 4_small_obs.yaml
- 4_test_locobot.yaml
- 4.yaml
- combo1.yaml
- combo2.yaml
- combo3.yaml
- combo4.yaml
- combo5.yaml
- concrete_floor.yaml
- confined_boundary.yaml
- empty_zade_house.yaml
- mrsd_lab.yaml
- new_layout2.yaml
- new_layout.yaml
- outersense_map.yaml
- test_case1.yaml
- test_case2.yaml
- zade_house0.yaml
- zade_house1.yaml
- zade_house2.yaml
- zade_house3.yaml
- zade_house4.yaml
- zade_house5.yaml
- zade_house.yaml

After the map is published, you should be able to see it in RViz. 

To do global planning, the Planner Control Panel GUI needs to be used. To give it a start point, please click on the button `Get Start Point From RViz`. The status will change to `Waiting for Start Pose from RViz`. Please go in the RViz window, click on the `Publish Point` button, and click on a point within the map that you want to have as the start location. Similarly, click on the button `Get Goal Point From RViz` to get the goal point using the same procedure. The start and goal point received should show up in the status next to those buttons. After that, you can click on the `Generate Plan` button to generate the global plan. After planning is complete, you can see the generated plan in RViz.

A new start and/or goal point can be given, and the process can be repeated to generate another global plan. A new map can also be loaded. Sometimes, RViz keeps dying when doing the planning process again, in this case, simply close and restart everything.
