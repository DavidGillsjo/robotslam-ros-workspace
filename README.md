# ros-workspace

## Things you might need
Install stuff
```sudo apt-get install python-wstool python-rosdep ninja-build```

Run stuff
```wstool update -t src```
```bash
# Install deb dependencies.
rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

# Build and install.
catkin_make_isolated --install --use-ninja
source install_isolated/setup.bash
```

## Building the docker image
1. Install docker (and nvidia-docker if you have a nvidia GPU)
2. From repo root: `cd docker`
3. `./build.sh`
4. Wait until finished, it may take some time...

## Using the docker image
1. `cd docker`
2. Simplest form: `./run`.
..* To use your nvidia card, add the option `USE_NVIDIA=1`.
..* If you want to mount another directory than you home, use `HOME=<path_to_mount>`.
..* I.e. `USE_NVIDIA=1 HOME=<path_to_mount> ./run.sh`
3. To attach more terminals to the container, use `./attach.sh <container_name>`.
Which will be `robotslam_intel` or `robotslam_nvidia`.
4. Once inside, run `source install_isolated/setup.bash` and you're ready to go.

## TurtleBot Office Mapping

We start by bringing up the map.
```bash
WORLD_FILE=worlds/willowgarage.world roslaunch turtlebot_gazebo turtlebot_world.launch
```

Which launches the turtlebot gazebo instance with turtlebot_world launch file and the "office" world.

We then initiate a new gmapping.
```bash
roslaunch turtlebot_gazebo gmapping_demo.launch
```

And to visualize we use rviz.
```bash
roslaunch turtlebot_rviz_launchers view_navigation.launch
```

And to navigate around we use teleop.
```bash
roslaunch turtlebot_teleop keyboard_teleop.launch
```
<
