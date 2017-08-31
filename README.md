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
