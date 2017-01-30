# ros-workspace

## TurtleBot Office Mapping

We start by bringing up the map.
```bash
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=worlds/willowgarage.world
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
