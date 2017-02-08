#!/bin/bash

for i in {1..23}
do
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{ header: { frame_id: "map" }, pose: { position: { x: 3, y: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } }' &

sleep 12

rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{ header: { frame_id: "map" }, pose: { position: { x: 0, y: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } }' &

sleep 21

echo $i

done
