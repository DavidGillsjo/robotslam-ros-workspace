//
// Created by andreas on 2017-03-14.
//

#include "PersistentActionServer.h"

void execute(const move_base_msgs::MoveBaseGoalConstPtr& goal, PersistentActionServer* server)
{
    server->HandleActionlibCallback(goal);
}

PersistentActionServer::PersistentActionServer() : node_handle(), client("move_base", true),
                                                   server(node_handle, "move_base_persistent", boost::bind(&execute, _1, this), false)
{
    //client = Client("move_base", true); // true -> don't need ros::spin()
    client.waitForServer();
    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 1.0;
    goal.target_pose.pose.orientation.w = 1.0;

    client.sendGoal(goal);

    client.waitForResult(ros::Duration(5.0));
    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        printf("Yay! The dishes are now clean");

    printf("Current State: %s\n", client.getState().toString().c_str());
}

void PersistentActionServer::HandleActionlibCallback(const move_base_msgs::MoveBaseGoalConstPtr& goal)
{

}