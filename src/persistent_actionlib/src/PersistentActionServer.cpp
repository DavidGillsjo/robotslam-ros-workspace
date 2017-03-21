//
// Created by andreas on 2017-03-14.
//

#include "PersistentActionServer.h"

PersistentActionServer::PersistentActionServer() : client("move_base", true),
                                                   server("move_base_persistent", boost::bind(&PersistentActionServer::HandleActionlibCallback, this, _1), false)
{
    //client = Client("move_base", true); // true -> don't need ros::spin()
    ROS_INFO("Connecting to move_base actionlib server...");
    client.waitForServer();
    ROS_INFO("Connected! Starting our own actionlib server...");
    server.start();
    ROS_INFO("Started! Spinning away...");
    ros::spin();
}

void PersistentActionServer::HandleActionlibCallback(const move_base_msgs::MoveBaseGoalConstPtr& received_goal)
{
    ROS_INFO("Received new goal.");
    goal = *received_goal; // Copy goal
    SendCurrentGoal();
    ros::Rate r(1);
    while (!server.isPreemptRequested() && ros::ok())
    {
        r.sleep();
    }
    CancelCurrentGoal();
}

void PersistentActionServer::SendCurrentGoal()
{
    ROS_INFO("Sending goal.");
    client.sendGoal(goal,
            boost::bind(&PersistentActionServer::ClientDoneCallback, this, _1, _2),
            boost::bind(&PersistentActionServer::ClientActiveCallback, this),
            boost::bind(&PersistentActionServer::ClientFeedbackCallback, this, _1)
    );
}

void PersistentActionServer::CancelCurrentGoal()
{
    client.cancelGoal();
    server.setPreempted();
}

void PersistentActionServer::ClientDoneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
{
    //ROS_INFO("Client reports done.");
    if (server.isActive())
    {
        SendCurrentGoal();
    }
}

void PersistentActionServer::ClientActiveCallback()
{
    //ROS_INFO("ClientActiveCallback.");
}

void PersistentActionServer::ClientFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
    //ROS_INFO("ClientFeedbackCallback.");
    server.publishFeedback(feedback);
}