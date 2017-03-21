//
// Created by andreas on 2017-03-14.
//

#ifndef PERSISTENT_ACTIONLIB_PERSISTENTACTIONSERVER_H
#define PERSISTENT_ACTIONLIB_PERSISTENTACTIONSERVER_H

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/node_handle.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;
typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> Server;

class PersistentActionServer
{
public:
    PersistentActionServer();
    void HandleActionlibCallback(const move_base_msgs::MoveBaseGoalConstPtr& goal);
    void ClientDoneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
    void ClientActiveCallback();
    void ClientFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
private:
    Client client;
    Server server;
    move_base_msgs::MoveBaseGoal goal;
    bool running = false;

    void SendCurrentGoal();
    void CancelCurrentGoal();
};


#endif //PERSISTENT_ACTIONLIB_PERSISTENTACTIONSERVER_H
