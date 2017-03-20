//
// Created by andreas on 2017-03-14.
//

#ifndef PERSISTENT_ACTIONLIB_PERSISTENTACTIONSERVER_H
#define PERSISTENT_ACTIONLIB_PERSISTENTACTIONSERVER_H

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

class PersistentActionServer
{
public:
    PersistentActionServer();
private:
    Client client;
};


#endif //PERSISTENT_ACTIONLIB_PERSISTENTACTIONSERVER_H
