//
// Created by andreas on 2017-03-14.
//

#include "PersistentActionServer.h"

PersistentActionServer::PersistentActionServer() {
    Client client("do_dishes", true); // true -> don't need ros::spin()
    client.waitForServer();
    chores::DoDishesGoal goal;
    // Fill in goal here
    client.sendGoal(goal);
    client.waitForResult(ros::Duration(5.0));
    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        printf("Yay! The dishes are now clean");
    printf("Current State: %s\n", client.getState().toString().c_str());
    return 0;
}