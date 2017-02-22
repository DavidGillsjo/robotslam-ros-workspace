#include "PlanManager.h"

remote_global_planner::PlanManager::PlanManager() {
    ROS_INFO("PlanManager Started!");
    this->rt_thread = new boost::thread(boost::bind(&PlanManager::listen, this));
}

remote_global_planner::PlanManager::~PlanManager() {
    ROS_INFO("PlanManager - Stopped!");
    this->rt_thread->interrupt();
}

void remote_global_planner::PlanManager::listen() {
    ROS_INFO("PlanManager - Started Listen Thread!");

    tf::TransformListener listener;
    ros::Rate rate(10.0);

    while(ros::ok()) {

        // Check if the thread is interuppted
        boost::this_thread::interruption_point();

        tf::StampedTransform transform;

        try {
            listener.lookupTransform("/base_link", "/map", ros::Time(0), transform);
        } catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        ROS_INFO_STREAM("MY POS IS " << transform.getOrigin().x());

        rate.sleep();
    }

    ROS_INFO("PlanManager - Stopped thread");

    return;
}
