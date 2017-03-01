#include "PlanManager.h"

namespace remote_global_planner {

    PlanManager::PlanManager() {
        ROS_INFO("PlanManager Started!");
        this->rt_thread = new boost::thread(boost::bind(&PlanManager::listen, this));
    }

    PlanManager::~PlanManager() {
        ROS_INFO("PlanManager - Stopped!");
        this->rt_thread->interrupt();
    }

    void PlanManager::listen() {
        ROS_INFO("PlanManager - Started Listen Thread!");

        tf::TransformListener listener;
        //ros::Rate rate(10.0);
        ros::Rate rate(5.0);

        while (ros::ok()) {

            // Check if the thread is interuppted
            boost::this_thread::interruption_point();

            tf::StampedTransform transform;

            try {
                listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
            } catch (tf::TransformException &ex) {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }

//            ROS_INFO_STREAM("MY POS IS " << transform.getOrigin().x() << ", " << transform.getOrigin().y());

            if (plan.size() > 0) {
                if (this->isAtWaypoint(transform.getOrigin(), plan[0])) {
                    plan.erase(plan.begin());
                }
            }

            rate.sleep();
        }

        ROS_INFO("PlanManager - Stopped thread");

        return;
    }

    void PlanManager::planCallback(const nav_msgs::Path path) {
        ROS_INFO("RemoteGlobalPlanner: Remote global planner got a new plan. Will publish when asked for.");
        plan = path.poses;
        //plan.insert(plan.begin(), pose_t());
    }

    plan_t PlanManager::getCurrentPlan() {
        return plan;
    }

    const bool PlanManager::isAtWaypoint(const tf::Vector3 current_position, const pose_t waypoint) {
        tf::Vector3 cp_vec(current_position.x(), current_position.y(),
                           current_position.z());
        tf::Vector3 wp_vec(waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z);
        tfScalar distance = wp_vec.distance(cp_vec);
//        ROS_INFO_STREAM("Next Waypoint: " << waypoint.pose.position.x << ", " << waypoint.pose.position.y);
//        ROS_INFO_STREAM("Distance to waypoint: " << distance);
        return distance <= this->waypoint_radius;
    }

}