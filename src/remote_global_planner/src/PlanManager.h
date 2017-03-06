#include <ros/ros.h>
#include <boost/thread.hpp>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>

#ifndef REMOTE_GLOBAL_PLANNER_PLANMANAGER_H
#define REMOTE_GLOBAL_PLANNER_PLANMANAGER_H

namespace remote_global_planner {
    typedef geometry_msgs::PoseStamped pose_t;
    typedef std::vector<pose_t> plan_t;

    class PlanManager {
    public:
        PlanManager();
        ~PlanManager();
        void planCallback(const nav_msgs::Path path);
        void skipWaypoint(const std_msgs::Header header);

        plan_t getCurrentPlan();
        plan_t getFullPlan();
    private:
        boost::thread* rt_thread;
        plan_t plan;

        // Parameters
        double waypoint_radius = 0.25;
        int immediate_waypoints = 2;

        /**
         * Listen to TF updates
         */
        void listen();

        /**
         * Checks if current_position is the same position as waypoint.
         * @param current_position Position to compare to waypoint.
         * @param waypoint Position to compare current_position to.
         * @return True if current_position is within the #RemoteGlobalPlanner::waypoint_radius of waypoint
         */
        const bool isAtWaypoint(const tf::Vector3 current_position, const pose_t waypoint);
    };
}


#endif
