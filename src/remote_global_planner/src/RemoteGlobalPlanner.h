/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <string>
#include <algorithm>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <tf/LinearMath/Vector3.h>
#include "PlanManager.h"

using std::string;

#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

namespace remote_global_planner {
    typedef geometry_msgs::PoseStamped pose_t;
    typedef std::vector<pose_t> plan_t;
    typedef costmap_2d::Costmap2DROS costmap_t;

    class RemoteGlobalPlanner : public nav_core::BaseGlobalPlanner {
    public:
        RemoteGlobalPlanner();
        RemoteGlobalPlanner(string name, costmap_t* costmap_ros);

        /** overridden classes from interface nav_core::BaseGlobalPlanner **/
        void initialize(string name, costmap_t* costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped& start,
                      const geometry_msgs::PoseStamped& goal,
                      plan_t& plan
        );
        void planCallback(const nav_msgs::Path path);
    private:
        string name;
        costmap_t* costmap_ros;
        plan_t plan;
        ros::NodeHandle node_handler;
        ros::Subscriber subscriber;
        ros::Publisher publisher;
        ros::Publisher immediate_publisher;
        bool initialized;
        PlanManager plan_manager;

        // Parameters
        double waypoint_radius = 0.15;

        /**
         * Checks if current_position is the same position as waypoint.
         * @param current_position Position to compare to waypoint.
         * @param waypoint Position to compare current_position to.
         * @return True if current_position is within the #RemoteGlobalPlanner::waypoint_radius of waypoint
         */
        const bool isAtWaypoint(const pose_t current_position, const pose_t waypoint);

        /**
         * Used to listen for tf updates and remove waypoints from the plan as they are reached.
         */
        void updatePlan();
    };
};
#endif