#include <pluginlib/class_list_macros.h>
#include "RemoteGlobalPlanner.h"

//register this planner as a BaseGlobalPlanner plugin
namespace remote_global_planner
{
    PLUGINLIB_EXPORT_CLASS(remote_global_planner::RemoteGlobalPlanner, nav_core::BaseGlobalPlanner)
    RemoteGlobalPlanner::RemoteGlobalPlanner()
    {
        this->initialized = false;
    }

    RemoteGlobalPlanner::RemoteGlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        this->initialized = false;
        initialize(name, costmap_ros);
    }

    void RemoteGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!this->initialized)
        {
            this->name = name;
            this->costmap_ros = costmap_ros;
            this->node_handler = ros::NodeHandle("~/" + name);

            ros::Subscriber sub = node_handler.subscribe("remote_global_plan_listener", 1000, &RemoteGlobalPlanner::planCallback, this);

            ROS_INFO("Remote global planner initialized successfully");
            this->initialized = true;
        }
        else
        {
            ROS_WARN("Remote global planner has already been initialized... doing nothing");
        }
    }

    bool RemoteGlobalPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                       plan_t &plan)
    {
        plan = this->plan;
    }

    void RemoteGlobalPlanner::planCallback(const nav_msgs::Path path)
    {
        this->plan = path.poses;
    }
}