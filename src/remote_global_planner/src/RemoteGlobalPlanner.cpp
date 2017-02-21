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
            this->plan = plan_t();
            //this->callback_queue = new ros::CallbackQueue();
            this->node_handler = new ros::NodeHandle(name);
            //this->node_handler->setCallbackQueue(this->callback_queue);

            this->subscriber = node_handler->subscribe("remote_global_plan_listener", 1, &RemoteGlobalPlanner::planCallback, this);

//            this->spinner = new ros::AsyncSpinner(1);
//            this->spinner->start();
//            ros::waitForShutdown();
//            ros::spin();

            ROS_INFO_STREAM("Remote global planner initialized successfully, Name: " + name);
//            ROS_INFO("Remote global planner initialized successfully");
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
        std::stringstream fmt;
        fmt << "RemoteGlobalPlanner: makePlan(start: ["
            << start.pose.position.x << ", " << start.pose.position.y << ", " << start.pose.position.z
            << "], goal: ["
            << goal.pose.position.x << ", " << goal.pose.position.y << ", " << goal.pose.position.z
            << "])";
        ROS_INFO_STREAM(fmt.str());
        plan = this->plan;
        this->plan.clear();
    }

    void RemoteGlobalPlanner::planCallback(const nav_msgs::Path path)
    {
        ROS_INFO("[===================] Remote global planner got a new plan. Will publish when asked for.");
        this->plan = path.poses;
    }
}