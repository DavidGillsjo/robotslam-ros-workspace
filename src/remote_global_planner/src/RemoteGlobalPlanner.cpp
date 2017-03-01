#include <pluginlib/class_list_macros.h>
#include "RemoteGlobalPlanner.h"
#include <tf/LinearMath/Vector3.h>

//register this planner as a BaseGlobalPlanner plugin
namespace remote_global_planner
{
    PLUGINLIB_EXPORT_CLASS(remote_global_planner::RemoteGlobalPlanner, nav_core::BaseGlobalPlanner);
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
            //this->plan = plan_t();
            this->node_handler = ros::NodeHandle("move_base/" + name);
            //this->subscriber = node_handler.subscribe("remote_global_plan_listener", 1, &RemoteGlobalPlanner::planCallback, this);
            this->subscriber = node_handler.subscribe("remote_global_plan_listener", 1, &PlanManager::planCallback, &this->plan_manager);
            this->publisher = node_handler.advertise<nav_msgs::Path>("global_plan", 1);
            this->immediate_publisher = node_handler.advertise<nav_msgs::Path>("immediate_global_plan", 1);

            this->node_handler.param("waypoint_radius", this->waypoint_radius, this->waypoint_radius);

            ROS_INFO_STREAM("RemoteGlobalPlanner: Initialized successfully, Name: " + name);
            this->initialized = true;
        }
        else
        {
            ROS_WARN("Remote global planner has already been initialized... doing nothing");
        }
    }

    bool RemoteGlobalPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                       plan_t &plan_out)
    {
        std::stringstream fmt;
        fmt << "RemoteGlobalPlanner: makePlan(start: ["
            << start.pose.position.x << ", " << start.pose.position.y << ", " << start.pose.position.z
            << "], goal: ["
            << goal.pose.position.x << ", " << goal.pose.position.y << ", " << goal.pose.position.z
            << "])";
        ROS_INFO_STREAM(fmt.str());

        plan_out = this->plan_manager.getCurrentPlan();

        if (plan_out.empty()) {
            return false;
        }

        geometry_msgs::PoseStamped& current_waypoint = plan_out[0];

        /*tf::Vector3 cp_vec(start.pose.position.x, start.pose.position.y, start.pose.position.z);
        tf::Vector3 wp_vec(current_waypoint.pose.position.x, current_waypoint.pose.position.y, current_waypoint.pose.position.z);
        tfScalar distance = wp_vec.distance(cp_vec);

        std::stringstream fmt_1;
        fmt_1 << "RemoteGlobalPlanner: " << distance << "m from next waypoint";
        ROS_INFO_STREAM(fmt_1.str());*/

        //plan[0] = start;
        /*if (this->isAtWaypoint(start, current_waypoint))
        {
            plan.erase(plan.begin());
        }
        */
        //plan_out = plan;
        //plan_out = manager.getCurrentPlan(start);
        //plan_out = plan_t(plan.begin(), std::min(plan.begin() + 2, plan.end()));
        plan_out.insert(plan_out.begin(), start);

        nav_msgs::Path path;
        path.header.frame_id = "map";
        path.header.seq = 50;
        path.header.stamp = ros::Time();
        path.poses = this->plan_manager.getFullPlan();
        publisher.publish(path);

        nav_msgs::Path immediate_path;
        immediate_path.header.frame_id = "map";
        immediate_path.header.seq = 50;
        immediate_path.header.stamp = ros::Time();
        immediate_path.poses = plan_out;
        immediate_publisher.publish(immediate_path);
        return true;
    }

    const bool RemoteGlobalPlanner::isAtWaypoint(const pose_t current_position, const pose_t waypoint)
    {
        tf::Vector3 cp_vec(current_position.pose.position.x, current_position.pose.position.y, current_position.pose.position.z);
        tf::Vector3 wp_vec(waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z);
        tfScalar distance = wp_vec.distance(cp_vec);
        return distance <= this->waypoint_radius;
    }
}