#include <ros/ros.h>
#include <boost/thread.hpp>
#include <tf/transform_listener.h>

#ifndef REMOTE_GLOBAL_PLANNER_PLANMANAGER_H
#define REMOTE_GLOBAL_PLANNER_PLANMANAGER_H

namespace remote_global_planner {
    class PlanManager {
    public:
        PlanManager();
        ~PlanManager();
    private:
        boost::thread* rt_thread;

        /**
         * Listen to TF updates
         */
        void listen();
    };
}


#endif
