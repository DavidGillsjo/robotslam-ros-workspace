#!/usr/bin/env python
import os
from robotslam_data_collection.srv import Start, StartResponse
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String
from threading import RLock, Thread, Event
import roslaunch
from datetime import datetime
import rosbag
import subprocess

import rospy

class CollectionMode:
    IDLE = 0
    ERROR = 1
    EXPLORING = 2
    COVERAGE = 3

def launchPackage(package, executable, env = []):
    node = roslaunch.core.Node(package, executable, env_args=env)

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    process = launch.launch(node)

    return process

class StoppableThread(Thread):
    def __init__(self, target, args):
        super(StoppableThread, self).__init__(target=target, args=args)
        self._stop_event = Event()

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

def rosbagWithCb(path, topics, callback_fn):
    rcode = subprocess.call(["rosbag", "record", "-o", path, "--lz4"] + topics)
    callback_fn(rcode)

class DataCollector:

    def __init__(self):
        rospy.init_node('data_collector')

        self.status_msgs = ["Idle",
                            "Error"
                            "Exploring",
                            "Covering Map"]
        self.error_msg = ""
        self.mode = CollectionMode.IDLE
        self.lock = RLock()
        self.launch_handle = None
        self.bag_thread = None
        rospy.on_shutdown(self.shutdown)

        # Services
        self.start_srv = rospy.Service('~start', Start, self.handleStart)
        self.end_srv = rospy.Service('~end', Trigger, self.handleEnd)

        #Publisher
        self.pub = rospy.Publisher('~status', String, queue_size=5)

    def handleStart(self, req):
        self.lock.acquire()

        if self.mode <= CollectionMode.ERROR:
            try:
                success, msg = self.startCollection(req)
            except Exception as e:
                success = False
                msg = str(e)
        else:
            success = False
            msg = "Robot is active, stop current action first."

        self.lock.release()

        return StartResponse(success=success, message=msg)

    def startCollection(self, req):
        rospy.loginfo("Starting data collection")

        if req.map == None:
            # Exploration
            msg = ""
            self.launch_handle = launchPackage("robotslam_launcher",
                                               "cartographer_exploration")
            self.mode = CollectionMode.EXPLORING
        else:
            # Coverage
            msg = "Coverage is not yet implemented, running exploration."
            self.launch_handle = launchPackage("robotslam_launcher",
                                               "cartographer_exploration")
            self.mode = CollectionMode.EXPLORING

        if req.store_rosbag:
            # Start rosbag
            bag_dir = rospy.get_param('rosbag_dir', '/tmp')
            bag_path = os.path.join(bag_dir,"{}_{}".format(datetime.now(), req.name))
            self.startBag(bag_path, req.topics)

    def startBag(self, bag_path, topics):
        self.bag_thread = StoppableThread(target=rosbagWithCb, args=(bag_path, topics, self.bagCb))
        self.bag_thread.start()
        rospy.loginfo("Rosbag started")

    def bagCb(self,rcode):
        rospy.loginfo("Rosbag exited")
        if rcode != 0:
            rospy.loginfo("Rosbag failed, enter error mode")
            self.lock.acquire()
            self.mode = CollectionMode.ERROR
            self.lock.release()

    def handleEnd(self, req):
        # Stop recording
        self.stopCollection()

        return TriggerResponse(success=True, message="")

    def stopCollection(self, forced = False):
        rospy.loginfo("Stopping data collection")
        if not forced:
            self.lock.acquire()
        # Store map etc?

        # Stop processes
        if self.launch_handle:
            self.launch_handle.stop()

        if self.bag_thread:
            self.bag_thread.stop()
            self.bag_thread.join()

        # Restore mode
        if self.mode != CollectionMode.ERROR:
            self.mode = CollectionMode.IDLE

        if not forced:
            self.lock.release()

    def publish_status(self):
        if self.mode != CollectionMode.ERROR:
            self.pub.publish(self.status_msgs[self.mode])
        else:
            self.pub.publish(self.error_msg)

    def monitor_status(self):
        if self.mode == CollectionMode.ERROR:
            self.stopCollection()


    def run(self):
        rate = rospy.Rate(1) # 1hz
        while not rospy.is_shutdown():
            self.publish_status()
            self.monitor_status()
            rate.sleep()

    def shutdown(self):
        if self.mode >= CollectionMode.EXPLORING:
            self.stopCollection(forced=True)

if __name__ == "__main__":
    try:
        dc = DataCollector()
        dc.run()
    except rospy.ROSInterruptException:
        pass
