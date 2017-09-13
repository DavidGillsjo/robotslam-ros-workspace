#!/usr/bin/env python

from robotslam_data_collection.srv import Start, StartResponse
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String
from threading import RLock, Thread
import roslaunch
from datetime import datetime
import rosbag

import rospy

class CollectionMode:
    IDLE
    EXPLORING
    COVERAGE
    ERROR

def launchPackage(package, executable, env = []):
    node = roslaunch.core.Node(package, executable, env_args=env)

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    return launch

class StoppableThread(threading.Thread):
    def __init__(self):
        super(StoppableThread, self).__init__()
        self._stop_event = threading.Event()

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
                            "Exploring",
                            "Covering Map"]
        self.error_msg = ""
        self.mode = CollectionMode.IDLE
        self.lock = RLock()
        self.launch_handle = None
        self.bag_thread = None

        # Services
        self.start_srv = rospy.Service('start', Start, self.handle_start)
        self.end_srv = rospy.Service('end', Trigger, self.handle_end)

        #Publisher
        self.pub = rospy.Publisher('status', String, queue_size=10)

    def handleStart(self, req):
        self.lock.acquire()

        if self.mode == CollectionMode.IDLE:
            try:
                success, msg = self.startCollection(req)
            except Error as e:
                success = False
                msg = str(e)
        else:
            success = False
            msg = "Robot is active, stop current action first."

        self.lock.release()

        return StartResponse(success=success, message=msg)

    def startCollection(self, req):
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

        if req.rosbag:
            # Start rosbag
            bag_dir = rospy.get_param('rosbag_dir', '/tmp')
            bag_path = os.path.join(bag_dir,"{}_{}".format(datetime.now(), req.name))
            self.startBag(bag_path, req.topics)

    def startBag(self, bag_path, topics):
        self.bag_thread = StoppableThread(target=rosbagWithCb, args=(bag_path, topics, bagCb))
        self.bag_thread.start()

    def bagCb(self,rcode):
        if rcode != 0:
            self.lock.acquire()
            self.mode = CollectionMode.ERROR
            self.lock.release()

    def handleEnd(self, req):
        self.lock.acquire()

        # Stop recording
        self.stopCollection()

        self.lock.release()

        return EndResponse(success=True, message="")

    def stopCollection(self):
        # Store map etc?

        # Stop processes
        if self.launch_handle:
            self.launch_handle.shutdown()

        if self.self.bag_thread:
            self.self.bag_thread.stop()
            self.self.bag_thread.join()

    def publish_status(self):
        if self.mode != CollectionMode.ERROR:
            self.pub.publish(self.status_msgs[self.mode])
        else:
            self.pub.publish(self.error_msg)

    def monitor_status(self):
        if self.mode == CollectionMode.ERROR:


    def run(self):
        rate = rospy.Rate(1) # 1hz
        while not rospy.is_shutdown():
            self.publish_status()
            self.monitor_status()
            rate.sleep()

if __name__ == "__main__":
    try:
        dc = DataCollector()
        dc.run()
    except rospy.ROSInterruptException:
        pass
