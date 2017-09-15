#!/usr/bin/env python
import os
from robotslam_data_collection.srv import Start, StartResponse
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String
from threading import RLock
from multiprocessing import Process
import roslaunch
from datetime import datetime
import rosbag
import subprocess

import rospy
import rospkg

class CollectionMode:
    IDLE = 0
    ERROR = 1
    INITIATE_EXPLORING = 2
    INITIATE_COVERAGE = 3
    EXPLORING = 4
    COVERAGE = 5
    STOP = 6

def launchPackage(package, rel_file_path, env = []):
    rospack = rospkg.RosPack()
    package_path = rospack.get_path(package)
    full_path = os.path.join(package_path, rel_file_path)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    #TODO: Supply ProcessListener to monitor for errors.
    launch = roslaunch.parent.ROSLaunchParent(uuid, [full_path])

    launch.start()

    return launch

def rosbagWithCb(path, topics, callback_fn):
    rcode = subprocess.call(["rosbag", "record", "-O", path, "--lz4"] + topics)
    callback_fn(rcode)

class DataCollector:

    def __init__(self):
        rospy.init_node('data_collector')

        self.status_msgs = ["Idle",
                            "Error",
                            "Initating exploration...",
                            "Initiating coverage...",
                            "Exploring",
                            "Covering Map",
                            "Stopping..."]
        self.error_msg = ""
        self.mode = CollectionMode.IDLE
        self.lock = RLock()
        self.launch_handle = None
        self.bag_thread = None
        self.req = None                         # Incoming service message
        self.req_timeout = rospy.Duration(30)   # Request timeout
        
        rospy.on_shutdown(self.shutdown)

        # Services
        self.start_srv = rospy.Service('~start', Start, self.handleStart)
        self.end_srv = rospy.Service('~end', Trigger, self.handleEnd)

        #Publisher
        self.pub = rospy.Publisher('~status', String, queue_size=5)

    def handleStart(self, req):
        self.lock.acquire()

        if self.mode <= CollectionMode.ERROR:
            self.req = req
            init_mode = (CollectionMode.INITIATE_COVERAGE if req.map 
                         else CollectionMode.INITIATE_EXPLORING)
            self.mode = init_mode
            
            # Wait until complete or failed
            rate = rospy.Rate(5)
            start_t = rospy.Time.now()
            while ( self.mode == init_mode and 
                    (rospy.Time.now() - start_t) < self.req_timeout ):
                rate.sleep()

            success = (self.mode != CollectionMode.ERROR)
            msg = "" if success else self.error_msg
        else:
            success = False
            msg = "Robot is active, stop current action first."

        self.lock.release()

        return StartResponse(success=success, message=msg)

    def createDataDir(self, name):
        root_dir = rospy.get_param('data_dir', '/tmp/rosdata')
        time_str = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        dir_name = time_str if len(name) == 0 else "{}_{}".format(name, time_str)
        data_path = os.path.join(root_dir, dir_name)

        try:
            os.makedirs(root_dir)
        except OSError:
            pass #May already exist

        # Should not fail since each folder should be unique (if fail we want to exit)
        os.mkdir(data_path)

        self.data_dir = data_path

    def startCollection(self):
        rospy.loginfo("Starting data collection")

        # Create dir for result
        self.createDataDir(self.req.name)

        # Start nodes
        if self.mode == CollectionMode.INITIATE_EXPLORING:
            # Exploration
            self.launch_handle = launchPackage("robotslam_launcher",
                                               "launch/cartographer_exploration.launch")
            self.mode = CollectionMode.EXPLORING
        elif self.mode == CollectionMode.INITIATE_COVERAGE:
            # Coverage
            #TODO: Implement coverage
            self.launch_handle = launchPackage("robotslam_launcher",
                                               "launch/cartographer_exploration.launch")
            self.mode = CollectionMode.COVERAGE
        else:
            raise ValueError("Cannot start collection in mode {}".format(self.mode))

        if self.req.store_rosbag:
            # Start rosbag
            self.startBag(self.req.topics)

    def startBag(self, topics):
        #Use directory name as filename
        bagpath = os.path.join(self.data_dir, "{}.bag".format(os.path.basename(self.data_dir)))
        self.bag_thread = Process(target=rosbagWithCb, args=(bagpath, topics, self.bagCb))
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
        self.mode = CollectionMode.STOP
        # Wait until complete or failed
        rate = rospy.Rate(5)
        start_t = rospy.Time.now()
        while ( self.mode == CollectionMode.STOP and 
                (rospy.Time.now() - start_t) < self.req_timeout ):
            rate.sleep()

        success = (self.mode == CollectionMode.IDLE)
        msg = "" if success else self.error_msg

        return TriggerResponse(success=success, message=msg)

    def stopCollection(self, forced = False):
        rospy.loginfo("Stopping data collection")
        if not forced:
            self.lock.acquire()
        # Store map etc?

        # Stop processes
        rospy.loginfo("---Stopping launch")
        if self.launch_handle:
            self.launch_handle.shutdown()
            self.launch_handle = None

        rospy.loginfo("---Stopping bag")
        if self.bag_thread:
            self.bag_thread.terminate()
            #self.bag_thread.join()
            self.bag_thread = None

        # Restore mode
        rospy.loginfo("---restore mode")
        if self.mode != CollectionMode.ERROR:
            self.mode = CollectionMode.IDLE

        if not forced:
            self.lock.release()

    def publish_status(self):
        if self.mode != CollectionMode.ERROR:
            self.pub.publish(self.status_msgs[self.mode])
        else:
            self.pub.publish(self.error_msg)

    def state_transition(self):
        if ( self.mode == CollectionMode.INITIATE_COVERAGE or
             self.mode == CollectionMode.INITIATE_EXPLORING ):
            try:
                self.startCollection()
            except Exception as e:
                self.error_msg = str(e)
                raise e
        
        elif ( self.mode == CollectionMode.ERROR or 
               self.mode == CollectionMode.STOP ):
            self.stopCollection()

    def run(self):
        rate = rospy.Rate(1) # 1hz
        while not rospy.is_shutdown():
            self.state_transition()
            self.publish_status()
            rate.sleep()

    def shutdown(self):
        rospy.loginfo("Shutting down")
        if ( self.mode == CollectionMode.EXPLORING or 
             self.mode == CollectionMode.COVERAGE ):
            self.stopCollection(forced=True)

if __name__ == "__main__":
    try:
        dc = DataCollector()
        dc.run()
    except rospy.ROSInterruptException:
        pass
