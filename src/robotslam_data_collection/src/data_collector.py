#!/usr/bin/env python
import os
from robotslam_data_collection.srv import Start, StartResponse
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String
from threading import RLock, Timer
from multiprocessing import Process
import roslaunch
from datetime import datetime
import rosbag
import subprocess
import signal
from visualization_msgs.msg import MarkerArray

import rospy
import rospkg

class CollectionMode:
    IDLE = 0
    IDLE_ERROR = 1
    ERROR = 2
    INITIATE_EXPLORING = 3
    INITIATE_COVERAGE = 4
    EXPLORING = 5
    COVERAGE = 6
    STOP = 7

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

def terminate_process_and_children(p):
  import psutil
  process = psutil.Process(p.pid)
  for sub_process in process.children(recursive=True):
      sub_process.send_signal(signal.SIGINT)
  p.wait()  # we wait for children to terminate

def killProc(proc, timeout):
    timeout["value"] = True
    proc.kill()

def writeTrajectory(path, traj_msg):
    with open(path, "wb") as f:
        twriter = csv.writer(f, delimiter=',')
        twriter.writerow(["version", "1"])
        for p in traj_msg.points:
            twriter.writerow([str(p.x), str(p.y)])


class DataCollector:

    def __init__(self):
        rospy.init_node('data_collector')

        self.status_msgs = ["Idle",
                            "Error",
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
        self.data_dir = None

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
        if self.bag_thread != None:
            rospy.loginfo("Rosbag already running, killing previous rosbag.")
            self.killRosbag()

        #Use directory name as filename
        bagpath = os.path.join(self.data_dir, "{}.bag".format(os.path.basename(self.data_dir)))
        self.bag_thread = subprocess.Popen(["rosbag", "record", "-O", bagpath, "--lz4"] + topics)
        rospy.loginfo("Rosbag started, recording to {}".format(bagpath))

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

        # Store map and stuff
        self.saveTrajectory()
        self.saveMap()

        # Stop processes
        rospy.loginfo("---Stopping launch")
        if self.launch_handle:
            self.launch_handle.shutdown()
            self.launch_handle = None

        rospy.loginfo("---Stopping bag")
        self.killRosbag()

        # Restore mode
        rospy.loginfo("---restore mode")
        if self.mode == CollectionMode.ERROR:
            self.mode = CollectionMode.IDLE_ERROR
        else:
            self.mode = CollectionMode.IDLE

        if not forced:
            self.lock.release()

    def saveMap(self):
        # Give map saver 10 seconds to save the map.
        map_path = os.path.join(self.data_dir, os.path.basename(self.data_dir))
        proc = subprocess.Popen(["rosrun", "map_server", "map_saver", "-f", map_path])
        timeout = {"value": False}
        timer = Timer(10, killProc, [proc, timeout])
        timer.start()
        stdout,stderr = proc.communicate()
        timer.cancel()
        if timeout["value"]:
            self.mode = CollectionMode.ERROR
            self.error_msg = "Could not save map"

    def saveTrajectory(self):
        timeout = False
        try:
            traj = rospy.wait_for_message("/trajectory_node_list", MarkerArray, timeout = 10)
        except rospy.ROSException:
            timeout = True
            self.mode = CollectionMode.ERROR
            self.error_msg = "No trajectory published on topic /trajectory_node_list"

        if not timeout:
            try:
                traj_path = os.path.join(self.data_dir, "traj.csv")
                writeTrajectory(traj_path, traj)
            except Exception:
                self.mode = CollectionMode.ERROR
                self.error_msg = "Could not write trajectory"

    def publishStatus(self):
        if ( self.mode != CollectionMode.ERROR and
             self.mode != CollectionMode.IDLE_ERROR ):
            self.pub.publish(self.status_msgs[self.mode])
        else:
            self.pub.publish(self.error_msg)

    def stateTransition(self):
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

    def manageRosbag(self):
        # Make sure it is alive if it exists
        if self.bag_thread != None:
            rcode = self.bag_thread.poll()
            if rcode != None:
                rospy.loginfo("Rosbag terminated for unkown reason, stopping...")
                self.bag_thread = None
                self.mode = CollectionMode.ERROR
                self.error_msg = "Rosbag terminated early with error code {}".format(rcode)

    def killRosbag(self):
        if self.bag_thread:
            terminate_process_and_children(self.bag_thread)
            self.bag_thread = None

    def run(self):
        rate = rospy.Rate(1) # 1hz
        while not rospy.is_shutdown():
            self.stateTransition()
            self.manageRosbag()
            self.publishStatus()
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
