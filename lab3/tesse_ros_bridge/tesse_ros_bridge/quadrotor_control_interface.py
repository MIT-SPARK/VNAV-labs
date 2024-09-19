#!/usr/bin/env python

import numpy as np
import copy

import rospy2 as rospy
#import tf
import tf2_ros
from std_msgs.msg import Header, String
from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.msg import Imu, CameraInfo
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators
from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    Point,
    PointStamped,
    TransformStamped,
    Quaternion,
    Twist,
    PoseWithCovarianceStamped,
    Vector3Stamped,
)
from ackermann_msgs.msg import AckermannDriveStamped
from rosgraph_msgs.msg import Clock
from cv_bridge import CvBridge, CvBridgeError

import tesse_ros_bridge.utils
from tesse_ros_bridge.noise_simulator import NoiseParams, NoiseSimulator

from tesse_msgs.srv import (
    SceneRequestService,
    ObjectSpawnRequestService,
    RepositionRequestService,
)
from tesse_msgs.msg import CollisionStats

from tesse_ros_bridge.consts import *

from tesse.msgs import *
from tesse.env import *
from tesse.utils import *


class TesseQuadrotorControlInterface:
    def __init__(self):
        """This class provides a ROS interface for controlling TESSE quadrotor agents.
        ROS users can simply send propeller speeds command to predefined propeller speeds
        topic, and this interface will transmit the appropriate messages to TESSE simulator.

        Quadrotor agent is controlled by setting the speed of each propeller. The speeds are radians per second.
        """
        # Networking parameters
        self.sim_ip = rospy.get_param("~sim_ip", "127.0.0.1")
        self.self_ip = rospy.get_param("~self_ip", "127.0.0.1")
        self.use_broadcast = rospy.get_param("~use_broadcast", False)
        self.position_port = rospy.get_param("~position_port", 9000)
        self.metadata_port = rospy.get_param("~metadata_port", 9001)
        self.image_port = rospy.get_param("~image_port", 9002)
        self.udp_port = rospy.get_param("~udp_port", 9004)
        self.step_port = rospy.get_param("~step_port", 9005)
        self.scan_port = rospy.get_param("~lidar_port", 9006)
        self.scan_udp_port = rospy.get_param("~lidar_udp_port", 9007)

        # Topics
        self.props_speeds_topic = "rotor_speed_cmds"

        # Initialize the Env object to communicate with simulator
        self.env = Env(
            simulation_ip=self.sim_ip,
            own_ip=self.self_ip,
            position_port=self.position_port,
            metadata_port=self.metadata_port,
            image_port=self.image_port,
            step_port=self.step_port,
        )

        # setup control interface subscriber
        self.props_speeds_sub = rospy.Subscriber(
            self.props_speeds_topic, Actuators, self.props_control_cb
        )

    def props_control_cb(self, msg):
        """Callback function used for propeller speed control

        :param msg: A mav_msgs.msg.Actuators message. The field angular_velocities is used for setting the propeller speeds.
        """
        # read prop speeds
        speeds = msg.angular_velocities
        self.env.send(PropSpeeds(speeds[0],speeds[1],speeds[2],speeds[3]))
        #rospy.loginfo(str(speeds))

def main():
    rospy.init_node("TesseQuadrotorControlInterface_node")
    node = TesseQuadrotorControlInterface()
    rospy.spin()

if __name__ == "__main__":
    main()
