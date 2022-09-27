#!/usr/bin/env python

import pandas as pd

import rospy
from geometry_msgs.msg import Twist


class TrajectoryWriter:

    def __init__(self):
        self.file_path = rospy.get_param("~traj_filepath")
        self.drive_topic = rospy.get_param("~drive_topic")

        self.df = pd.DataFrame(
                columns=["stamp", "force_x", "force_y", "torque_z"]
        )

        rospy.Subscriber(self.drive_topic, Twist, self.drive_cb)
        rospy.on_shutdown(self.clean_up)

    def drive_cb(self, msg):
        force_x = msg.linear.x
        force_y = msg.linear.y
        torque_z = msg.angular.z
        stamp = rospy.Time.now()
        new_row = {"stamp:": stamp, "force_x": force_x, "force_y": force_y, "torque_z": torque_z}

        self.df = self.df.append(new_row, ignore_index=True)
    
    def spin(self):
        rospy.spin()

    def clean_up(self):
        print("TrajectoryWriter: Saving traj file to " + self.file_path)
        self.df.set_index("stamp", drop=True, inplace=True)
        self.df.to_csv(self.file_path, index=False, header=True)


if __name__ == "__main__":
    rospy.init_node("TrajectoryWriter_node")
    node = TrajectoryWriter()
    node.spin()
