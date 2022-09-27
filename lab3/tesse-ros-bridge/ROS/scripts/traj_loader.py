#!/usr/bin/env python

import pandas as pd
import time

import rospy
from geometry_msgs.msg import Twist


class TrajectoryLoader:

    def __init__(self):
        self.file_path = rospy.get_param("~traj_filepath")
        self.drive_topic = rospy.get_param("~drive_topic")
        self.clock_rate = rospy.get_param("~clock_rate")

        self.df = pd.read_csv(self.file_path, header=0)
        self.df.set_index("stamp:", drop=True, inplace=True)

        self.drive_pub = rospy.Publisher(self.drive_topic, Twist, queue_size=10)
    
    def spin(self):
        while self.drive_pub.get_num_connections() < 1:
            continue

        print("TrajectoryLoader: Beginning trajectory playback", )
        for index, row in self.df.iterrows():
            t = time.time()
            
            # ros_ts = rospy.Time.from_sec(index)
            twist_st = Twist()
            twist_st.linear.x = row['force_x']
            twist_st.linear.y = row['force_y']
            twist_st.angular.z = row['torque_z']
            
            self.drive_pub.publish(twist_st)

            dt = time.time() - t
            time.sleep((1.0 / self.clock_rate) - dt)
        
        print("TrajectoryLoader: Trajectory playback complete", )


if __name__ == "__main__":
    rospy.init_node("TrajectoryLoader_node")
    node = TrajectoryLoader()
    node.spin()
