#!/usr/bin/env python

import unittest
import numpy as np
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation

import tf
import tf2_ros
from sensor_msgs.msg import CameraInfo

import tesse_ros_bridge.utils

class TestUtilsOffline(unittest.TestCase):

    def test_parse_metadata_0(self):
        """Test correct metatdata parsing from xml message offline."""
        raw_data = ET.parse("data/metadata_0.xml")
        data_str = ET.tostring(raw_data.getroot())

        data = tesse_ros_bridge.utils.parse_metadata(data_str)
        self.assertEqual(data['position'], [-5.692576, 2.499105, 10.63836])
        self.assertEqual(data['quaternion'], [0, 0.5372996, 0, 0.8433914])
        self.assertEqual(data['velocity'], [0, -0.0004944276, 0])
        self.assertEqual(data['ang_vel'], [0, 0, 0])
        self.assertEqual(data['acceleration'], [0, 0.001516496, 0])
        self.assertEqual(data['ang_accel'], [0, 0, 0])
        self.assertEqual(data['time'], 7.935)
        self.assertEqual(data['collision_status'], False)

    def test_parse_cam_data_0(self):
        """Test corrrect camera metadata parsing from xml message offline."""
        raw_data = ET.parse('data/cam_data_0.xml')
        data_str = ET.tostring(raw_data.getroot())

        data = tesse_ros_bridge.utils.parse_cam_data(data_str)
        self.assertEqual(data['name'], 'depth')
        self.assertEqual(data['id'], 3)
        self.assertEqual(data['parameters']['height'], 480)
        self.assertEqual(data['parameters']['width'], 720)
        self.assertEqual(data['parameters']['fov'], 37.84929)
        self.assertTrue(np.allclose(data['position'], [0, 0.05, 0]))
        self.assertTrue(np.allclose(data['quaternion'], [0.5, -0.5,  0.5, -0.5]))
        self.assertEqual(data['draw_distance']['far'], 50)
        self.assertEqual(data['draw_distance']['near'], 0.3)

    def test_parse_lidar_data_0(self):
        """Test correct lidar metadata parsing from xml message offline."""
        raw_data = ET.parse('data/lidar_data_0.xml')
        data_str = ET.tostring(raw_data.getroot())

        data = tesse_ros_bridge.utils.parse_lidar_data(data_str)
        self.assertEqual(data['name'], 'trunk_Lidar2D')
        self.assertEqual(data['id'], 1)
        self.assertEqual(data['parameters']['max_range'], 200)
        self.assertEqual(data['parameters']['min_angle'], -3.92699)
        self.assertEqual(data['parameters']['max_angle'], 0.785398)
        self.assertEqual(data['parameters']['ray_count'], 1080)
        self.assertTrue(np.allclose(data['position'], [-0.17, -0.017, 0]))
        self.assertTrue(np.allclose(data['quaternion'], [0, 0, 0, 1]))

    def test_parse_object_data_0(self):
        raw_data = ET.parse('data/object_data_0.xml')
        data_str = ET.tostring(raw_data.getroot())

        data = tesse_ros_bridge.utils.parse_object_data(data_str)
        self.assertEqual(data[0]['type'], 'SMPL_Female_Autonomous')
        self.assertEqual(data[0]['time'], 423.6156)
        self.assertEqual(data[0]['id'], 0)
        self.assertEqual(data[0]['parameters'], {})
        self.assertTrue(np.allclose(data[0]['position'], [18.71819, 25.33417, 1.904896]))
        self.assertTrue(np.allclose(data[0]['quaternion'], [0.0, 0.0, -0.04361943562467316, 0.9990482194748085]))

    def test_metadata_to_odom_0(self):
        """Test correct conversion of metadata to Odometry message offline."""
        raw_data = ET.parse("data/metadata_0.xml")
        data_str = ET.tostring(raw_data.getroot())

        data = tesse_ros_bridge.utils.parse_metadata(data_str)

        odom = tesse_ros_bridge.utils.metadata_to_odom(data, 0, "f1", "f2")
        self.assertEqual(odom.header.stamp, 0)
        self.assertEqual(odom.header.frame_id, "f1")
        self.assertEqual(odom.child_frame_id, "f2")
        self.assertEqual(odom.pose.pose.position.x, data['position'][0])
        self.assertEqual(odom.pose.pose.position.y, data['position'][1])
        self.assertEqual(odom.pose.pose.position.z, data['position'][2])
        self.assertEqual(odom.pose.pose.orientation.x, data['quaternion'][0])
        self.assertEqual(odom.pose.pose.orientation.y, data['quaternion'][1])
        self.assertEqual(odom.pose.pose.orientation.z, data['quaternion'][2])
        self.assertEqual(odom.pose.pose.orientation.w, data['quaternion'][3])
        self.assertEqual(odom.twist.twist.linear.x, data['velocity'][0])
        self.assertEqual(odom.twist.twist.linear.y, data['velocity'][1])
        self.assertEqual(odom.twist.twist.linear.z, data['velocity'][2])
        self.assertEqual(odom.twist.twist.angular.x, data['ang_vel'][0])
        self.assertEqual(odom.twist.twist.angular.y, data['ang_vel'][1])
        self.assertEqual(odom.twist.twist.angular.z, data['ang_vel'][2])

    def test_metadata_to_imu_0(self):
        """Test correct conversion of metadata to Imu message offline."""
        raw_data = ET.parse("data/metadata_0.xml")
        data_str = ET.tostring(raw_data.getroot())

        data = tesse_ros_bridge.utils.parse_metadata(data_str)
        proc_dict = tesse_ros_bridge.utils.process_metadata(data, 0, [0,0,0], np.identity(3))

        imu = tesse_ros_bridge.utils.metadata_to_imu(proc_dict, 0, "f")

        self.assertEqual(imu.header.frame_id, "f")
        self.assertEqual(imu.header.stamp, 0)
        self.assertEqual(imu.angular_velocity.x, proc_dict['ang_vel'][0])
        self.assertEqual(imu.angular_velocity.y, proc_dict['ang_vel'][1])
        self.assertEqual(imu.angular_velocity.z, proc_dict['ang_vel'][2])
        self.assertEqual(imu.linear_acceleration.x,
            proc_dict['acceleration'][0])
        self.assertEqual(imu.linear_acceleration.y,
            proc_dict['acceleration'][1])
        self.assertEqual(imu.linear_acceleration.z,
            proc_dict['acceleration'][2] + 9.81)

        # TODO(marcus): add checks on angular velocity between two frames

    def test_generate_camera_info(self):
        """Test generation of CameraInfo messages for one camera."""
        raw_data = ET.parse('data/cam_data_0.xml')
        data_str = ET.tostring(raw_data.getroot())

        data = tesse_ros_bridge.utils.parse_cam_data(data_str)
        
        info = tesse_ros_bridge.utils.generate_camera_info(data, "test")
        self.assertEqual(info.header.frame_id, "test")
        self.assertEqual(info.width, data['parameters']['width'])
        self.assertEqual(info.height, data['parameters']['height'])

        # TODO(marcus): add more checks

    def test_generate_camera_info_stereo(self):
        """Test generation of CameraInfo messages for left and right cameras."""
        raw_data = ET.parse('data/cam_data_0.xml')
        data_str = ET.tostring(raw_data.getroot())

        data = tesse_ros_bridge.utils.parse_cam_data(data_str)

        (left, right) = tesse_ros_bridge.utils.generate_camera_info_stereo(data, data)
        self.assertEqual(left.header.frame_id, "left_cam")
        self.assertEqual(right.header.frame_id, "right_cam")
        self.assertEqual(left.width, data['parameters']['width'])
        self.assertEqual(left.height, data['parameters']['height'])
        self.assertEqual(right.width, data['parameters']['width'])
        self.assertEqual(right.height, data['parameters']['height'])

        # TODO(marcus): add more checks

    def test_vfov_from_hfov(self):
        """Test proper generation of vertical FOV given horizontal FOV."""
        width = 700
        height = 480
        hfov = 60

        # TODO(marcus): make sure these expected values are correct!
        actual = tesse_ros_bridge.utils.vfov_from_hfov(hfov, width, height)
        expected = 43.19696059328124
        self.assertEqual(actual, expected)

    def test_f_from_hfov(self):
        """Test focal length generation from FOV."""
        width = 700
        height = 480
        hfov = 60
        vfov = 60

        # TODO(marcus): make sure these expected values are correct!
        actual = tesse_ros_bridge.utils.fx_from_hfov(hfov, width)
        expected = 606.2177826491071
        self.assertEqual(actual, expected)

        actual = tesse_ros_bridge.utils.fy_from_vfov(vfov, height)
        expected = 415.69219381653056
        self.assertEqual(actual, expected)

    def test_make_camera_info_msg(self):
        """Test generation of CameraInfo message for one camera"""
        # TODO(marcus): complete
        pass

    def test_right_handed_frame_0(self):
        """Test rotation matrices to ensure they are right handed offline."""
        raw_data = ET.parse("data/metadata_0.xml")
        data_str = ET.tostring(raw_data.getroot())

        data = tesse_ros_bridge.utils.parse_metadata(data_str)
        quat = np.array(data['quaternion'])
        rot = tf.transformations.quaternion_matrix(quat)
        self.assertEqual(np.linalg.det(rot), 1)

    def test_process_metadata_0(self):
        """Test correct transformation of Unity metadata at rest, offline."""
        raw_data = ET.parse("data/metadata_0.xml")
        data_str = ET.tostring(raw_data.getroot())

        pre = tesse_ros_bridge.enu_T_unity
        post = tesse_ros_bridge.brh_T_blh
        post_2 = tesse_ros_bridge.bros_T_brh

        data = tesse_ros_bridge.utils.parse_metadata(data_str)
        proc = tesse_ros_bridge.utils.process_metadata(data, data['time']-2,
            [0,0,0], np.identity(3))

        transform = proc['transform']
        transform_R = transform[:3,:3]
        transform_t = transform[:3,3]

        # First check the transformation matrix.
        # Right-handed check.
        self.assertEqual(np.linalg.det(transform_R), 1)
        # X and Z axes are switched:
        self.assertEqual(transform_t[0], data['position'][0])
        self.assertEqual(transform_t[1], data['position'][2])
        self.assertEqual(transform_t[2], data['position'][1])

        truth_quat = tf.transformations.quaternion_from_matrix(
            (pre.dot(post)).dot(
                tf.transformations.quaternion_matrix(
                    data['quaternion']
                )
            ).dot(np.transpose(post_2))
        )
        self.assertTrue(np.allclose(proc['quaternion'], truth_quat))

        truth_vel = (post_2[:3,:3].dot(post[:3,:3])).dot(data['velocity'])
        self.assertTrue(np.allclose(proc['velocity'], truth_vel))

        # TODO(marcus): this is not correct.
        # truth_angvel = data['ang_vel']
        # print('\n')
        # print(truth_angvel)
        # print(proc['ang_vel'])
        # self.assertTrue(np.allclose(proc['ang_vel'], truth_angvel))

        self.assertTrue(np.allclose(proc['acceleration'], proc['velocity']*0.5))

        self.assertEqual(proc['time'], data['time'])
        self.assertEqual(proc['collision_status'], data['collision_status'])

    def test_process_metadata_1(self):
        """Test correct transformation of Unity metadata in motion, offline."""
        data_1 = ET.parse("data/metadata_1.xml")
        data_2 = ET.parse("data/metadata_2.xml")
        data_1_str = ET.tostring(data_1.getroot())
        data_2_str = ET.tostring(data_2.getroot())

        enu_T_unity= tesse_ros_bridge.enu_T_unity
        brh_T_blh = tesse_ros_bridge.brh_T_blh

        dict_1 = tesse_ros_bridge.utils.parse_metadata(data_1_str)
        dict_2 = tesse_ros_bridge.utils.parse_metadata(data_2_str)
        proc_1 = tesse_ros_bridge.utils.process_metadata(dict_1, 0, [0,0,0], np.identity(3))
        proc_2 = tesse_ros_bridge.utils.process_metadata(dict_2, dict_1['time'],
            proc_1['velocity'], np.identity(3))

        prev_enu_T_brh = proc_1['transform']
        enu_T_brh = proc_2['transform']
        prev_enu_T_brh[:,3] = enu_T_brh[:,3] = np.array([0,0,0,1])

        prev_unity_T_brh = brh_T_blh.dot(
            tf.transformations.quaternion_matrix(dict_1['quaternion']))
        unity_T_brh = brh_T_blh.dot(
            tf.transformations.quaternion_matrix(dict_2['quaternion']))

        dt = dict_2['time'] - dict_1['time']
        expected_ang_vel = Rotation.from_quat(
            tf.transformations.quaternion_from_matrix(np.transpose(
                prev_enu_T_brh).dot(enu_T_brh))).as_rotvec() / dt
        actual_ang_vel = proc_2['ang_vel']

        # print "\nexpected ang_vel: ", expected_ang_vel
        # print "actual ang_vel:   ", actual_ang_vel

        # TODO(marcus): this is not correct
        # self.assertTrue(np.allclose(expected_ang_vel, actual_ang_vel))

        expected_accel = (proc_2['velocity'] - proc_1['velocity']) / \
            (proc_2['time']-proc_1['time'])
        actual_accel = proc_2['acceleration']
        self.assertTrue(np.allclose(expected_accel, actual_accel))

        # TODO(marcus): add a test for angular rates in all three axes

    def test_acceleration_from_metadata(self):
        data_1 = ET.parse("data/metadata_3.xml")
        data_2 = ET.parse("data/metadata_4.xml")
        data_3 = ET.parse("data/metadata_5.xml")
        data_4 = ET.parse("data/metadata_6.xml")
        data_1_str = ET.tostring(data_1.getroot())
        data_2_str = ET.tostring(data_2.getroot())
        data_3_str = ET.tostring(data_3.getroot())
        data_4_str = ET.tostring(data_4.getroot())

        enu_T_unity= tesse_ros_bridge.enu_T_unity
        brh_T_blh = tesse_ros_bridge.brh_T_blh

        dict_1 = tesse_ros_bridge.utils.parse_metadata(data_1_str)
        dict_2 = tesse_ros_bridge.utils.parse_metadata(data_2_str)
        dict_3 = tesse_ros_bridge.utils.parse_metadata(data_3_str)
        dict_4 = tesse_ros_bridge.utils.parse_metadata(data_4_str)

        proc_1 = tesse_ros_bridge.utils.process_metadata(dict_1, 0, [0,0,0], np.identity(3))
        proc_2 = tesse_ros_bridge.utils.process_metadata(dict_2, dict_1['time'], proc_1['velocity'], proc_1['transform'][:3,:3])
        proc_3 = tesse_ros_bridge.utils.process_metadata(dict_3, dict_2['time'], proc_2['velocity'], proc_2['transform'][:3,:3])
        proc_4 = tesse_ros_bridge.utils.process_metadata(dict_4, dict_3['time'], proc_3['velocity'], proc_3['transform'][:3,:3])

        np.set_printoptions(precision=8)

        assert(proc_1['time'] == dict_1['time'])
        assert(proc_2['time'] == dict_2['time'])
        assert(proc_3['time'] == dict_3['time'])
        assert(proc_4['time'] == dict_4['time'])

        assert(np.allclose(proc_2['time'], dict_1['time'] + 0.005))
        assert(np.allclose(proc_3['time'], dict_2['time'] + 0.005))
        assert(np.allclose(proc_4['time'], dict_3['time'] + 0.005))

        # Double differentiation to get acceleration
        dt_12 = proc_2['time'] - proc_1['time']
        dt_23 = proc_3['time'] - proc_2['time']

        expected_vel_2 = (proc_2['position'] - proc_1['position']) / dt_12
        expected_vel_3 = (proc_3['position'] - proc_2['position']) / dt_23

        expected_acc_3 = (expected_vel_3 - expected_vel_2) / dt_23
        actual_acc_3 = proc_3['acceleration']

        # We have calculated accel in world frame, now convert to body frame.
        expected_acc_3_brh = np.transpose(proc_3['transform'][:3,:3]).dot(expected_acc_3)

        self.assertTrue(np.allclose(expected_acc_3_brh, actual_acc_3, atol=1.e-1))

if __name__ == '__main__':
    unittest.main()
