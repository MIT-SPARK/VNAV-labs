#!/usr/bin/env python

import copy
import numpy as np

import rospy2 as rospy
import tf2_ros
import tf_transformations

class NoiseParams:
    """ Holds noise parameters for the noise simulator """

    def _init_(self, noise_params):
        """ Holds the noise params """
    def __init__(self):
        # Noise parameters
        self.position_noise_mu    = rospy.get_param("~position_noise_mu", 0.0)
        self.position_noise_sigma = rospy.get_param("~position_noise_sigma", 0.0)
        self.rotation_noise_mu    = rospy.get_param("~rotation_noise_mu", 0.0)
        self.rotation_noise_sigma = rospy.get_param("~rotation_noise_sigma", 0.0)
        self.linear_velocity_noise_mu    = rospy.get_param("~linear_velocity_noise_mu", 0.0)
        self.linear_velocity_noise_sigma = rospy.get_param("~linear_velocity_noise_sigma", 0.0)

        self.gyroscope_noise_density_mu     = rospy.get_param("~gyroscope_noise_density_mu", 0.0)
        self.gyroscope_noise_density        = rospy.get_param("~gyroscope_noise_density", 0.0)
        self.accelerometer_noise_density_mu = rospy.get_param("~accelerometer_noise_density_mu", 0.0)
        self.accelerometer_noise_density    = rospy.get_param("~accelerometer_noise_density", 0.0)

        # Bias parameters
        self.gyroscope_bias_correlation_time     = rospy.get_param("~gyroscope_bias_correlation_time", 0.0)
        assert(self.gyroscope_bias_correlation_time > 0.0)
        self.gyroscope_bias_random_walk_mu       = rospy.get_param("~gyroscope_bias_random_walk_mu", 0.0)
        self.gyroscope_bias_random_walk          = rospy.get_param("~gyroscope_bias_random_walk", 0.0)
        self.accelerometer_bias_correlation_time = rospy.get_param("~accelerometer_bias_correlation_time", 0.0)
        assert(self.accelerometer_bias_correlation_time > 0.0)
        self.accelerometer_bias_random_walk_mu   = rospy.get_param("~accelerometer_bias_random_walk_mu", 0.0)
        self.accelerometer_bias_random_walk      = rospy.get_param("~accelerometer_bias_random_walk", 0.0)

class NoiseSimulator():
    """ A Noise Simulator
    It applies noise to the following elements in metadata:
        - position
        - orientation
        - velocity
        - IMU: accelerometer and gyroscope
    """

    def __init__(self, noise_params):
        """ Initializes the noise simulator by asking for the following params with [unit]:
            - rotation_noise_mu []
            - rotation_noise_sigma []
            - rotation_noise_mu []
            - rotation_noise_sigma []
            - position_noise_mu []
            - position_noise_sigma []
            - linear_velocity_noise_mu []
            - linear_velocity_noise_sigma []
        """
        self.rotation_noise_mu              = noise_params.rotation_noise_mu
        self.rotation_noise_sigma           = noise_params.rotation_noise_sigma
        self.position_noise_mu              = noise_params.position_noise_mu
        self.position_noise_sigma           = noise_params.position_noise_sigma
        self.linear_velocity_noise_mu       = noise_params.linear_velocity_noise_mu
        self.linear_velocity_noise_sigma    = noise_params.linear_velocity_noise_sigma

        self.gyroscope_noise_density_mu     = noise_params.gyroscope_noise_density_mu
        self.gyroscope_noise_density        = noise_params.gyroscope_noise_density
        self.accelerometer_noise_density_mu = noise_params.accelerometer_noise_density_mu
        self.accelerometer_noise_density    = noise_params.accelerometer_noise_density

        # Bias parameters
        self.gyroscope_bias_random_walk_mu       = noise_params.gyroscope_bias_random_walk_mu
        self.gyroscope_bias_random_walk          = noise_params.gyroscope_bias_random_walk
        self.gyroscope_bias_correlation_time     = noise_params.gyroscope_bias_correlation_time
        self.accelerometer_bias_random_walk_mu   = noise_params.accelerometer_bias_random_walk_mu
        self.accelerometer_bias_random_walk      = noise_params.accelerometer_bias_random_walk
        self.accelerometer_bias_correlation_time = noise_params.accelerometer_bias_correlation_time

        # Init biases at 0
        self.gyroscope_bias = np.array([0., 0., 0.])
        self.accelerometer_bias = np.array([0., 0., 0.])
        self.prev_gyroscope_bias = np.array([0., 0., 0.])
        self.prev_accelerometer_bias = np.array([0., 0., 0.])

        # Init time at 0
        self.prev_time = 0.0

    def apply_noise_to_metadata(self, metadata):
        """ Apply noise to metadata.

            Configured by the `noise_enable` and `noise_<quantity>_{mu,sigma}`
            params. For position, orientation, and linear velocity, noise is
            sampled once from a gaussian distribution and applied. This primarily
            affects odometry. For angular rates and linear acceleration, a
            time-varying bias with a fixed dt is applied, followed by sampled
            white noise.
            Note that noise is applied independently to each axis for all fields.

            Args:
                metadata: A processed (ROS frame) metadata dictionary.

            Returns:
                Deep copy of metadata with noise applied.
        """
        metadata_noisy = copy.deepcopy(metadata)

        # Apply pose and linear velocity noise (odometry):
        i_noise, j_noise, k_noise = np.random.normal([self.rotation_noise_mu]*3,
                                                     [self.rotation_noise_sigma]*3)
        metadata_noisy['quaternion'] = tf_transformations.quaternion_multiply(
            metadata_noisy['quaternion'],
            tf_transformations.quaternion_from_euler(i_noise, j_noise, k_noise)
        )
        metadata_noisy['position']     += np.random.normal([self.position_noise_mu]*3,
                                                           self.position_noise_sigma)
        metadata_noisy['velocity']     += np.random.normal([self.linear_velocity_noise_mu]*3,
                                                           self.linear_velocity_noise_sigma)

        # Add noise to IMU
        if self.prev_time > 0:
            if self.prev_time == metadata_noisy['time']:
                # Something wrong is going on... Times should not be the same
                # If this happens, we should maybe even consider dropping the msg
                print("WARN: 0 dt btw IMU measurements, not applying noise...")
            else:
                self.add_noise_to_imu(metadata_noisy)

        self.prev_time = metadata_noisy['time']
        return metadata_noisy

    def add_noise_to_imu(self, metadata_noisy):
        # First, advance imu bias forward in time.
        # We model the IMU as being driven by white noise and a bias
        # modelled using a first-order Gauss-Markov process
        dt = metadata_noisy['time'] - self.prev_time
        sqrt_dt = np.sqrt(dt)

        # Advance Gyro bias
        sigma_b_g_c = self.gyroscope_bias_random_walk
        tau_g = self.gyroscope_bias_correlation_time
        # State transition for gyro in discrete setting:
        phi_g_d = np.exp(-dt / tau_g)
        # Exact covariance of the process: eq.14 in Luca's notes (check also Maybeck 4-114)
        # Approximation assumes dt ~ 0 and results in: sigma_b_g_d = sigma_b_g_c * sqrt_dt
        sigma_b_g_d = sigma_b_g_c * np.sqrt(tau_g / 2.0 * (1.0 - np.exp(-2.0 * dt / tau_g)))
        # Gauss-Markov
        self.gyroscope_bias = phi_g_d * self.gyroscope_bias + \
                np.random.normal([self.gyroscope_bias_random_walk_mu]*3, sigma_b_g_d)

        # Advance Acc bias
        sigma_b_a_c = self.accelerometer_bias_random_walk
        tau_a = self.accelerometer_bias_correlation_time
        # State transition for acc in discrete setting:
        phi_a_d = np.exp(-dt / tau_a)
        # Exact covariance of the process: eq.14 in Luca's notes (check also Maybeck 4-114)
        # Approximation assumes dt ~ 0 and results in: sigma_b_a_d = sigma_b_a_c * sqrt_dt
        sigma_b_a_d = sigma_b_a_c * np.sqrt(tau_a / 2.0 * (1.0 - np.exp(-2.0 * dt / tau_a)))
        self.accelerometer_bias = phi_a_d * self.accelerometer_bias + \
                np.random.normal([self.accelerometer_bias_random_walk_mu]*3, sigma_b_a_d)

        # Apply imu bias: see eq. 101 in Crassidis05 http://www.acsu.buffalo.edu/~johnc/gpsins_gnc05.pdf
        # Although that equation is meant for IMU modelled as a Wiener process,
        # while we model our IMU as a Gauss-Markov process here
        metadata_noisy['ang_vel']      += (self.gyroscope_bias + self.prev_gyroscope_bias) / 2.0
        metadata_noisy['acceleration'] += (self.accelerometer_bias + self.accelerometer_bias) / 2.0
        self.prev_gyroscope_bias       = self.gyroscope_bias
        self.prev_accelerometer_bias   = self.accelerometer_bias

        # Apply white noise to imu, ensure dt is non-zero
        assert(sqrt_dt != 0.0)
        metadata_noisy['ang_vel']      += np.random.normal([self.gyroscope_noise_density_mu]*3,
                                                           self.gyroscope_noise_density / sqrt_dt)
        metadata_noisy['acceleration'] += np.random.normal([self.accelerometer_noise_density_mu]*3,
                                                           self.accelerometer_noise_density / sqrt_dt)
        return metadata_noisy
