import rospy

from yaw_controller import YawController
import pid

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

PREDICTIVE_STEERING = 1.0

class Controller(object):
    """
    *** STEP 4 ***


    Adjusts vehicles steering to minimize cross tracking error and align to the predicted path
    """

    def __init__(self, wheel_base_from_ros_param_server, steer_ratio_from_ros_param_sever,
                max_lat_accel_from_ros_param_server, max_steer_angle_from_ros_param_server):
        """
        Take twist data as input to initialize pid and yaw_controller
        """

        # create yaw controller
        wb = wheel_base_from_ros_param_server
        sr = steer_ratio_from_ros_param_sever
        mla = max_lat_accel_from_ros_param_server
        msa = max_steer_angle_from_ros_param_server

        self.yaw_controller = YawController(wb, sr, 0.0, mla, msa)

        # create / fine tune pid steering controller
        self.steering_correction_pid = pid.PID（kp=0.5, ki=0.004, kd=0.25, mn=-msa, mx=msa)

        self.timestamp = rospy.get_time()

    def reset(self):
        self.steering_correction_pid.reset()

    def control(self, cte, dbw_enabled, twist_cmd_linear_velocity,
                twist_cmd_angular_velocity, current_linear_velocity):
        """
        Combine PID and yaw controller to adjust steering for given reference path and
        target velocities
        """

        new_timestamp = rospy.get_time()
        duration = new_timestamp - self.timestamp

        self.timestamp = new_timestamp
        if dbw_enabled:
            # calculate new steering angle
            corrected_steering_angle = self.steering_correction_pid(cte, duration)

            # get predicted steering angle from waypoints curve
            yaw_steer = self.yaw_controller.get_steering(twist_cmd_linear_velocity,
                            twist_cmd_angular_velocity, current_linear_velocity)

            steering_angle = corrected_steering_angle + yaw_steer * PREDICTIVE_STEERING

            return steering_angle
        else:
            self.steering_correction_pid.reset() # manual mode, reset pid

        return 0.0

    def control_speed_based_on_proportional_throttle_brake(self, target_velocity,
        current_linear_velocity, max_throttle, max_brake):

        return 0.8, 0.0
