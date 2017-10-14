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
        self.steering_correction_pid = pid.PID(kp=0.5, ki=0.004, kd=0.25, mn=-msa, mx=msa)

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
            corrected_steering_angle = self.steering_correction_pid.step(cte, duration)

            # get predicted steering angle from waypoints curve
            yaw_steer = self.yaw_controller.get_steering(twist_cmd_linear_velocity,
                            twist_cmd_angular_velocity, current_linear_velocity)

            # steering_angle = corrected_steering_angle + yaw_steer * PREDICTIVE_STEERING
            steering_angle =  yaw_steer * PREDICTIVE_STEERING


            return steering_angle
        else:
            self.steering_correction_pid.reset() # manual mode, reset pid

        return 0.0

    def control_speed_based_on_proportional_throttle_brake(self, target_linear_velocity,
        current_velocity, max_throttle_proportional, max_brake_proportional):
        """
        Manipulates throttle, brake based on difference between target and current
        linear velocity, limited by dbw_node parameters max_throttle_proportional
        and max_brake_proportional.

        """
        velocity_change_required = target_linear_velocity - current_velocity
        rospy.logwarn("------")
        rospy.logwarn("twist_controller.py: velocity_change_required %s = target_linear_velocity %s \
            - current_velocity %s", velocity_change_required, target_linear_velocity, current_velocity)

        throttle, brake = 0.0, 0.0
        if velocity_change_required > 0.1:
            # limit increase in throttle
            throttle, brake = min(velocity_change_required / target_linear_velocity, max_throttle_proportional), 0.0
            rospy.logwarn("increase throttle: throttle: %s = min(velocity_change_required %s / target_linear_velocity %s, \
            max_throttle_proportional %s)", throttle, velocity_change_required, target_linear_velocity, max_throttle_proportional)

        elif velocity_change_required < -0.1:
            # limit increase in brake
            throttle, brake = 0,0, min(velocity_change_required / target_linear_velocity, max_brake_proportional)
            rospy.logwarn("increase brake: brake: %s = min(velocity_change_required %s / target_linear_velocity %s, \
            max_brake_proportional %s)", brake, velocity_change_required, target_linear_velocity, max_brake_proportional)

        return throttle, brake
