#!/usr/bin/env python

import rospy
from tf import transformations
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane

from math import cos, sin
import numpy as np

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''
POINTS_TO_FIT = 10
class DBWNode(object):
    """
    *** STEP 3 ****
    Actuate the throttle, steering and brake to successfully navigate the waypoints
    with the correct target velocity
    """
    def __init__(self):
        rospy.init_node('dbw_node')

        # variables
        self.current_ego_pose = None # ego car current position and orientation, type: pose

        # ROS server parameters
        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        # Publishers
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # Subscribe to all the topics you need to
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb, queue_size=1)
        rospy.Subscriber('/final_waypoints', Lane, self.waypoints_cb, queue_size=1)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb, queue_size=1)

        # Subscribed messages
        self.twist_cmd = None
        self.twist_cmd_linear_velocity = None
        self.twist_cmd_angular_velocity = None
        self.velocity = None
        self.current_linear_velocity = None
        self.current_angular_velocity = None
        self.dbw_enabled = False
        self.waypoints = None # final waypoints


        # TODO: Create `TwistController` object
        self.controller = Controller()

        self.loop()

    def loop(self):
        rate = rospy.Rate(10) # 50Hz
        rospy.logfatal("dbw1 node is here *****************************************")
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            data = [self.velocity, self.waypoints, self.current_ego_pose]
            all_variable = all([x is not None for x in data])

            if not all_variable:
                continue
            rospy.logfatal("dbw loop")
            rospy.logwarn("cte  "+str(self.cte_calc(self.current_ego_pose, self.waypoints)))

            throttle, brake, steering = self.controller.control()            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
            # if <dbw is enabled>:
            if 1:
                rospy.logfatal("publishing throttle, brake, and steering.")
                self.publish(throttle, brake, steering)
                rospy.logfatal("published.")

            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    def cte_calc(self, pose, waypoints):
        """
        Calculate the distance from the ego car's current position to the waypoints path
        """

        x_coords, y_coords = self.transform_waypoints(pose, waypoints, POINTS_TO_FIT)
        # 3 degree polynomial fitting
        coefficients = np.polyfit(x_coords, y_coords, 3)
        # distance between car position and transformed waypoint
        distance = np.polyval(coefficients, 5.0) # TODO why 5.0
        return distance

    def transform_waypoints(self, pose, waypoints, points_to_use=None):
        """
        Do transformation that sets the origin of waypoints to the ego car's position,
        oriented along x-axis and returns transformed coordinates.
        """

        x_coords = []
        y_coords = []

        _, _, yaw = self.get_euler(pose)
        origin_x = pose.position.x
        origin_y = pose.position.y

        if points_to_use is None:
            points_to_use = len(waypoints)

        for i in range(points_to_use):
            shift_x = waypoints[i].pose.pose.position.x - origin_x
            shift_y = waypoints[i].pose.pose.position.y - origin_y

            x = shift_x * cos(0 - yaw) - shift_y * sin(0 - yaw) # TODO why 0 - yaw?
            y = shift_x * sin(0 - yaw) + shift_y * cos(0 - yaw)

            x_coords.append(x)
            y_coords.append(y)
        #########################################
        rospy.logwarn(x_coords)
        rospy.logwarn(y_coords)
        ############################################
        return x_coords, y_coords

    def get_euler(self, pose):
		"""
		return roll(x), pitch(y), yaw(z) from a Quaternion
		"""
		return transformations.euler_from_quaternion(
			[pose.orientation.x, pose.orientation.y, pose.orientation.z,
			pose.orientation.w])

    ### Callback functions
    def pose_cb(self, message):
        self.current_ego_pose = message.pose

    def twist_cb(self, message):
        self.twist_cmd = message.twist
        # only x velocity is obtained
        self.twist_cmd_linear_velocity = message.twist.linear.x
        # only yaw is obtained since it is 2D space
        self.twist_cmd_angular_velocity = message.twist.angular.z

    def velocity_cb(self, message):
        self.velocity = message.twist
        self.current_linear_velocity = message.twist.linear.x
        self.current_angular_velocity = message.twist.angular.z

    def waypoints_cb(self, message):
        self.waypoints = message.waypoints

    def dbw_enabled_cb(self, message):
        """
        Enabled self-driving mode will publish throttle, steer and brake mode.
        """

        self.dbw_enabled = bool(message.data)
        if self.dbw_enabled:
            rospy.logwarn("**** ============================ ****")
            rospy.logwarn("**** Self-Driving Mode Activated  ****")
            rospy.logwarn("**** ============================ ****")
        else:
            rospy.logwarn("**** ============================= ****")
            rospy.logwarn("**** Manual Driving Mode Activated ****")
            rospy.logwarn("**** ============================= ****")


if __name__ == '__main__':
    DBWNode()
