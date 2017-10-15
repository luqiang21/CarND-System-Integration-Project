#!/usr/bin/env python

import rospy
from tf import transformations
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

from math import cos, sin
from copy import deepcopy

'''
Most code learned from Andrew Wilkie andrew.d.wilkie@gmail.com.

This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
	"""
	*** STEP 2 ***
	This node  will publish waypoints from the car's current position to some distance ahead,
	with the correct target velocities, depending on traffic lights and obstacles.
	"""
	def __init__(self):
		rospy.init_node('waypoint_updater')

		rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
		rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

		rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

		rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)



		self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

		# TODO: Add other member variables you need below
		self.current_ego_pose = None # ego car current position and orientation
		self.base_waypoints = None
		self.traffic_lights = None
		self.frame_id = None

		self.stop_wp_ix = -1 # -1 represents no red traffic_light ahead

		self.current_linear_velocity = None
		self.current_angular_velocity = None

#        rospy.spin()
		self.loop()

	def loop(self):
		rate = rospy.Rate(2)

		while not rospy.is_shutdown():
			rate.sleep() # With the help of Rate's method sleep(), it offers a convenient way for looping at the desired rate. With its argument of 10, we should expect to go through the loop 10 times per second (as long as our processing time does not exceed 1/10th of a second!)

			if self.base_waypoints is None or self.current_ego_pose is None or self.frame_id is None:
				continue
			# get the closest waypoint index for car's current position.
			car_index = self.get_closest_waypoint_index(self.current_ego_pose, self.base_waypoints)

			# get the waypoints to look ahead
			lookahead_waypoints = self.get_next_waypoints(self.base_waypoints, car_index, LOOKAHEAD_WPS)

			# Publish, lane is the obtained waypoints with Lane type.
			lane = self.create_lane(self.frame_id, lookahead_waypoints)
			self.final_waypoints_pub.publish(lane)

			### debug ###
			rospy.logwarn("ego car position")
			rospy.logwarn(self.current_ego_pose.position)
			for i in range(len(lookahead_waypoints)-199):
				rospy.logfatal(i)
				rospy.logwarn(lookahead_waypoints[i].pose.pose.position)



			rospy.logwarn("ego car speed")
			rospy.logfatal(self.current_linear_velocity)
			rospy.logfatal(self.current_angular_velocity)
			rospy.logwarn("stop_wp_ix")
			rospy.logfatal(self.stop_wp_ix)
			# if red light, need to plan stop at stop line.
			if self.stop_wp_ix != -1:
				stop_wp = self.base_waypoints[stop_wp_ix]



	def pose_cb(self, msg):
		self.current_ego_pose = msg.pose # msg Type is PoseStamped, has header and pose.
		self.frame_id = msg.header.frame_id # TODO explain frame_id

	def waypoints_cb(self, msg):
		self.base_waypoints = msg.waypoints


	def traffic_cb(self, msg):
		self.stop_wp_ix = msg.data

	def current_velocity_cb(self, msg):
		self.current_linear_velocity = msg.twist.linear.x
		self.current_angular_velocity = msg.twist.angular.z


	def get_next_waypoints(self, waypoints, i, n):
		"""return a list of waypoints ahead of the ego car"""
		# n is the number of lookahead_waypoints
		m = min(len(waypoints), i + n)
		return deepcopy(waypoints[i:m]) # TODO why deepcoy?

	def get_closest_waypoint_index(self, pose, waypoints):
		""" return the index of the closest waypoint """
		best_distance = float('inf')
		best_waypoint_index = 0
		my_position = pose.position

		for i, waypoint in enumerate(waypoints):
			waypoint_position = waypoint.pose.pose.position
			# data structure refer to rosmsg info styx_msgs/Lane

			gap = self.get_distance_between_two_points(my_position, waypoint_position)

			if gap < best_distance:
				best_waypoint_index, best_distance = i, gap

		is_behind = self.is_waypoint_behind_ego_car(pose, waypoints[best_waypoint_index])
		if is_behind:
			best_waypoint_index += 1

		return best_waypoint_index

	def get_distance_between_two_points(self, a, b):
		''' return distance between two points '''
		# a, b type is position
		dx = a.x - b.x
		dy = a.y - b.y
		return dx * dx + dy * dy

	def is_waypoint_behind_ego_car(self, pose, waypoint):
		"""
		Do transformation that sets origin to the ego car position, oriented along
		x-axis, and return True if waypoint is behind, else False
		"""
		_, _, yaw = self.get_euler(pose)

		origin_x = pose.position.x
		origin_y = pose.position.y

		shift_x = waypoint.pose.pose.position.x - origin_x
		shift_y = waypoint.pose.pose.position.y - origin_y

		# TODO understand how comparsion is done.
		x = shift_x * cos(0 - yaw) - shift_y * sin(0 - yaw)

		if x > 0:
			return False

		return True

	def get_euler(self, pose):
		"""
		return roll(x), pitch(y), yaw(z) from a Quaternion
		"""
		return transformations.euler_from_quaternion(
			[pose.orientation.x, pose.orientation.y, pose.orientation.z,
			pose.orientation.w])

	def create_lane(self, frame_id, waypoints):
		new_lane = Lane()
		new_lane.header.frame_id = frame_id
		new_lane.waypoints = waypoints
		new_lane.header.stamp = rospy.Time.now()
		return new_lane




	def get_waypoint_velocity(self, waypoint):
		return waypoint.twist.twist.linear.x

	def set_waypoint_velocity(self, waypoints, waypoint, velocity):
		waypoints[waypoint].twist.twist.linear.x = velocity

	def distance(self, waypoints, wp1, wp2):
		dist = 0
		dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
		for i in range(wp1, wp2+1):
			dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
			wp1 = i
		return dist


if __name__ == '__main__':
	try:
		WaypointUpdater()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start waypoint updater node.')
