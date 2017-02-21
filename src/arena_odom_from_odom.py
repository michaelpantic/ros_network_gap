#!/usr/bin/env python
# license removed for brevity
import rospy
import tf.transformations
import numpy as np
import time
import tf
from nav_msgs.msg import Odometry
enu_arena_trans = 0
enu_arena_rot = 0
enu_arena_mtx = 0
pub = 0
robot_name = 0
rate = 0

def odom_callback(msg_in):
	x = msg_in.pose.pose.position.x
	y = msg_in.pose.pose.position.y
	z = msg_in.pose.pose.position.z

	#apply tranformation
	vec_pose = np.array([[x],[y],[z],[1.0]])
	vec_pose_transformed = np.dot(enu_arena_mtx, vec_pose)
	vec_quat = np.array([msg_in.pose.pose.orientation.x,msg_in.pose.pose.orientation.y,msg_in.pose.pose.orientation.z,msg_in.pose.pose.orientation.w])
	vec_quat_transformed = tf.transformations.quaternion_multiply(enu_arena_rot, vec_quat)

	# publish
	msg = Odometry()
	msg.header.frame_id = "arena"
	msg.child_frame_id = robot_name
	msg.header.stamp = msg_in.header.stamp #copy stamp
	msg.pose.pose.position.x = vec_pose_transformed[0]
	msg.pose.pose.position.y = vec_pose_transformed[1]
	msg.pose.pose.position.z = vec_pose_transformed[2]

	msg.pose.pose.orientation.x = vec_quat_transformed[0]
	msg.pose.pose.orientation.y = vec_quat_transformed[1]
	msg.pose.pose.orientation.z = vec_quat_transformed[2]
	msg.pose.pose.orientation.w = vec_quat_transformed[3]

	msg.twist.twist.linear = msg_in.twist.twist.linear
	msg.twist.twist.angular = msg_in.twist.twist.angular

	pub.publish(msg)
	rate.sleep()

if __name__ == '__main__':
	try:
		rospy.init_node('arena_pos_publisher_pose', anonymous=True)
		odom_frame = rospy.get_param('~odom_frame', 'world')
		arena_frame = rospy.get_param('~arena_frame', 'arena')
		robot_name = rospy.get_param('~robot_name', 'robot')
		max_rate = int(rospy.get_param('~max_rate', 5))
		rate = rospy.Rate(max_rate)

		# set up TF listener
		rospy.logwarn("Reading TF from world to arena only at startup!")
		listener = tf.TransformListener()
		listener.waitForTransform(arena_frame, odom_frame, rospy.Time(), rospy.Duration(10.0))
		(enu_arena_trans, enu_arena_rot) = listener.lookupTransform(arena_frame, odom_frame, rospy.Time(0))
		enu_arena_mtx = listener.fromTranslationRotation(enu_arena_trans, enu_arena_rot)

		# remove listener, to not use wifi bandwith due to TF all the time (quite a lot!)
		del listener

		rospy.loginfo("Transform from " + odom_frame + " to " + arena_frame + " received")
		rospy.Subscriber("odometry_input", Odometry, odom_callback, queue_size = 1)

		# set up publisher
		pub = rospy.Publisher('arena_odometry', Odometry, queue_size=10)

		rospy.spin()

	except rospy.ROSInterruptException:
		pass
