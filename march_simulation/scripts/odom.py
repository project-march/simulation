#! /usr/bin/env python

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

rospy.init_node('odom_pub')

odom_pub = rospy.Publisher ('/my_odom', Odometry)
imu_broadcaster = tf2_ros.TransformBroadcaster()
rospy.wait_for_service ('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

odom = Odometry()
odom.header.frame_id='world'
odom.child_frame_id = 'imu_link'

model = GetModelStateRequest()
model.model_name='march'
r = rospy.Rate(5)
while not rospy.is_shutdown():
    r.sleep()
    result = get_model_srv(model)
    transform = TransformStamped()

    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = 'world'
    transform.child_frame_id = 'base_link'
    transform.transform.translation = result.pose.position
    transform.transform.rotation = result.pose.orientation

    imu_broadcaster.sendTransform(transform)
