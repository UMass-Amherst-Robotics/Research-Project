#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

teleop_velocity = Twist()

def teleop_update_velocity(new_velocity):
  rospy.loginfo("{} I heard {}".format(rospy.get_caller_id(), new_velocity))
  global teleop_velocity
  teleop_velocity = new_velocity


def main():
  rospy.init_node("robot_marker")
  pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
  publish_rate = rospy.Rate(30)
  connection_check_rate = rospy.Rate(1)
  rospy.Subscriber("cmd_vel", Twist, teleop_update_velocity)
  rospy.loginfo("Node started")

  # Wait for subscribers
  while not rospy.is_shutdown() and pub.get_num_connections() < 1:
    connection_check_rate.sleep()

  rospy.loginfo("Starting to publish")
  dt = 1 / 30
  global teleop_velocity
  # Note that all numerical values in the pose object are 0 by default
  pose = Pose()
  pose.orientation.w = 1.0

  while not rospy.is_shutdown() and pub.get_num_connections() >= 1:
    marker = Marker()
    # Fill in the marker objects properties
    marker.header.frame_id = "map"
    marker.id = 0
    marker.ns = "robot"
    marker.action = marker.ADD
    marker.lifetime = rospy.Duration(0)
    marker.type = marker.CUBE

    # Note that all numerical values in the marker object are 0 by default
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0

    marker.color.a = 1.0
    marker.color.r = 0.5

    marker.header.stamp = rospy.Time.now()

    # Update the marker using the teleop velocity
    # x_1 + v * dt = x_2
    pose.position.x += teleop_velocity.linear.x * dt
    pose.position.y += teleop_velocity.linear.y * dt
    pose.position.z += teleop_velocity.linear.z * dt

    roll, pitch, yaw = euler_from_quaternion([pose.orientation.x, pose.orientation.y,\
    pose.orientation.z, pose.orientation.w])
    roll += teleop_velocity.angular.x * dt
    pitch += teleop_velocity.angular.y * dt
    yaw += teleop_velocity.angular.z * dt

    q = quaternion_from_euler(roll, pitch, yaw)
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

    marker.pose.position.x = pose.position.x
    marker.pose.position.y = pose.position.y
    marker.pose.position.z = pose.position.z

    marker.pose.orientation.x = pose.orientation.x
    marker.pose.orientation.y = pose.orientation.y
    marker.pose.orientation.z = pose.orientation.z
    marker.pose.orientation.w = pose.orientation.w

    pub.publish(marker)
    publish_rate.sleep()

  rospy.loginfo("Shuting down")


if __name__ == "__main__":
  main()
