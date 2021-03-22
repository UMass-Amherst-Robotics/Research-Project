#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker

def main():
  rospy.loginfo("Starting node")
  rospy.init_node("robot_marker")
  pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
  rate = rospy.Rate(1)
  rospy.loginfo("Node started")

  # Fill in the marker objects properties
  marker = Marker()
  marker.header.frame_id = "body"
  marker.id = 0
  marker.ns = "robot"
  marker.action = marker.ADD
  marker.lifetime = rospy.Duration()
  marker.type = marker.CUBE

  # Wait for subscribers
  while pub.get_num_connections() < 1:
    rate.sleep()

  rospy.loginfo("Starting to publish")
  while not rospy.is_shutdown():
    marker.header.stamp = rospy.Time.now()

    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 0.0

    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0

    marker.color.a = 1.0
    marker.color.r = 0.5
    marker.color.g = 0.0
    marker.color.b = 0.0

    pub.publish(marker)
    rate.sleep()

  rospy.loginfo("Shuting down")

if __name__ == "__main__":
  main()
