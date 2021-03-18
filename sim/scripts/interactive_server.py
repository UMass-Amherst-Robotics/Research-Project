#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import InteractiveMarkerServer, InteractiveMarker

def process_feedback(feedback):
  rospy.loginfo("{} is now at ({}, {}, {})".format( \
    feedback.marker_name, \
    feedback.pose.position.x, \
    feedback.pose.position.y, \
    feedback.pose.position.z \
  ))

def main():
  rospy.loginfo("Starting node")
  rospy.init_node("basic_marker")
  pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
  rate = rospy.Rate(1)
  rospy.loginfo("Node started")

  server = InteractiveMarkerServer("simple_marker")

  int_marker = InteractiveMarker()
  int_marker.header.frame_id = "base_link"
  int_marker.header.stamp = rospy.Time.now()
  int_marker.name = "my_marker"
  int_marker.description = "Simple 1-DOF Control"

  box_marker = Marker()
  box_marker.type = box_marker.CUBE
  box_marker.scale.x = 0.45
  box_marker.scale.y = 0.45
  box_marker.scale.z = 0.45
  box_marker.color.r = 0.313
  box_marker.color.b = 0.118
  box_marker.color.g = 0.118
  box_marker.color.a = 1.0

  box_control = InteractiveMarkerControl()
  box_control.always_visible = True
  box_control.markers.append(box_marker)

  int_marker.controls.append(box_control)

  rotate_control = InteractiveMarkerControl()
  rotate_control.name = "move_x"
  rotate_control.interaction_mode = rotate_control.MOVE_AXIS

  int_marker.controls.append(rotate_control)
  server.insert(int_marker, process_feedback)
  server.applyChanges()
  rospy.spin()

if __name__ == "__main__":
  main()
