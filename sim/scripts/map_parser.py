#!/usr/bin/env python3
import sys
from math import atan, pi
from os.path import isfile

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from tf_conversions import transformations

"""
The syntax for the map of obstacles is simple. It is of the form:
(x1,y1) -- (x2,y2) -- (w,h)
where the first two points are the endpoints of the obstacle and the third contains the width and
height of the obstacle. Obstacles are seperated by newlines.
"""

def main():
  rospy.init_node("map_parser")
  pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)
  check_rate = rospy.Rate(1) # 1 Hz

  # Get command line argument
  filename = rospy.get_param("~filename")
  print(filename)

  # Read in the file. Check if it exists first.
  map_data_raw = []
  map_markers = MarkerArray()
  print(map_markers)
  if not isfile(filename):
    sys.stderr.write("File \"{}\" does not exist.\n".format(filename))
  else:
    # Wait for subscribers
    while pub.get_num_connections() < 1:
      check_rate.sleep()

    # The "with ... as ..." syntax will automatically close the file once we've read it
    with open(filename, "r") as f:
      map_data_raw = f.readlines()
    print(map_data_raw)

    for i in range(len(map_data_raw)):
      component_strings = map_data_raw[i].split("--")
      # Convert the seperate tuple strings to tuples and put them in a list
      components = list(map(eval, component_strings))
      print(components)

      # Compute the midpoint and the distance between the endpoints. Also compute the vector that
      # connects the two endpoints
      dist = ((components[0][0] - components[1][0])**2 + \
      (components[0][1] - components[1][1])**2)**0.5
      midpoint = ((components[0][0] + components[1][0]) / 2, \
      (components[0][1] + components[1][1]) / 2)
      connecting_vector = (components[1][0] - components[0][0], components[1][1] - components[0][1])
      # We want to compute the angle between the x-axis and the connecting vector -- this will be
      # the yaw of the marker
      yaw = 0.0
      try:
        yaw = atan(connecting_vector[1] / connecting_vector[0])
      except ZeroDivisionError:
        # Recall, the lim as t approaches infinity of arctan(t) is pi/2
        yaw = pi / 2
      print("Distance: {}\nMidpoint: {}\nConnecting vector: {}\nAngle: {}".format(\
      dist,midpoint,connecting_vector,yaw))

      # Create the marker object
      marker = Marker()
      marker.header.frame_id = "map"
      marker.header.stamp = rospy.Time.now()
      marker.ns = "map_markers"
      marker.id = i
      marker.action = marker.ADD
      marker.lifetime = rospy.Duration()
      marker.type = marker.CUBE

      marker.pose.position.x = midpoint[0]
      marker.pose.position.y = midpoint[1]
      marker.pose.position.z = components[2][1] / 2

      q = transformations.quaternion_from_euler(0,0,yaw)
      marker.pose.orientation.x = q[0]
      marker.pose.orientation.y = q[1]
      marker.pose.orientation.z = q[2]
      marker.pose.orientation.w = q[3]

      marker.scale.x = dist
      marker.scale.y = components[2][0]
      marker.scale.z = components[2][1]

      marker.color.r = 0.8
      marker.color.g = 0.8
      marker.color.b = 0.8
      marker.color.a = 1.0

      rospy.logdebug(marker)
      map_markers.markers.append(marker)

    print(map_markers)
    pub.publish(map_markers)


if __name__ == "__main__":
  main()
