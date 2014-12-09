#!/usr/bin/env python

import rospy
from std_msgs.msg import Time

def main():
  rospy.init_node('trigger_action_clock')
  pub = rospy.Publisher('clock', Time)
  r = rospy.Rate(1)
  while not rospy.is_shutdown():
    time = rospy.Time.now()
    msg = Time(time)
    pub.publish(msg)
    r.sleep()

if __name__ == '__main__':
  main()
