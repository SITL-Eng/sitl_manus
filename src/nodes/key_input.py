#! /usr/bin/env python3

from readchar import readkey

import rospy
from sitl_dvrk.msg import BoolStamped

if __name__ == "__main__":
    rospy.init_node("glove_key_input")
    pub_on_off = rospy.Publisher('glove/left/on_off', BoolStamped, queue_size=10)
    rate = rospy.Rate(10)
    msg = BoolStamped()
    rospy.loginfo("Press one of the following keys...")
    rospy.loginfo("s: Turn on the gloves")
    rospy.loginfo("e: Turn off the gloves")
    try:
        while not rospy.is_shutdown():
            key = readkey()
            msg.header.stamp = rospy.Time.now()
            if key == "s":
                msg.data = True
                # pub_on_off.publish(msg)
            elif key == "e":
                msg.data = False
                # pub_on_off.publish(msg)
            rate.sleep()
    except Exception as e:
        rospy.logerr(e)
    finally:
        print("")