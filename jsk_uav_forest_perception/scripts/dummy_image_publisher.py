#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

class DummyImagePublisher:
    def __init__(self):
        img_pub = rospy.Publisher("/dummy_image", Image, queue_size = 1)

        r = rospy.Rate(30)
        img_msg = Image()
        img_msg.height = 1
        img_msg.width = 1
        img_msg.encoding = 'rgb8'
        img_msg.is_bigendian = 0
        img_msg.step = 3
        img_msg.data = [0] * 1 * 3

        while not rospy.is_shutdown():
            img_msg.header.stamp = rospy.Time.now()
            img_pub.publish(img_msg)
            r.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('dummy_image_publisher')
        node = DummyImagePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
