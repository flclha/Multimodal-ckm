#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image

class ImageStampRelay:
    def __init__(self):
        self.frame_id = rospy.get_param('~frame_id', 'uvc_camera')
        self.min_interval = float(rospy.get_param('~min_interval', 1.0))
        self.last_pub_wall_time = rospy.Time(0)
        self.pub = rospy.Publisher('/uvc_camera/image_sync', Image, queue_size=5)
        self.sub = rospy.Subscriber('/uvc_camera/image', Image, self.cb, queue_size=5)
        if self.min_interval > 0:
            rospy.loginfo('uvc_image_stamp_relay throttling to %.3f seconds between frames', self.min_interval)
        else:
            rospy.loginfo('uvc_image_stamp_relay throttling disabled')

    def cb(self, msg):
        now = rospy.Time.now()
        if self.min_interval > 0 and self.last_pub_wall_time != rospy.Time(0):
            if (now - self.last_pub_wall_time).to_sec() < self.min_interval:
                return

        out = Image()
        out.header = msg.header
        if out.header.stamp == rospy.Time(0):
            rospy.logwarn_throttle(
                1.0,
                'input image has zero header.stamp; falling back to rospy.Time.now()')
            out.header.stamp = now
        if not out.header.frame_id:
            out.header.frame_id = self.frame_id
        out.height = msg.height
        out.width = msg.width
        out.encoding = msg.encoding
        out.is_bigendian = msg.is_bigendian
        out.step = msg.step
        out.data = msg.data
        self.pub.publish(out)
        self.last_pub_wall_time = now

if __name__ == '__main__':
    rospy.init_node('uvc_image_stamp_relay')
    ImageStampRelay()
    rospy.spin()
