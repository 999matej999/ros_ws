#!/usr/bin/env python
import rospy
from std_msgs.msg import Header

def callback(message):
   
    now = rospy.Time.now()
    delta_t = (now.secs - message.stamp.secs)*1000000000 + (now.nsecs - message.stamp.nsecs)
   
    print('seq: ' + str(message.seq))
    print('frame id: ' + message.frame_id)
    print('send time: ' + str(message.stamp.secs) + '.' + str(message.stamp.nsecs).zfill(9) + 's')
    print('receive time: ' + str(now.secs) + '.' + str(now.nsecs).zfill(9) + 's')
    print('delay [ns]: ' + str(delta_t))
    print(30*'*')

def listener():

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/my_topic", Header, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

