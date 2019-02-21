#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Pose

def talker():
    pose_msg = Pose()
    pose_msg.position.x = 1.0
    pose_msg.position.y = 1.0
    pose_msg.position.z = 0.0
    pose_msg.orientation.x = 0.7071068
    pose_msg.orientation.y = 0
    pose_msg.orientation.z = 0
    pose_msg.orientation.w = 0.7071068

    pub = rospy.Publisher('/pose', Pose, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(pose_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass