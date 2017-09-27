#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped


def main():
    myWaypoint = PoseStamped()
    pub = rospy.Publisher('goal_pose',PoseStamped,queue_size=1)
    rospy.init_node('User',anonymous=True)
    rate=rospy.Rate(1)

    while not rospy.is_shutdown():
        myWaypoint.position.x = input("Enter X: ")
        myWaypoint.position.y = input("Enter Y: ")
        pub.publish(myWaypoint)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass