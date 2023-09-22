#!/usr/bin/env python3
# ROS python API
import rospy

from geometry_msgs.msg import Point, PoseStamped

def get_pose_values():
    x = float(input("Enter X position: "))
    y = float(input("Enter Y position: "))
    z = float(input("Enter Z position: "))
    
    return x, y, z

def main():
    rospy.init_node('goal_node', anonymous=True)
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    goal_msg = PoseStamped()

    x, y, z = get_pose_values()

    goal_msg.pose.position.x = x
    goal_msg.pose.position.y = y
    goal_msg.pose.position.z = z
    goal_msg.header.stamp = rospy.Time.now()
    goal_msg.header.frame_id = 'world'

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        goal_pub.publish(goal_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass