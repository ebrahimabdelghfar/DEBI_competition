#!/usr/bin/env python3
from Import import *

class NavigateToGoal():
    def __init__(self):
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server()
        self.move_base.stop_tracking_goal()
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    def goto(self, goal):
        '''
        argument: 
            goal: 6x1 list [x, y, z, qx, qy, qz]
                translation are in meters
                rotation are in degrees
        '''
        robot_goal = MoveBaseGoal()
        robot_goal.target_pose.header.frame_id = "map"
        robot_goal.target_pose.header.stamp = rospy.Time.now()
        #give the translational cordinate
        robot_goal.target_pose.pose.position.x = goal[0]
        robot_goal.target_pose.pose.position.y = goal[1]
        robot_goal.target_pose.pose.position.z = goal[2]
        #transform euler angle to quaternion
        q = transformations.quaternion_from_euler(goal[3], goal[4], goal[5])
        robot_goal.target_pose.pose.orientation.x = q[0]
        robot_goal.target_pose.pose.orientation.y = q[1]
        robot_goal.target_pose.pose.orientation.z = q[2]
        robot_goal.target_pose.pose.orientation.w = q[3]
        self.move_base.send_goal(robot_goal)
        self.move_base.wait_for_result()
    def Stop(self):
        self.move_base.cancel_all_goals()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
        

