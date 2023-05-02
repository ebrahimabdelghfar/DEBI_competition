#!/usr/bin/env python3
from Import import *
# from arm_contol import *
BallDetectedFlag=False
flag_safe=True
position=0
yaw=0
#BallInfo
ErrorCenters=0.0
Radius=0.0
#End
def BallDetected(data:Bool):
    global BallDetectedFlag
    BallDetectedFlag=data.data

def odom_callback(data:Odometry):
    global position,yaw
    # Update current position and orientation of the robot
    position = data.pose.pose.position
    orientation = data.pose.pose.orientation
    orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
    _, _, yaw = euler_from_quaternion(orientation_list)

def BallInfo(data:Float32MultiArray):
    global ErrorCenters,Radius
    ErrorCenters=data.data[1]
    Radius=data.data[0]

def BallSerach():
    global ErrorCenters , BallDetectedFlag
    while(True):
        msg=Twist()
        msg.angular.z=0.2
        RobotControl.publish(msg)
        if(abs(ErrorCenters)<=2 and BallDetectedFlag ):
            msg.angular.z=0.0
            RobotControl.publish(msg)
            break

def approachBall(SetpointRadius=62):
    global ErrorCenters,Radius,BallDetectedFlag, position
    if position.x>1.35:
        BallSerach()
    while (True):
        msg=Twist()
        msg.linear.x=(abs(Radius-SetpointRadius)/SetpointRadius)*0.21
        msg.angular.z=-(ErrorCenters/320)*0.32
        RobotControl.publish(msg)
        if(abs(Radius-SetpointRadius)<=1):
            msg=Twist()
            msg.angular.z=0.0000
            msg.linear.x=0.1
            RobotControl.publish(msg)
            rospy.sleep(0.5*4.25)
            msg=Twist()
            msg.angular.z=0.0
            msg.linear.x=0.0
            RobotControl.publish(msg)
            break
def clearRot():
    msg=Twist()
    msg.angular.z=0.2
    RobotControl.publish(msg)
    rospy.sleep(0.8)
    msg=Twist()
    msg.angular.z=0.0
    msg.linear.x=0.0
    RobotControl.publish(msg)

rospy.Subscriber("/odom",Odometry,odom_callback)
rospy.Subscriber("/BallFlag",Bool,BallDetected)
rospy.Subscriber("/Ball_info",Float32MultiArray,BallInfo)
RobotControl=rospy.Publisher("/cmd_vel",Twist,queue_size=1,latch=True)





