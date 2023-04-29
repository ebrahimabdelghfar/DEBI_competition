from Import import *

class RobotControl:
    '''
    This class contain module that facilitate the controll of the robot using moveit 
    '''
    def __init__(self,node_name="robot_control",group_name="joint_group",planner_id="RRTConnectkConfigDefault",planning_time=5.0,num_planning_attempts=10,allow_replanning=True):
        '''
        constructor arguments:
            node_name: name of the node
            group_name: name of the group of joints to be controlled by moveit
        '''
        # initialize the node
        rospy.init_node(node_name,anonymous=True)
        # initialize moveit_commander and rospy node
        roscpp_initialize(sys.argv)
        
        # Instantiate a RobotCommander object.
        # Provides information such as the robot’s kinematic model and the robot’s current joint states
        self.robot = RobotCommander()

        print(self.robot.get_group_names())
        # Instantiate a PlanningSceneInterface object.
        # This provides a remote interface for getting, setting, and updating the robot’s internal understanding of the surrounding world:
        self.scene = PlanningSceneInterface()

        # define the group of joints to be controlled by moveit
        self.move_group = MoveGroupCommander(group_name)
        self.move_group.set_planner_id(planner_id)
        self.move_group.set_planning_time(planning_time)
        self.move_group.set_num_planning_attempts(num_planning_attempts)
        self.move_group.allow_replanning(allow_replanning)
        pass
    def gripper_control(self, joint_value,velocity=0.1,acceleration=0.1):
        '''
        --------------------
        This function is used to move the robot to the desired joint state
        --------------------
        arguments:
            joint_goal: list of joint angles in degree
            velocity: velocity of the robot
            acceleration: acceleration of the robot
        '''
        # set the velocity of the robot
        self.move_group.set_max_velocity_scaling_factor(velocity)
        # set the acceleration of the robot
        self.move_group.set_max_acceleration_scaling_factor(acceleration)
        # set the goal joint state
        self.move_group.go(joint_value,wait=True)
        # stop any residual movement
        self.move_group.stop()
        pass
    def go_by_joint_angle(self, joint_goal_list,velocity=0.1,acceleration=0.1,angle_is_degree=True):
        '''
        --------------------
        This function is used to move the robot to the desired joint state
        --------------------
        arguments:
            joint_goal: list of joint angles in degree
            velocity: velocity of the robot
            acceleration: acceleration of the robot
            angle_is_degree : if the input angle is in degree
        '''
        if (angle_is_degree):
            joint_goal=[joint_goal_list[0]*math.pi/180,joint_goal_list[1]*math.pi/180,joint_goal_list[2]*math.pi/180,joint_goal_list[3]*math.pi/180]
        else:
            joint_goal=joint_goal_list
        # set the velocity of the robot
        self.move_group.set_max_velocity_scaling_factor(velocity)
        # set the acceleration of the robot
        self.move_group.set_max_acceleration_scaling_factor(acceleration)
        # set the goal joint state
        self.move_group.go(joint_goal,wait=True)
        # stop any residual movement
        self.move_group.stop()
        pass
    def go_to_pose_goal_cartesian(self, pose_goal,velocity=0.1,acceleration=0.1):
        '''
        --------------------
        This function is used to move the robot to the desired pose by cartesian path
        --------------------
        arguments:
            pose_goal: geometry_msgs.msg.Pose
            velocity: velocity of the robot
            acceleration: acceleration of the robot
        '''
        # set the velocity of the robot
        self.move_group.set_max_velocity_scaling_factor(velocity)
        # set the acceleration of the robot
        self.move_group.set_max_acceleration_scaling_factor(acceleration)
        # set the goal pose
        self.move_group.set_pose_target(pose_goal)
        # plan the motion
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        # clear the targets
        self.move_group.clear_pose_targets()
        pass
    def go_to_pose_goal_cartesian_waypoints(self, waypoints,velocity=0.1,acceleration=0.1,list_type=False):
        '''
        --------------------
        This function is used to move the robot to the desired pose by cartesian path
        --------------------
        arguments:
            if list_type is true 
                waypoints: nx6 list of waypoints 
                    n: number of waypoints
                    6: x,y,z,roll,pitch,yaw
            if list_type is false 
                waypoints is given by type geometry_msgs.msg.Pose

            velocity: velocity of the robot
        
            acceleration: acceleration of the robot

            list_type indicatie if the given waypoints is geometry_msgs.msg.Pose or list
        '''
        geo_pose=geometry_msgs.msg.Pose() #create a geometry_msgs.msg.Pose() object
        list_of_poses = []
        for ways in waypoints:
            #set the position of the pose
            geo_pose.position.x = ways[0]
            geo_pose.position.y = ways[1]
            geo_pose.position.z = ways[2]
            #set the orientation of the pose
            quantrion = tf.transformations.quaternion_from_euler(ways[3],ways[4],ways[5])
            geo_pose.orientation.x = quantrion[0]
            geo_pose.orientation.y = quantrion[1]
            geo_pose.orientation.z = quantrion[2]
            geo_pose.orientation.w = quantrion[3]
            #append the pose to the list
            list_of_poses.append(copy.deepcopy(geo_pose))

        # set the goal pose
        (plan, fraction) = self.move_group.compute_cartesian_path(
                                    list_of_poses,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold
        # plan the motion

        # generate a new plan with the new velocity and acceleration by retiming the trajectory
        new_plan=self.move_group.retime_trajectory(self.robot.get_current_state(),plan,velocity_scaling_factor=velocity,acceleration_scaling_factor=acceleration)
        
        # execute the plan
        self.move_group.execute(new_plan,wait=True)

        pass
    def get_joint_state(self):
        '''
        fuctionality:
            This function is used to get the robot's joints state
        --------------------
        arguments:
            no arguments
        --------------------
        This function is used to get the current joint state of the robot
        --------------------
        return:
            joint_state: list of joint angles in radians
        --------------------
        '''
        # get the current joint state
        joint_state = self.move_group.get_current_joint_values()
        return joint_state
    def get_pose(self):
        '''
        fuctionality:
            This function is used to get the robot's or the end effector
        --------------------
        arguments:
            no arguments
        --------------------
        This function is used to get the current pose of the robot
        --------------------
        return:
            pose: geometry_msgs.msg.Pose
        --------------------
        '''
        # get the current pose
        pose = self.move_group.get_current_pose().pose
        return pose
    def get_joints_velocity(self):
        '''
        fucntionality:
            This function is used to get the current joint velocity of the robot
        --------------------
        arguments:
            no arguments
        --------------------
        This function is used to get the current joint velocity of the robot
        --------------------
        return:
            joint_velocity: list of joint velocities in radians per second
        --------------------
        '''
        # get the current joint velocity
        joint_velocity = self.move_group.get_current_joint_velocity()
        return joint_velocity
    def get_end_effector_velocity(self):
        '''
        functionality:
            This function is used to get the current end effector velocity of the robot
        --------------------
        arguments:
            no arguments
        --------------------
        return:
            end_effector_velocity: geometry_msgs.msg.Twist
        --------------------
        '''
        # get the current end effector velocity
        end_effector_velocity = self.move_group.get_current_velocity()
        return end_effector_velocity

class frames_transformations:
    '''
    this class is used to put and tarnsform frames in the tf tree
    '''
    def __init__(self):
        '''
        --------------------
        the constructor of the class
        --------------------
        functioninality:
            This function is used to instantiate the tf2_ros objects
        --------------------
        '''
        # This object is used to store the frames
        self.tfBuffer = tf2_ros.Buffer()

        # instantiate a tf2_ros.TransformListener object
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # instantiate a tf2_ros.StaticTransformBroadcaster object
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        # instantiate a tf2_ros.TransformBroadcaster object
        self.broadcaster = tf2_ros.TransformBroadcaster()

        pass

    def transform(self, parent_id, child_frame_id):
        '''
        functioninality:
            This function is used get the transform between two frames and return the pose of the child frame
        --------------------
        arguments:
            parent_id: string name of the parent frame
            child_frame_id: string name of the child frame
        --------------------
        return:
            pose: geometry_msgs.msg.Pose()
        '''
        # transform the frame

        transform_msg = geometry_msgs.msg.TransformStamped()
        pose=geometry_msgs.msg.Pose()

        transform_msg = self.tfBuffer.lookup_transform(parent_id,child_frame_id,rospy.Time.now())
        #transfer from TransformStamped() to PoseStamped()
        pose.position.x=transform_msg.transform.translation.x
        pose.position.y=transform_msg.transform.translation.y
        pose.position.z=transform_msg.transform.translation.z
        pose.orientation.x=transform_msg.transform.rotation.x
        pose.orientation.y=transform_msg.transform.rotation.y
        pose.orientation.z=transform_msg.transform.rotation.z
        pose.orientation.w=transform_msg.transform.rotation.w
        
        return pose

    def put_frame_static_frame(self,parent_frame_name="base_link",child_frame_name="tool0",frame_coordinate=[0,0,0,0,0,0]):
        '''
        --------------------
        This function is used to put the frame in the tf tree
        --------------------
        arguments:
            parent_frame_name: name of the parent frame
            child_frame_name: name of the child frame
            frame_coordinate: list of coordinates of the frame 1x6
                [x,y,z,rx,ry,rz]
                translation( in meter ) and rotation ( in radians )
        functionality:
            This function is used to put the frame in the tf tree
            it have delay of 0.05 seconds for the frame to be published
        --------------------
        '''
        frames_msg=geometry_msgs.msg.TransformStamped()
        frames_msg.header.frame_id=parent_frame_name
        frames_msg.child_frame_id=child_frame_name
        frames_msg.transform.translation.x=frame_coordinate[0]
        frames_msg.transform.translation.y=frame_coordinate[1]
        frames_msg.transform.translation.z=frame_coordinate[2]
        quatrion=tf.transformations.quaternion_from_euler(frame_coordinate[3],frame_coordinate[4],frame_coordinate[5])
        frames_msg.transform.rotation.x=quatrion[0]
        frames_msg.transform.rotation.y=quatrion[1]
        frames_msg.transform.rotation.z=quatrion[2]
        frames_msg.transform.rotation.w=quatrion[3]
        # put the frame in the tf tree
        self.static_broadcaster.sendTransform(frames_msg)
        rospy.sleep(0.2)


arm_group=RobotControl(group_name="arm",planner_id="PRM",planning_time=10.0)
gripper_group=RobotControl(group_name="gripper")
TransformationCalculator=frames_transformations()

def OpenGripper(speed=0.1,acceleration=0.1):
    gripper_group.gripper_control([0.03,0.03],speed,acceleration) #open the gripper

def CloseGripper(speed=0.1,acceleration=0.1):
    gripper_group.gripper_control([-0.001,-0.001],speed,acceleration) #open the gripper

def DownToGo(speed=0.1,acceleration=0.1):
    # TransformationCalculator.put_frame_static_frame(parent_frame_name="base_footprint",child_frame_name="ball_pos",frame_coordinate=[0.12,0.000,0.025,0.0,1.57,0.0])
    # pose=TransformationCalculator.transform(parent_id="base_footprint",child_frame_id="ball_pos")
    arm_group.go_by_joint_angle([0,42,5,42],speed,acceleration,True)
    # arm_group.go_to_pose_goal_cartesian(pose,speed,acceleration)

def UpToGo(speed=0.1,acceleration=0.1):
    TransformationCalculator.put_frame_static_frame(parent_frame_name="base_footprint",child_frame_name="ball_pos",frame_coordinate=[0.0,0.000,0.3,0.0,0.0,0.0])
    pose=TransformationCalculator.transform(parent_id="base_footprint",child_frame_id="ball_pos")
    arm_group.go_to_pose_goal_cartesian(pose,speed,acceleration)

def SideToGo(speed=0.1,acceleration=0.1):
    TransformationCalculator.put_frame_static_frame(parent_frame_name="base_footprint",child_frame_name="ball_pos",frame_coordinate=[-0.08,0.21,0.045,0.0,1.57,1.57])
    pose=TransformationCalculator.transform(parent_id="base_footprint",child_frame_id="ball_pos")
    arm_group.go_to_pose_goal_cartesian(pose,speed,acceleration)

def arm_pack_ball():
    OpenGripper(speed=1,acceleration=1)
    DownToGo(speed=1,acceleration=1)
    CloseGripper(speed=0.1,acceleration=0.1)
    UpToGo(speed=0.1,acceleration=0.1)

    
def PutShoot():
    arm_group.go_by_joint_angle([math.radians(0),math.radians(73),math.radians(-47),math.radians(-46)],0.2,0.2,angle_is_degree=False)
    OpenGripper(speed=1,acceleration=1)




def arm_unpack_ball():
    #validatio that the robot hold the ball
    if(abs(gripper_group.get_joint_state()[0])>= 0.0015):
        PutShoot()
        UpToGo(speed=1,acceleration=1)
    else:
        msg=Twist()
        msg.linear.x=0.75
        RobotControl.publish(msg)
        rospy.sleep(0.1)
        msg=Twist()
        msg.linear.x=0.0
        RobotControl.publish(msg)
        
def UpAndShoot():
    OpenGripper(speed=0.1,acceleration=0.1)
    joint_pos=arm_group.get_joint_state()
    arm_group.go_by_joint_angle([joint_pos[0],joint_pos[1]+math.radians(-20),joint_pos[2],joint_pos[3]],0.1,0.1,angle_is_degree=False)
    joint_pos=arm_group.get_joint_state()
    arm_group.go_by_joint_angle([joint_pos[0]+math.radians(30),joint_pos[1],joint_pos[2],joint_pos[3]],0.1,0.1,angle_is_degree=False)
    joint_pos=arm_group.get_joint_state()
    arm_group.go_by_joint_angle([joint_pos[0],joint_pos[1]+math.radians(20),joint_pos[2],joint_pos[3]],0.1,0.1,angle_is_degree=False)
    joint_pos=arm_group.get_joint_state()
    arm_group.go_by_joint_angle([joint_pos[0]+math.radians(-30),joint_pos[1],joint_pos[2],joint_pos[3]],1,1,angle_is_degree=False)

