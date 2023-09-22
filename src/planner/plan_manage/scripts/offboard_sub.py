#!/usr/bin/env python3
# ROS python API
import rospy

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
from geometry_msgs.msg import Twist, TwistStamped
from geometry_msgs.msg import Vector3, Vector3Stamped

# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
import numpy as np
from nav_msgs.msg import Odometry

# Flight modes class
# Flight modes are activated using ROS services
class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self):
        rospy.wait_for_service('mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService(altitude = 3)
        except rospy.ServiceException as e:
            print("Service takeoff call failed: ,%s")%e

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s")%e

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print("Service disarming call failed: %s")%e

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Stabilized Mode could not be set.")%e

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Offboard Mode could not be set.")%e

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Altitude Mode could not be set.")%e

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Position Mode could not be set.")%e

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
               print("service set_mode call failed: %s. Autoland Mode could not be set.")%e

class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PoseStamped()
        # set the flag to use position setpoints and yaw angle
        self.sv = TwistStamped()
        self.sa = Vector3Stamped()

        # self.sodomcmd = Odometry()

        # Step size for position update
        self.STEP_SIZE = 2.0
        # Fence. We will assume a square fence for now
        self.FENCE_LIMIT = 5.0

        # A Message for the current local position of the drone
        
        # initial values for setpoints
        self.sp.pose.position.x = 0.0
        self.sp.pose.position.y = 0.0
        self.ALT_SP = 1.0
        self.sp.pose.position.z = self.ALT_SP
        self.local_pos = Point(0.0, 0.0, self.ALT_SP)

        # self.sv.twist.linear

        # speed of the drone is set using MPC_XY_CRUISE parameter in MAVLink
        # using QGroundControl. By default it is 5 m/s.

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        self.qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        self.qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        self.qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        self.qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        # print(self.qx, self.qy, self.qz, self.qw)
        
        return [self.qx, self.qy, self.qz, self.qw]

    # Callbacks

    ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

    # def newPoseCB(self, msg):
    #     self.sp.pose.position.x = msg.pose.position.x
    #     self.sp.pose.position.y = msg.pose.position.y
    #     self.sp.pose.position.z = msg.pose.position.z
    
    #     self.sp.pose.orientation.x = msg.pose.orientation.x
    #     self.sp.pose.orientation.y = msg.pose.orientation.y
    #     self.sp.pose.orientation.z = msg.pose.orientation.z
    #     self.sp.pose.orientation.w = msg.pose.orientation.w
    
    def newPoseOdomCmdCB(self, msg):
        self.sp.pose.position.x = msg.pose.pose.position.x
        self.sp.pose.position.y = msg.pose.pose.position.y
        self.sp.pose.position.z = msg.pose.pose.position.z
    
        self.sv.twist.linear.x = msg.twist.twist.linear.x
        self.sv.twist.linear.y = msg.twist.twist.linear.y
        self.sv.twist.linear.z = msg.twist.twist.linear.z       

        self.sa.vector.x = msg.twist.twist.angular.x
        self.sa.vector.y = msg.twist.twist.angular.y
        self.sa.vector.z = msg.twist.twist.angular.z

        self.sp.pose.orientation.x = self.get_quaternion_from_euler(0,0,msg.pose.pose.orientation.x)[0]
        self.sp.pose.orientation.y = self.get_quaternion_from_euler(0,0,msg.pose.pose.orientation.x)[1]
        self.sp.pose.orientation.z = self.get_quaternion_from_euler(0,0,msg.pose.pose.orientation.x)[2]
        self.sp.pose.orientation.w = self.get_quaternion_from_euler(0,0,msg.pose.pose.orientation.x)[3]
        self.sv.twist.angular.z = msg.pose.pose.orientation.y
        # self.sp.pose.orientation
        # print("Hi")
        # print(" ")

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    ## Update setpoint message
    def updateSp(self):
        self.sp.pose.position.x = self.local_pos.x
        self.sp.pose.position.y = self.local_pos.y
        self.sp.pose.position.z = self.local_pos.z

# Main function
def main():
    rospy.init_node('setpoint_node', anonymous=True)
    modes = fcuModes()  #flight modes
    cnt = Controller()  # controller object
    rate = rospy.Rate(50)
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    # rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)

    # rospy.Subscriber('new_pose', PoseStamped, cnt.newPoseCB) #new waypoint
    rospy.Subscriber("/position_odom_cmd", Odometry, cnt.newPoseOdomCmdCB)
    sp_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    sv_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    sa_pub = rospy.Publisher('/mavros/setpoint_accel/accel', Vector3Stamped, queue_size=10)
    # Setpoint publisher    
    movement_cmd = AttitudeTarget()
    thrust_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)

    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()
        print("ARMING")

    cnt.sp.pose.position.x = 0
    cnt.sp.pose.position.y = 0
    cnt.sp.pose.position.z = 1.0

    cnt.sp.pose.orientation.x = 0
    cnt.sp.pose.orientation.y = 0
    cnt.sp.pose.orientation.z = 0
    cnt.sp.pose.orientation.w = -1

    sp_pub.publish(cnt.sp)
    # sv_pub.publish(cnt.sv)
    # sa_pub.publish(cnt.sa)
    rate.sleep()

    modes.setOffboardMode()
    print("---------")
    print("OFFBOARD")
    print("---------")

    # ROS main loop
    while not rospy.is_shutdown():

        sp_pub.publish(cnt.sp)    
        sv_pub.publish(cnt.sv)
        sa_pub.publish(cnt.sa)
        rate.sleep()
        # print("while loop")
        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass