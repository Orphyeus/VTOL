#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from geometry_msgs.msg import Point, PoseStamped
from math import atan2, asin, cos, sin, pi


class FCU:
    def __init__(self):

        rospy.init_node('mc_node', anonymous=True)  # name of this node
        self.rate = rospy.Rate(20.0)
        
        # pub/sub
        self.pixhawk_sp_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.local_pos_pub  = rospy.Publisher('drone/local_pos', Point, queue_size = 1)
        rospy.Subscriber('mavros/state', State, self.state_cb)
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.local_pose_cb)
        rospy.Subscriber('mavros/extended_state', ExtendedState, self.extended_state_cb)

        # services 
        self.flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
        self.takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
        self.landingService = rospy.ServiceProxy('mavros/cmd/land', mavros_msgs.srv.CommandTOL)
        self.armService     = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)

        # variables 
        self.current_state = State()
        self.current_state.connected = 0
        self.pixhawk_pose  = PoseStamped()
        self.pixhawk_sp    = PoseStamped()
        self.local_pos_int = 0
        self.local_pos  = Point(0, 0, 0)
        self.local_pos0 = Point(0, 0, 0)
        self.local_sp   = Point(0, 0, 0)
        self.extended_state = ExtendedState()
        # connect to pixhawk 
        self.connect()

        pass

    def takeoff(self):
        self.arm()

        while ((self.extended_state.landed_state == 1) or (+10.0 > self.local_pos.z)):
            rospy.wait_for_service('mavros/cmd/takeoff')
            # self.takeoffService(min_pitch = 15.0, altitude = 30.0, yaw = 0.0)
            self.takeoffService(altitude = 30.0, yaw = 90.0)
            rospy.loginfo("Taking off, %d", self.extended_state.landed_state)
            self.rate.sleep()


    def land(self):

        # would probably not want to put inside a while loop 
        while (self.extended_state.landed_state != 1):
            rospy.wait_for_service('mavros/cmd/landing')
            self.landingService(min_pitch = 0.0, altitude = 30.0, yaw = 90.0)
            rospy.loginfo("Landing off")
            self.rate.sleep()

    def arm(self):
        while (self.current_state.armed == False):
            rospy.wait_for_service('mavros/cmd/arming')
            self.armService(True)
            self.rate.sleep()
        # rospy.loginfo("ARMED: %d", self.current_state.armed)


    def disarm(self):
        while (self.current_state.armed == True):
            rospy.wait_for_service('mavros/cmd/arming')
            self.armService(False)
            self.rate.sleep()
        rospy.loginfo("ARMED: %d", self.current_state.armed)


    def connect(self):
        timeout = 100.0
        t_s = rospy.get_time()
        while ((rospy.is_shutdown != 0) and (self.current_state.connected == False)):
            rospy.loginfo("Connecting...")
            if (rospy.get_time() - t_s > timeout):
                break
            self.rate.sleep()

        if (self.current_state.connected == True):
            rospy.loginfo("Pixhawk connected")
        else:
            rospy.loginfo("Failed to connect")


    def setOffboardMode(self):
        self.arm()
        rospy.wait_for_service('mavros/set_mode')
        self.flightModeService(custom_mode = 'OFFBOARD')

    def offboardModeSetup(self):
        self.pixhawk_sp.pose.position.x = 0
        self.pixhawk_sp.pose.position.y = 0
        self.pixhawk_sp.pose.position.z = 30

        # set offboard mode
        for cnt in range (0,100):
            self.pixhawk_sp_pub.publish(self.pixhawk_sp)
            self.rate.sleep()
        
        for cnt in range (0,10):
            self.setOffboardMode()
            self.rate.sleep()

    def state_cb(self, msg):
        self.current_state = msg

    def extended_state_cb(self, msg):
        self.extended_state = msg

    def local_pose_cb(self, msg):

        self.pixhawk_pose = msg
        if (self.local_pos_int == 0):
            self.local_pos0.x = self.pixhawk_pose.pose.position.x
            self.local_pos0.y = self.pixhawk_pose.pose.position.y
            self.local_pos0.z = self.pixhawk_pose.pose.position.z
            self.local_pos_int = 1

        self.local_pos.x = self.pixhawk_pose.pose.position.x - self.local_pos0.x
        self.local_pos.y = self.pixhawk_pose.pose.position.y - self.local_pos0.y
        self.local_pos.z = self.pixhawk_pose.pose.position.z - self.local_pos0.z

    def setWaypoint(self, local_sp, yaw_d):

        self.pixhawk_sp.pose.position.x = local_sp.x + self.local_pos0.x
        self.pixhawk_sp.pose.position.y = local_sp.y + self.local_pos0.y
        self.pixhawk_sp.pose.position.z = local_sp.z + self.local_pos0.z

        if (yaw_d > 180.0):
            yaw_d = (-yaw_d - 90.0)*pi/180.0
            self.pixhawk_sp.pose.orientation.w = -sin(yaw_d/2)
            self.pixhawk_sp.pose.orientation.x = 0
            self.pixhawk_sp.pose.orientation.y = 0
            self.pixhawk_sp.pose.orientation.z = cos(yaw_d/2)
        else:
            yaw_d = (-yaw_d - 90.0)*pi/180.0
            self.pixhawk_sp.pose.orientation.w = sin(yaw_d/2)
            self.pixhawk_sp.pose.orientation.x = 0
            self.pixhawk_sp.pose.orientation.y = 0
            self.pixhawk_sp.pose.orientation.z = -cos(yaw_d/2)

        self.pixhawk_sp_pub.publish(self.pixhawk_sp)
        self.local_pos_pub.publish(self.local_pos)


def main():

    
    fcu = FCU()    
    
    fcu.arm()
    fcu.offboardModeSetup()
    fcu.takeoff()

    timeout = 30.0
    t_s = rospy.get_time()
    local_sp = Point(100.0, 100.0, 50.0)
    yaw_d = 90.0

    while (timeout > rospy.get_time() - t_s):
    
        fcu.setOffboardMode()
        fcu.setWaypoint(local_sp, yaw_d)    
        

        fcu.rate.sleep()
    
    fcu.land()

    fcu.disarm()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
