import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest


current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def change_mode(mode):
    rospy.wait_for_service('/mavros/set_mode')
    try:
        set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        resp = set_mode_service(custom_mode=mode)
        return resp.mode_sent
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def control_vtol():
    rospy.init_node("vtol_control_node")
    rospy.Subscriber("mavros/state", State, state_cb)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 10  # Kalkış yüksekliği

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    rate = rospy.Rate(20)  # 20 Hz
    while not rospy.is_shutdown():
        if current_state.mode != "OFFBOARD":
            change_mode("OFFBOARD")  # Kalkış için MC modu
            rospy.sleep(5)

        if current_state.mode == "OFFBOARD" and current_state.armed:
            change_mode("AUTO.LOITER")  # Uçuş için FW modu
            rospy.sleep(5)

        # İniş Koşulu: Belirli bir yüksekliğe ulaşıldığında MC moduna geç
        if current_state.armed and pose.pose.position.z < 1.0:  # Örnek yükseklik koşulu
            change_mode("OFFBOARD")  # İniş için MC modu
            rospy.sleep(5)

        local_pos_pub.publish(pose)
        rate.sleep()


if __name__ == "__main__":
    try:
        control_vtol()
    except rospy.ROSInterruptException:
        pass
