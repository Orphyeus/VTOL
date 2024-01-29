import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL, SetMode, CommandBool, CommandBoolRequest

# Drone'un şu anki durumunu tutan global değişken
current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def arm_drone():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        arm_cmd = CommandBoolRequest(value=True)
        arming_client(arm_cmd)
        rospy.loginfo("Drone silahlandırıldı")
    except rospy.ServiceException as e:
        rospy.logerr("Arming işlemi başarısız: %s", e)

def change_mode(mode):
    rospy.wait_for_service('/mavros/set_mode')
    try:
        set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        set_mode_client(custom_mode=mode)
        rospy.loginfo("{} moduna geçildi".format(mode))
    except rospy.ServiceException as e:
        rospy.logerr("Mod değiştirme işlemi başarısız: %s", e)

def takeoff(altitude):
    rospy.wait_for_service('/mavros/cmd/takeoff')
    try:
        takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        takeoff_service(altitude=altitude, latitude=0, longitude=0, min_pitch=0, yaw=0)
        rospy.loginfo("Takeoff başlatıldı")
    except rospy.ServiceException as e:
        rospy.logerr("Takeoff işlemi başarısız: %s", e)

def land():
    rospy.wait_for_service('/mavros/cmd/land')
    try:
        land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        land_service(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
        rospy.loginfo("İniş başlatıldı")
    except rospy.ServiceException as e:
        rospy.logerr("Land işlemi başarısız: %s", e)

if __name__ == '__main__':
    rospy.init_node("vtol_autonomous_flight")
    rospy.Subscriber("mavros/state", State, state_cb)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    # Arming
    arm_drone()
    rospy.sleep(5)

    # Takeoff (MC modunda)
    takeoff(altitude=10)
    rospy.sleep(10)

    # OFFBOARD moduna geçiş ve belirli bir noktaya uçuş (FW modunda)
    change_mode("OFFBOARD")
    target_pose = PoseStamped()
    target_pose.pose.position.x = 20
    target_pose.pose.position.y = 30
    target_pose.pose.position.z = 40
    for i in range(100):
        local_pos_pub.publish(target_pose)
        rospy.sleep(0.1)

    # Land (MC modunda)
    change_mode("AUTO.LAND")
    land()
    rospy.sleep(10)
