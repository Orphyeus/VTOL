import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL, SetMode, CommandBool, CommandBoolRequest

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

rospy.init_node("vtol_autonomous_flight")
state_sub = rospy.Subscriber("mavros/state", State, state_cb)
local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

def takeoff(altitude):
    rospy.wait_for_service('/mavros/cmd/takeoff')
    try:
        takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        takeoff_service(altitude=altitude, latitude=0, longitude=0, min_pitch=0, yaw=0)
        rospy.loginfo("Takeoff başlatıldı")
    except rospy.ServiceException as e:
        rospy.logerr("Takeoff servis çağrısı başarısız: %s", e)

set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

def fly_to_point_in_offboard_mode(x, y, z):
    # OFFBOARD moda geçiş
    rospy.wait_for_service('/mavros/set_mode')
    try:
        set_mode_resp = set_mode_client(custom_mode="OFFBOARD")
        if set_mode_resp.mode_sent:
            rospy.loginfo("Offboard moduna geçildi")
    except rospy.ServiceException as e:
        rospy.logerr("Mod değiştirme servisi çağrısı başarısız: %s", e)

    # Belirli bir noktaya uçuş
    target_pose = PoseStamped()
    target_pose.pose.position.x = x
    target_pose.pose.position.y = y
    target_pose.pose.position.z = z

    for i in range(100):  # 5 saniye süresince
        local_pos_pub.publish(target_pose)
        rospy.sleep(0.05)

def land():
    rospy.wait_for_service('/mavros/cmd/land')
    try:
        land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        land_service(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
        rospy.loginfo("İniş başlatıldı")
    except rospy.ServiceException as e:
        rospy.logerr("Land servis çağrısı başarısız: %s", e)

if __name__ == '__main__':
    try:
        # Arming
        arm_drone()
        rospy.sleep(5)

        # Takeoff (MC modunda)
        takeoff(altitude=10)  # 10 metre yüksekliğe takeoff
        rospy.sleep(10)

        # OFFBOARD modunda belirli bir noktaya uçuş (FW modunda)
        fly_to_point_in_offboard_mode(x=20, y=30, z=40)
        rospy.sleep(10)

        # Land (MC modunda)
        land()
    except rospy.ROSInterruptException:
        pass
