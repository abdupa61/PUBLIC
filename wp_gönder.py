#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ROS python API
import rospy, mavros,math,cv2
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, Twist
from sensor_msgs.msg import NavSatFix,Image
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String, Float64
from decimal import *
from cv_bridge import CvBridge, CvBridgeError


# Current Position
latitude = 0
longitude = 0
altitude=0
local_x = 0
local_y = 0
# Position before Move function execute
previous_latitude = 47.397742
previous_longitude = 8.5455936
previous_altitude = 0.0
global sp_pub,sp_glob_pub

mavros.set_namespace() #for global callback
def globalPositionCallback(globalPositionCallback):
    global latitude
    global longitude
    global altitude
    altitude = int(math.ceil(globalPositionCallback.altitude)- 47.5)
    latitude = globalPositionCallback.latitude
    longitude = globalPositionCallback.longitude


def localPositionCallback(localPositionCallback):
    global altitude, local_x, local_y, local_w
    local_x = localPositionCallback.pose.position.x
    local_y = localPositionCallback.pose.position.y

def glob_pos_pub (wp_lat,wp_long,wp_alt):
    cnt=Controller()
    global altitude,latitude,longitude,sp_glob_pub
    cnt.sp_glob.latitude = wp_lat
    cnt.sp_glob.longitude = wp_long
    cnt.sp_glob.altitude = wp_alt #7 metre 0 metre =488
    cnt.sp_glob.velocity.x = 1
    cnt.sp_glob.velocity.y = 1
    cnt.sp_glob.acceleration_or_force.x = 1
    cnt.sp_glob.acceleration_or_force.y = 1
    rate = rospy.Rate(20.0)

    while not rospy.is_shutdown():
        rate.sleep()
        sp_glob_pub.publish(cnt.sp_glob)
        latitude = float("{0:.6f}".format(latitude))
        longitude = float("{0:.6f}".format(longitude))
        #print("poselanıyor")
        #print(longitude,latitude,altitude,cnt.sp_glob.altitude)
        if wp_lat==latitude and wp_long==longitude and wp_alt==altitude:
            print("Konuma gidildi.")
            break

# cv2 bridge
bridge = CvBridge()
cv_image = ""
counter = 0
last_red_latitude = 0
last_red_longitude = 0
red_latitude = 0
red_longitude = 0
pre_radius = 0

def image_callback(ros_image):
    global bridge, cv_image, counter, latitude, longitude, red_latitude, red_longitude,pre_radius
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
    while True:
        lower_red = (0, 80, 80)
        upper_red = (0, 255, 255)

        scale_percent = 50
        frame = cv_image
        #resize frame
        width = int(frame.shape[1] * scale_percent / 100)
        height = int(frame.shape[0] * scale_percent / 100)
        dim = (width, height)
        frame = cv2.resize(frame, dim)
        blurred_org_frame = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv_frame = cv2.cvtColor(blurred_org_frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_frame, lower_red, upper_red)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]

        if len(contours) > 0:

            # find contour which has max area
            c = max(contours, key=cv2.contourArea)
            # find its coordinates and radius
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if radius > 10:
                # draw circle around blue color
                #cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                #cv2.circle(mask, (int(x), int(y)), int(radius), (0, 255, 255), 2)

                # draw contours around blue color
                #cv2.drawContours(frame, contours, -1, (0, 255, 255), 2)
                if pre_radius < radius:
                    #rospy.sleep(0.005)
                    pre_radius=radius
                    print("radius=", radius)
                    red_latitude=latitude
                    red_longitude=longitude

        break

# Flight modes class
# Flight modes are activated using ROS services
class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self):
        global longitude, latitude
        rospy.wait_for_service('mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy(
                '/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService(altitude=0, latitude=latitude,
                           longitude=longitude, min_pitch=0, yaw=0)
        except rospy.ServiceException as e:
            print("Service takeoff call failed: %s" % e)

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            print("Waiting for arming...")
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print("Service arming call failed: %s" % e)

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            print("Waiting for disarming...")
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print("Service disarming call failed: %s" % e)

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            print("It's stabilazed mode!")
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException, e:
            print("service set_mode call failed: %s. Stabilized Mode could not be set." % e)

    def setOffboardMode(self):
        global sp_pub
        cnt = Controller()
        rate = rospy.Rate(20.0)
        k = 0
        while k < 10:
            sp_pub.publish(cnt.sp)
            rate.sleep()
            k = k + 1
        sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        rospy.wait_for_service('/mavros/set_mode')

        try:
            flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
            response = flightModeService(custom_mode='OFFBOARD')
            return response.mode_sent
        except rospy.ServiceException, e:
            print
            "service set_mode call failed: %s. Offboard Mode could not be set." % e
            return False

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException, e:
            print("service set_mode call failed: %s. Altitude Mode could not be set." % e)

    def setLoiterMode(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
            isModeChanged = flightModeService(custom_mode='AUTO.LOITER')  # return true or false
        except rospy.ServiceException as e:
            print(
                "service set_mode call failed: %s. AUTO.LOITER Mode could not be set. Check that GPS is enabled %s" % e)

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException, e:
            print("service set_mode call failed: %s. Position Mode could not be set." % e)

    def setLandMode(self):
        global pos_mode
        pos_mode = False
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            landService = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
            isLanding = landService(altitude=0)
        except rospy.ServiceException, e:
            print
            "service land call failed: %s. The vehicle cannot land " % e


class Controller:

    # initialization method
    def __init__(self):
        global local_x, local_y, local_w
        # Drone state
        self.state = State()  # using that msg for send few setpoint messages, then activate OFFBOARD mode, to take effect
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        self.sp_glob = GlobalPositionTarget()
        self.sp_glob.velocity.x = 1
        self.sp_glob.velocity.y = 1
        self.sp_glob.type_mask = int('010111111000', 2)
        self.sp_glob.coordinate_frame = 6  # FRAME_GLOBAL_INT
        self.sp_glob.latitude = 0
        self.sp_glob.longitude = 0
        self.sp_glob.altitude = 0
        self.level = PoseStamped()

        self.local_position_publisher = rospy.Publisher(mavros.get_topic('setpoint_position', 'local'), PoseStamped, queue_size=10)
    # set the flag to use position setpoints and yaw angle
    # self.wp.position.z = self.ALT_SP
        self.local_pos = Point(0, 0, 0)
        self.sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        self.rate = rospy.Rate(20.0)

## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    def updateSp(self):
        self.level.pose.position.x = local_x
        self.level.pose.position.y = local_y


def waypointmove():
    global red_longitude, red_latitude
    modes = fcuModes()
    glob_pos_pub( 47.397720,8.545593,495)
    glob_pos_pub( 47.397700,8.545593,495)
    glob_pos_pub( 47.397680,8.545593,495)
    glob_pos_pub( 47.397660,8.545593,495)
    glob_pos_pub( 47.397640,8.545593,495)
    glob_pos_pub( 47.397620,8.545593,495)
    glob_pos_pub( 47.397600,8.545593,495)
    glob_pos_pub( 47.397580,8.545593,495)
    glob_pos_pub( 47.397560,8.545593,495)

    glob_pos_pub( 47.397540,8.545593,495)
    glob_pos_pub( 47.397520,8.545593,495)
    glob_pos_pub(red_latitude,red_longitude,495)
    glob_pos_pub( red_latitude,red_longitude,491) #3 metreye alçal
    modes.setLoiterMode()
    print("3 metreye alçaldı")
    rospy.sleep(5)
    modes.setOffboardMode()
    glob_pos_pub( red_latitude,red_longitude,495) #7 metreye yüksel
    print("7 metreye yükseldi")
    glob_pos_pub( 47.397742,8.545593,495) #başlangıca geri dön
    modes.setLoiterMode()
    print("loitera geçildi")
    rospy.sleep(2)
    modes.setLandMode()
# Main function
def main():
    global sp_pub,sp_glob_pub
    # initiate node
    rospy.init_node('setpoint_node', anonymous=True)

    # flight mode object
    modes = fcuModes()

    # controller object
    cnt = Controller()

    # ROS loop rate
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)
    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPositionCallback)
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, localPositionCallback)
    rospy.Subscriber("iris/camera/rgb/image_raw", Image, image_callback)


    # Setpoint publisher
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    sp_glob_pub=rospy.Publisher('mavros/setpoint_raw/global', GlobalPositionTarget, queue_size=1)
    # Make sure the drone is armed

    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()

    modes.setTakeoff()
    rospy.sleep(10)
    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    print("KALKIS YUKSEKLIGI", altitude,cnt.sp_glob.altitude)
    print("MAIN: SET OFFBOARD")
    # activate OFFBOARD mode
    modes.setOffboardMode()
    waypointmove()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

