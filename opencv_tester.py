#!/usr/bin/env python
# ROS python API
import rospy, cv2
import numpy as np

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, Twist
from sensor_msgs.msg import NavSatFix, Image
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from decimal import *
from math import radians, cos, sin, asin, sqrt
from cv_bridge import CvBridge, CvBridgeError

# Message publisher for haversine
velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
msg1 = Twist()

# Current Position
latitude = 0
longitude = 0
altitude = 0.0
local_x=0
local_y=0
# Position before Move function execute
previous_latitude = 0
previous_longitude = 0
previous_altitude = 0.0

def globalPositionCallback(globalPositionCallback):
    global latitude
    global longitude
    latitude = globalPositionCallback.latitude
    longitude = globalPositionCallback.longitude

def localPositionCallback(localPositionCallback):
    global altitude, local_x,local_y
    altitude=localPositionCallback.pose.position.z
    local_x=localPositionCallback.pose.position.x
    local_y = localPositionCallback.pose.position.y


# cv2 bridge
bridge = CvBridge()
cv_image = ""
counter = 0
last_red_latitude = 0
last_red_longitude = 0
red_latitude = 0
red_longitude = 0
pre_radius = 0


# callback for image
def image_callback(ros_image):
    global bridge, cv_image, counter, latitude, longitude, red_latitude, red_longitude,pre_radius
    # kamera genişliği
    dispW = 960
    # kamera yüksekliği
    dispH = 720
    flip = 4
    # flipe 4 atanır kamera düz görüntü verir
    camSet = 'nvarguscamerasrc !  video/x-raw(memory:NVMM), width=3264, height=2464, ' \
             'format=NV12, framerate=21/1 ! nvvidconv flip-method=' + str(flip) + \
             ' ! video/x-raw, width=' + str(dispW) + ', height=' + str(dispH) + \
             ', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'

    cap = cv2.VideoCapture(camSet)

    kernel = np.ones((5, 5), np.float32) / 25
    if not cap.isOpened():
        print("error cam cannot open")
        exit()
    while True:
        lower_red = (0, 80, 80)
        upper_red = (0, 255, 255)

        frame = cap.read()
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
    cap.release()

class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self):
	global longitude, latitude
        rospy.wait_for_service('mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy(
                '/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService(altitude=10, latitude=latitude,
                           longitude=longitude, min_pitch=0, yaw=0)
        except rospy.ServiceException as e: 
            print ("Service takeoff call failed: %s" %e)

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            print("Waiting for arming...")
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s" % e)

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            print("Waiting for disarming...")
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print("Service disarming call failed: %s" % e)

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            print("It's stabilazed mode!")
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException as e:
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
        except rospy.ServiceException as e:
            print
            "service set_mode call failed: %s. Offboard Mode could not be set." % e
            return False

    def setLoiterMode(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
            isModeChanged = flightModeService(custom_mode='AUTO.LOITER')  # return true or false
        except rospy.ServiceException as e:
            print(
                "service set_mode call failed: %s. AUTO.LOITER Mode could not be set. Check that GPS is enabled %s" % e)

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Altitude Mode could not be set." % e)

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Position Mode could not be set." % e)

    def setLandMode(self):
        global pos_mode
        print("SET LAND")
        pos_mode = False
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            landService = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
            isLanding = landService(altitude=0)
        except rospy.ServiceException as e:
            print
            "service land call failed: %s. The vehicle cannot land " % e


class Controller:

    # initialization method
    def __init__(self):
        global local_x, local_y
        # Drone state
        self.state = State()  # using that msg for send few setpoint messages, then activate OFFBOARD mode, to take effect
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        # set the flag to use position setpoints and yaw angle
        #self.wp.position.z = self.ALT_SP
        self.local_pos = Point(0,0,0)
        self.sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        self.rate = rospy.Rate(20.0)

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    def updateSp(self):
        self.sp.position.x = local_x
        self.sp.position.y = local_y

def alcal():
    print("ALCALIYOR")
    rate = rospy.Rate(20.0)
    cnt = Controller()
    ALT_SP = 2
    cnt.sp.position.z = ALT_SP
    while not rospy.is_shutdown():
        cnt.updateSp()
        cnt.sp_pub.publish(cnt.sp)
        rate.sleep()
        if (cnt.sp.position.z +0.15)> altitude:
            print("ALCALDIGI DEGER=",altitude)
            break
def yuksel():
    print("YUKSELIYOR")
    rate = rospy.Rate(20.0)
    cnt = Controller()
    ALT_SP = 7
    cnt.sp.position.z = ALT_SP
    while not rospy.is_shutdown():
        cnt.updateSp()
        cnt.sp_pub.publish(cnt.sp)
        rate.sleep()
        if (cnt.sp.position.z -0.15)< altitude:
            print("YUKSELDIGI DEGER=",altitude)
            break


def haversine():
    global previous_longitude, previous_latitude, longitude, latitude

    lon1, lat1, lon2, lat2 = previous_longitude, previous_latitude, longitude, latitude

    # lat, long in radians
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # difference lat, long
    dlon = abs(lon2 - lon1)
    dlat = abs(lat2 - lat1)

    # haversine formula
    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    c = 2 * asin(sqrt(a))
    r = 6371  # Radius of earth in kilometers

    return int (c * r * 1000)


def distance_calculate(lon1, lat1, lon2, lat2):
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])
    dlon = abs(lon2 - lon1)
    dlat = abs(lat2 - lat1)
    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    c = 2 * asin(sqrt(a))
    r = 6371  # Radius of earth in kilometers

    return Decimal(c * r * 1000)


def moveX(distance, speed):
    global previous_longitude, previous_latitude, longitude, latitude
    msg1.linear.x = speed
    rate = rospy.Rate(20.0)
    modes = fcuModes()

    while not rospy.is_shutdown():
	mesafe = int(distance)	
        haversine_distance = haversine()+  distance - mesafe
        if haversine_distance >= distance:
	    print ("ifteki mesafe:", haversine_distance)
            break

        velocity_pub.publish(msg1)
        rate.sleep()

    print(
        "Latitude: {:.7f} ,Longitude: {:.7f}\nDistance Covered: {:.7f}".format(latitude, longitude, haversine_distance))

    msg1.linear.x = 0.
    msg1.linear.y = 0.
    msg1.linear.z = 0.
    for i in range(100):
        velocity_pub.publish(msg1)
    print("Speed X: {} ,Speed Y: {} ,Speed Z: {} ".format(msg1.linear.x, msg1.linear.y, msg1.linear.z))

    previous_latitude = latitude
    previous_longitude = longitude


def moveY(distance, speed):
    global previous_longitude, previous_latitude, longitude, latitude
    msg1.linear.y = speed
    rate = rospy.Rate(20.0)
    modes = fcuModes()

    while not rospy.is_shutdown():
	mesafe = int(distance)	
        haversine_distance = haversine()+  distance - mesafe
        if haversine_distance >= distance:
            break
        velocity_pub.publish(msg1)
        rate.sleep()

    print(
        "Latitude: {:.7f} ,Longitude: {:.7f}\nDistance Covered: {:.7f}".format(latitude, longitude, haversine_distance))

    msg1.linear.x = 0.
    msg1.linear.y = 0.
    msg1.linear.z = 0.
    for i in range(100):
        velocity_pub.publish(msg1)

    print("Speed X: {} ,Speed Y: {} ,Speed Z: {} ".format(msg1.linear.x, msg1.linear.y, msg1.linear.z))

    # after reaching desired position, set home_latitude, home_longitude to current position value
    previous_latitude = latitude
    previous_longitude = longitude


global x , y

global sp_pub

# Main function
def main():
    global sp_pub
    # initiate node
    rospy.init_node('setpoint_node', anonymous=True)
    # flight mode object
    modes = fcuModes()
    # controller object
    cnt = Controller()
    # ROS loop rate
    rate = rospy.Rate(20.0)
    # Subscribes to drone
    rospy.Subscriber('mavros/state', State, cnt.stateCb)
    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPositionCallback)
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, localPositionCallback)
    rospy.Subscriber("iris/camera/rgb/image_raw", Image, image_callback)

    # Setpoint publisher
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
