#!/usr/bin/env python
import math
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, Point
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
#from math import inf
import numpy as np
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.image_count = 467
        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        #  can be used used to determine the vehicle's location.
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        # provides the complete list of waypoints for the course.
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)

        # provides an image stream from the car's camera. These images are used to determine the color of upcoming traffic lights.
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.closest_waypoint = 0

        self.IGNORE_DISTANCE_LIGHT = 90.0
        self.old_stop_line_pos_wp = []
        self.last_car_position = 0
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
        Returns:
            int: index of the closest waypoint in self.waypoints
        """
        
        pos = pose.position
        l_id = 0
        r_id = len(self.waypoints) - 1
        m_id = len(self.waypoints)-1

        while l_id < r_id:
            ldist = self.pos_distance(self.waypoints[l_id].pose.pose.position, pos)
            rdist = self.pos_distance(self.waypoints[r_id].pose.pose.position, pos)
            xmid = (l_id + r_id) // 2
            mdist = self.pos_distance(self.waypoints[xmid].pose.pose.position, pos)

            closest_dist = ldist
            m_id = l_id
            if mdist < closest_dist:
                closest_dist = mdist
                m_id = xmid
            if rdist < closest_dist:
                closest_dist = rdist
                m_id = r_id

            # If l_id is right before xmid and xmid is right before r_id,
            # then xmid is the closest waypoint
            if l_id == xmid -1 and xmid == r_id -1:
                break

            # c: car
            # l: left point
            # r: right point
            # m: xmid
            # *: closest waypoint
            if rdist < mdist:
                if ldist < rdist:
                    # l--c----r--m
                    r_id = xmid - 1
                else:
                    # l----c--r--m
                    l_id = xmid + 1

            elif mdist < closest_dist:
                # l--c--m--*--r
                l_id = xmid-1
            elif mdist > closest_dist :
                # l--c--*--m--r
                r_id = xmid+1

            elif mdist == closest_dist:
                # ?-cm-?
                if ldist < rdist:
                    # l--cm---r
                    r_id = xmid + (r_id - xmid) // 2
                elif rdist < ldist:
                    # l---cm--r
                    l_id = xmid - (xmid - l_id) // 2

        return m_id

    def pos_distance(self, a, b):
        """ Distance between two positions
        """
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)

    def distance_2d(self, a, b):
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)

    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """


        # From udacity.
        fx = 2574
        fy = 2744
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        trans = None

        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        # Use tranform and rotation to calculate 2D position of light in image
        if (trans != None):
            # Convert rotation vector so we can use it.
            yaw = tf.transformations.euler_from_quaternion(rot)[2]

            # Rotation followed by translation
            px = point_in_world.x
            py = point_in_world.y
            pz = point_in_world.z
            xt = trans[0]
            yt = trans[1]
            zt = trans[2]

            Rnt = (
                px * math.cos(yaw) - py * math.sin(yaw) + xt,
                px * math.sin(yaw) + py * math.cos(yaw) + yt,
                pz + zt)

            u = int(fx * -Rnt[1] / Rnt[0] + image_width / 2 - 30)
            v = int(fy * -(Rnt[2] - 1.0) / Rnt[0] + image_height + 50)

            light_width = 1.0
            light_height = 1.95

            distance = self.distance_2d(self.pose.pose.position, point_in_world)

            # Size of traffic light within 2D picture
            light_width_estimate = 2 * fx * math.atan(light_width / (2 * distance))
            light_height_estimate = 2 * fx * math.atan(light_height / (2 * distance))
            # Get points for traffic light's bounding box
            bbox_topleft = (int(u - light_width_estimate / 2), int(v - light_height_estimate / 2))
            bbox_bottomright = (int(u + light_width_estimate / 2), int(v + light_height_estimate / 2))
        else:
            # No translation matrix so we cannot find the light.
            bbox_topleft = (0, 0)
            bbox_bottomright = (0, 0)

        return (bbox_topleft, bbox_bottomright)


    def resize_image(self, img, width, height):
        
        aspect_ratio_width = 0.5
        aspect_ratio_height = height/width
        img_height, img_width = img.shape[:2]
        crop_height = int(img_width / aspect_ratio_width)
        extra_height = (img_height - crop_height) / 2
        crop_width = int(img_height / aspect_ratio_height)
        extra_width = (img_width - crop_width) / 2
        # Crop image to keep aspect ratio
        if extra_height > 0:
            crop_img = img[int(extra_height):int(img_height-math.ceil(extra_height)), 0:int(img_width)]
        elif extra_width > 0:
            crop_img = img[0:int(img_height), int(extra_width):int(img_width-math.ceil(extra_width))]
        else:
            crop_img = img

        return cv2.resize(crop_img, (width, height), 0, 0, interpolation=cv2.INTER_AREA)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        pt = Point()
        pt.x = light.pose.pose.position.x
        pt.y = light.pose.pose.position.y
        pt.z = 0
        
        # Convert given traffic light coordinates into position within 2D image
        tleft, bright = self.project_to_image_plane(light.pose.pose.position)
        cropped_image = cv_image[tleft[1]:bright[1], tleft[0]:bright[0]]

        if (cropped_image.shape[0] > 0 and cropped_image.shape[1] > 0):
            cropped_image = self.resize_image(cropped_image, 30, 60)

        #Get classification
        clazz = self.light_classifier.get_classification(cropped_image)
        rospy.loginfo(clazz)

        # TODO: Make sure the classifier works correctly and re-enable this code:
        # return clazz
        return light.state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color
        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        light = None

        if self.waypoints is not None:
            # List of positions that correspond to the line to stop in front of for a given intersection
            stop_line_positions = self.config['stop_line_positions']
            if(self.pose):
                car_position = self.get_closest_waypoint(self.pose.pose)

            #TODO find the closest visible traffic light (if one exists)
            light = self.get_closest_light(self.pose.pose)

            if light:
                light_wp = self.get_closest_waypoint(light.pose.pose)
                state = self.get_light_state(light)

                # Debugging traffic light:
                #
                # rospy.loginfo("light_xyz: ({}, {}, {}), wp_xyz({}): ({}, {}, {})".format(
                #     light.pose.pose.position.x,
                #     light.pose.pose.position.y,
                #     light.pose.pose.position.z,
                #     light_wp,
                #     self.waypoints[light_wp].pose.pose.position.x,
                #     self.waypoints[light_wp].pose.pose.position.y,
                #     self.waypoints[light_wp].pose.pose.position.z
                # ))
                return light_wp, state
            self.waypoints = None
        return -1, TrafficLight.UNKNOWN

    def get_closest_light(self, pose):
        """ Get the position of the closest traffic light.

        Args:
            pose (Pose): Position of car.
        Returns:
            TrafficLight: light object.
        """
        # TODO: Decide if we should have a horizon (a max distance at which the car will try and capture the light).
        horizon = 100

        min_dist = float("inf")
        light = None
        for l in self.lights:
            dist = self.pos_distance(pose.position, l.pose.pose.position)
            if dist < min_dist and dist < horizon:
                min_dist = dist
                light = l
        return light

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
