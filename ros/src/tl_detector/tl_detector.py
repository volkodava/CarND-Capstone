#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, Point
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import numpy as np
import yaml
import time
from light_detector import LightDetector
from PIL import Image as PILImage
import math

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.stop_wps = None
        self.camera_image = None
        self.lights_wps = None

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        #sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub4 = rospy.Subscriber('/image_color', Image, self.image_cb)

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
        self.loop_rate = 1

        model = 'light_classification/subhash_frozen_inference_graph.pb'
        label_path = 'light_classification/subhash_label_map.pbtxt'
        # model = 'light_classification/mmsarode_frozen_inference_graph.pb'
        # label_path = 'light_classification/mmsarode_label_map.pbtxt'

        num_classes = 4
        self.light_detector = LightDetector(model, label_path, num_classes)

        self.run_loop()

    def run_loop(self):
        rate = rospy.Rate(self.loop_rate)
        while not rospy.is_shutdown():
            if self.camera_image is not None \
                    and self.pose is not None \
                    and self.waypoints is not None:
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
                    if state == TrafficLight.RED or state == TrafficLight.YELLOW:
                        light_wp = light_wp
                    else:
                        light_wp = -1
                    self.last_wp = light_wp
                    rospy.loginfo("LD %s", light_wp)
                    self.upcoming_red_light_pub.publish(Int32(light_wp))
                else:
                    self.upcoming_red_light_pub.publish(Int32(self.last_wp))
                self.state_count += 1
            rate.sleep()

    def pose_cb(self, msg):
        self.pose = msg.pose

    def closest_waypoint(self, wps, position, orient=None, around_wp = None):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        if around_wp:
            cand_wps = wps[around_wp: around_wp+20]
        else:
            cand_wps = wps
        dist = [dl(w.pose.pose.position, position) for w in cand_wps]
        min_wp = np.argmin(dist)
        if around_wp:
            min_wp += around_wp
        if orient:
            _, _, theta = tf.transformations.euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
            head = lambda a, b: math.atan2(a.y - b.y, a.x - b.x)
            heading = head(wps[min_wp].pose.pose.position, position)
            if abs(heading - theta) > np.pi/4:
                min_wp = (min_wp + 1) % len(wps)
        return min_wp

    def waypoints_cb(self, msg):
        if not self.waypoints:
            self.waypoints = msg.waypoints
            self.stop_wps = []
            for [x, y] in self.config['stop_line_positions']:
                stop_wp = self.closest_waypoint(self.waypoints, Point(x,y,0))
                self.stop_wps.append(stop_wp)


    def traffic_cb(self, msg):
        lights = msg.lights
        closest_red_light_wp = -1
        red_light_wps = [self.stop_wps[i] for i, l in enumerate(lights) if l.state == TrafficLight.RED or l.state == TrafficLight.YELLOW]
        if red_light_wps:
            red_light_waypoints = [self.waypoints[rwp] for rwp in red_light_wps]
            closest_red_light_index = self.closest_waypoint(red_light_waypoints, self.pose.position, self.pose.orientation)
            closest_red_light_wp = red_light_wps[closest_red_light_index]
        self.upcoming_red_light_pub.publish(Int32(closest_red_light_wp))


    def capture_image_cb(self, msg):
        if self.light_detector:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            image, classes, dt = self.light_detector.infer(cv_image)
            if classes.count(2.0) > 1 or classes.count(3.0) > 1:
                waypoints = [self.waypoints[i] for i in self.stop_wps]
                stop_wp = self.closest_waypoint(waypoints, self.pose.position)
                light_wp = self.stop_wps[stop_wp]
                rospy.loginfo("LD: Stop %s  %s ", light_wp, classes)
                self.upcoming_red_light_pub.publish(Int32(light_wp))

            rospy.loginfo("LD: Classes %s %s", classes, dt)
            #cv2.imwrite("image_dump2/sim_"+str(time.time())+".jpg", image)

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.camera_image = msg

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        stop_waypoints = [self.waypoints[wp] for wp in self.stop_wps]
        closest_wp_index = self.closest_waypoint(stop_waypoints, pose.position, pose.orientation)
        return self.stop_wps[closest_wp_index]

    def get_light_state(self):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if self.camera_image is None:
            rospy.loginfo("Camera image NOT found")
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
        image, classes, dt = self.light_detector.infer(cv_image)


        # cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        #
        # #Get classification
        # return self.light_classifier.get_classification(cv_image)
        state = TrafficLight.UNKNOWN
        if classes:
            max_class = max(classes, key=classes.count)
            class_dict = {
                1: TrafficLight.GREEN, 2: TrafficLight.RED,
                3: TrafficLight.YELLOW, 4: TrafficLight.UNKNOWN
            }
            class_dict_names = {
                1: "GREEN", 2: "RED",
                3: "YELLOW", 4: "UNKNOWN"
            }
            state = class_dict[max_class]
            rospy.loginfo("Class %s", class_dict_names[max_class])
        else:
            rospy.loginfo("Classes NOT found")

        return state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        # stop_line_positions = self.config['stop_line_positions']
        closest_stop_wp = self.get_closest_waypoint(self.pose)
        state = self.get_light_state()
        return closest_stop_wp, state

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
