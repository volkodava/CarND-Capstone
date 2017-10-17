#!/usr/bin/env python

import rospy
import bisect
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray
import matplotlib.pyplot as plt
import numpy as np

import math
import tf

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_lights_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.lane = None
        self.pose = None
        self.lights = None

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg
        if self.lane:
            # if self.pose.pose.position.x > 2000:
            #     self.plot()
            lane = self.waypoints_following(self.lane, self.pose)
            self.final_waypoints_pub.publish(lane)

    def plot(self, mn, mx):
        wp = self.lane.waypoints[self.next_waypoint()]
        xy = np.array([[w.pose.pose.position.x, w.pose.pose.position.y] for w in self.lane.waypoints[wp-100:wp+100]])
        plt.plot(xy[:,0], xy[:,1], "r+")
        curr = self.pose.pose.position
        plt.plot(curr.x, curr.y, "bs")
        plt.plot(wp.pose.pose.position.x, wp.pose.pose.position.y, "gs")
        plt.show()


    def waypoints_cb(self, msg):
        self.lane = msg

    def traffic_lights_cb(self, tl):
        self.lights = tl.lights
        #rospy.loginfo('Waypoint updater received traffic lights %s', len(tl.lights))

    def copy_waypoint(self, wp):
        w = Waypoint()
        w.pose.pose.position.x = wp.pose.pose.position.x
        w.pose.pose.position.y = wp.pose.pose.position.y
        w.pose.pose.position.z = wp.pose.pose.position.z
        w.pose.pose.orientation.x = wp.pose.pose.orientation.x
        w.pose.pose.orientation.y = wp.pose.pose.orientation.y
        w.pose.pose.orientation.z = wp.pose.pose.orientation.z
        w.pose.pose.orientation.w = wp.pose.pose.orientation.w
        w.twist.twist.linear.x = wp.twist.twist.linear.x
        w.twist.twist.linear.y = wp.twist.twist.linear.y
        w.twist.twist.linear.z = wp.twist.twist.linear.z
        w.twist.twist.angular.x = wp.twist.twist.angular.x
        w.twist.twist.angular.y = wp.twist.twist.angular.y
        w.twist.twist.angular.z = wp.twist.twist.angular.z
        return w

    def next_waypoint(self):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        head = lambda a, b: math.atan2(a.y-b.y, a.x-b.x)
        dist = [dl(w.pose.pose.position, self.pose.pose.position) for w in self.lane.waypoints]
        min_wp = np.argmin(dist)
        o = self.pose.pose.orientation
        _, _, theta = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])
        heading = head(self.lane.waypoints[min_wp].pose.pose.position, self.pose.pose.position)
        if abs(heading-theta) > np.pi/4.:
            min_wp += 1
            if min_wp >= len(self.lane.waypoints):
                min_wp = 0
        return min_wp


    def waypoints_following(self, lane, pose):
        curr_x = pose.pose.position.x
        wp_index = self.next_waypoint()
        rospy.loginfo("%s wp out of %s", wp_index, len(self.lane.waypoints))
        wp = [self.copy_waypoint(w) for w in lane.waypoints[wp_index : wp_index+LOOKAHEAD_WPS]]
        # if self.lights:
        #     waypoints = [w.pose.pose.position.x for w in wp]
        #     red_lights = [bisect.bisect(waypoints, l.pose.pose.position.x) for l in self.lights if l.state == l.RED]
        #     green_lights = [bisect.bisect(waypoints, l.pose.pose.position.x) for l in self.lights if l.state == l.GREEN]
        #     #intersections = [bisect.bisect(waypoints, r) for r in red_lights]
        #     stop_light = next((l for l in red_lights if l > 0 and l < len(waypoints)), None)
        #     go_light = next((l for l in green_lights if l > 0 and l < len(waypoints)), None)
        #     if stop_light:
        #         rospy.loginfo('Stop light %s', stop_light)
        #         curr_v = wp[0].twist.twist.linear.x
        #         dec_v = curr_v/stop_light
        #         #for i in range(0, stop_light): wp[i].twist.twist.linear.x -= (i+1)*dec_v
        #         #for w in wp: w.twist.twist.linear.x = 0
        #         rospy.loginfo('wps - %s ', [w.twist.twist.linear.x for w in wp[:10]])
        #     # if go_light:
        #     #     rospy.loginfo('Go light %s', go_light)
        #     #     # for i in range(go_light-10, go_light): wp[i].twist.twist.linear.x = 10
        #     #     rospy.loginfo('wps - %s ', [w.twist.twist.linear.x for w in wp[:10]])
        # # yp = [w.pose.pose.position.y for w in wp]
        return Lane(None, wp)


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
