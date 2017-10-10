#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import tf

import math
import PyKDL
from copy import deepcopy

import time

# from scipy.interpolate import interp1d

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

LOOKAHEAD_WPS = 12


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        self.waypoints = None
        self.waypoints_header = None

        # To avoid publishing same points multiple times.
        self.published_wp = None

        # For benchmarking closest wp code 
        # self.sum_wp_time = 0.0
        # self.count_wp_time = 0

        # Current waypoint id.
        self.cur_wp = None

        # Current car's pose.
        self.cur_pose = None

        self.tl_config = {
            "min_dist": 50, # In meters
            "a": -10,
        }

        rospy.spin()

    def pose_cb(self, msg):
        """
        msg:

        geometry_msgs/Pose pose
          geometry_msgs/Point position
            float64 x
            float64 y
            float64 z
          geometry_msgs/Quaternion orientation
            float64 x
            float64 y
            float64 z
            float64 w
        """
        self.pose = msg.pose
        pos = self.pose.position
        quat = PyKDL.Rotation.Quaternion(self.pose.orientation.x,
                                         self.pose.orientation.y,
                                         self.pose.orientation.z,
                                         self.pose.orientation.w)
        orient = quat.GetRPY()
        yaw = orient[2]

        if self.waypoints is not None:
            # For circular id i.e. to keep from breaking when
            # `(cur_wp_id + LOOKAHEAD_WPS) > len(self.waypoints)`
            n = len(self.waypoints)

            # rospy.loginfo("curyaw: {}".format(yaw))

            # start = time.time()
            self.cur_wp = self.get_closest_waypoint(self.pose)
            # end = time.time()
            # self.sum_wp_time += (end - start)
            # self.count_wp_time += 1
            # avg_wp_time = self.sum_wp_time / self.count_wp_time
            # rospy.loginfo("m_id time: {}".format(avg_wp_time))

            lane = Lane()
            for i, wp in enumerate(self.waypoints[
                self.cur_wp%n:(self.cur_wp+LOOKAHEAD_WPS)%n]):

                if i == 0:
                    idx = self.cur_wp + i

                    # Calculates yaw rate
                    next_wp = self.waypoints[(idx+1)%n]
                    next_x = next_wp.pose.pose.position.x
                    next_y = next_wp.pose.pose.position.y

                    quat = PyKDL.Rotation.Quaternion(next_wp.pose.pose.orientation.x,
                                                     next_wp.pose.pose.orientation.y,
                                                     next_wp.pose.pose.orientation.z,
                                                     next_wp.pose.pose.orientation.w)
                    next_orient = quat.GetRPY()
                    next_yaw = next_orient[2]

                    yaw_dist = next_yaw - yaw

                    self.set_waypoint_velocity(wp, 10, yaw_dist)

                lane.waypoints.append(wp)

            # rospy.loginfo("(p) next_wp angular: {}".format(lane.waypoints[0].twist.twist.angular))
            self.final_waypoints_pub.publish(lane)

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints
        self.waypoints_header = waypoints.header

    def traffic_cb(self, msg):
        tl_wp = msg.data
        # `tl_wp >= self.cur_wp` ensures traffic light is in front of the car.
        if tl_wp > -1 and tl_wp >= self.cur_wp:
            self.realize_tl_action(tl_wp)
        rospy.loginfo("tl_wp: {}".format(tl_wp))

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoint, velocity, yaw):
        waypoint.twist.twist.linear.x = velocity * math.cos(yaw)
        waypoint.twist.twist.linear.z = velocity * math.sin(yaw)

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        for i in range(wp1, wp2+1):
            dist += self.pos_distance(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

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

    def realize_tl_action(self, tl_wp):
        # TODO
        # if pos_distance(self.pose, self.waypoints[tl_wp].pose.pose) < \
        #    self.tl_config["min_dist"]:
        #     # Find x number of waypoints before tl_wp, and then gradually
        #     # lower the speed

        #     # remaining distance
        #     rem_dist = self.tl_config["min_dist"]
        #     while rem_dist > 0:
        #         dist = 
        #         rem_dist - dist
        pass

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
