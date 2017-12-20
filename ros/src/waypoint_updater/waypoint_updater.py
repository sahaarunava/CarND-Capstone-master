#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
from copy import deepcopy
from path_planner import PathPlanner 

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

LOOKAHEAD_WPS = 30 # Number of waypoints we will publish. You can change this number

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        self.current_pose = None
        self.base_waypoints = None

        self.planner = PathPlanner(LOOKAHEAD_WPS)

        # rospy.spin()

    def publish(self, waypoints):
        if waypoints == []:
            return 

        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.get_rostime()
        lane.waypoints = waypoints
        self.final_waypoints_pub.publish(lane)

    def run(self):
        rospy.loginfo('### Run')
        rate = rospy.Rate(2) # 10hz
        while not rospy.is_shutdown():
            rospy.loginfo('### looping ... ')
            waypoints = self.planner.generate_waypoints()
            self.publish(waypoints)
            rate.sleep()

    def pose_cb(self, msg):
        # msg - geometry_msgs/PoseStamped
        # rospy.loginfo('### pose Received')
        self.current_pose = msg
        self.planner.update_vehicle_location(self.current_pose)

    def waypoints_cb(self, lane):
        # waypoints - styx_msgs/Lane, only received once
        rospy.loginfo('### BASE Waypoint Received')
        self.base_waypoints = lane.waypoints
        self.planner.set_base_waypoints(lane.waypoints)

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
        WaypointUpdater().run()
    except rospy.ROSInterruptException:
rospy.logerr('Could not start waypoint updater node.')