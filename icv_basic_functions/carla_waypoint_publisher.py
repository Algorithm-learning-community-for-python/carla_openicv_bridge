#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Generates a plan of waypoints to follow

It uses the current pose of the ego vehicle as starting point. If the
vehicle is respawned, the route is newly calculated.

The goal is either read from the ROS topic `/carla/<ROLE NAME>/move_base_simple/goal`, if available
(e.g. published by RVIZ via '2D Nav Goal') or a fixed spawnpoint is used.

The calculated route is published on '/carla/<ROLE NAME>/waypoints'
"""
import math
import threading
import sys
sys.path.append("..")
sys.path.append("../pyicv")
sys.path.append("../protocol")
sys.path.append("../tools")
sys.path.append("../thirdparty")
sys.path.append("/home/jiangkun/carla/PythonAPI/carla")

import transformations
from threading import Thread, Lock, Event

from transformations import euler_from_quaternion
from protocol.nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from carla_msgs.msg import CarlaWorldInfo
from zzz_driver_msgs.msg import RigidBodyStateStamped

import carla
from carla import Location, Rotation, Transform, Vector3D

from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
from icvtime import icvTime
from icvPublisher import Publisher
from icvSubscriber import Subscriber
import time

class CarlaToRosWaypointConverter(object):

    """
    This class generates a plan of waypoints to follow.

    The calculation is done whenever:
    - the hero vehicle appears
    - a new goal is set
    """
    WAYPOINT_DISTANCE = 2.0

    def __init__(self, carla_world, setstart, setgoal):
        self.world = carla_world

        #only for circles
        self.map = carla_world.get_map()
        self.ego_vehicle = None
        self.role_name = 'ego_vehicle'
        self.waypoint_publisher = Publisher(
            'carla_ego_vehicle_waypoints'.format(self.role_name))

        # set initial goal

        self.start = setstart
        self.goal = setgoal
        
        self.current_route = None
        self.goal_subscriber = Subscriber("/carla/{}/goal".format(self.role_name))
        self._pose_subscriber = Subscriber("ego_pose_localization")#, RigidBodyStateStamped, self.pose_callback
        self._pose =RigidBodyStateStamped()
        self.pose_=PoseStamped()

        self._update_lock = threading.Lock()
        self.update_loop_thread = Thread(target=self.loop)
        self.update_loop_thread.start()  
        # use callback to wait for ego vehicle
        self.world.on_tick(self.find_ego_vehicle_actor)

    def loop(self):
        while True:
            time.sleep(0.25)
            self.publish_waypoints()
            '''
            if self.goal_subscriber.getstate():
                self.goal_subscriber.reset()
                print "goal?"
                self.goal_subscriber.subscribe(self.pose_)
                self.on_goal(self.pose_)'''
            '''
            if self._pose_subscriber.getstate():
                self._pose_subscriber.reset()
                self._pose_subscriber.subscribe(self._pose)
                self.pose_callback(self._pose)'''

    def pose_callback(self, msg):
        # Note: Here we actually assume that pose is updating at highest frequency
        print ("We received pose in carla waypoint publisher")
        #self.publish_waypoints()
    
    def on_goal(self, goal):
        """
        Callback for /move_base_simple/goal

        Receiving a goal (e.g. from RVIZ '2D Nav Goal') triggers a new route calculation.

        :return:
        """
        carla_goal = carla.Transform()
        carla_goal.location.x = goal.pose.position.x
        carla_goal.location.y = -goal.pose.position.y
        carla_goal.location.z = goal.pose.position.z + 2  # 2m above ground
        quaternion = (
            goal.pose.orientation.x,
            goal.pose.orientation.y,
            goal.pose.orientation.z,
            goal.pose.orientation.w
        )
        _, _, yaw = euler_from_quaternion(quaternion)
        carla_goal.rotation.yaw = -math.degrees(yaw)

        self.goal = carla_goal
        print ("start to reroute")
        self.reroute()

    def reroute(self):
        """
        Triggers a rerouting
        """
        if self.ego_vehicle is None or self.goal is None:
            # no ego vehicle, remove route if published
            self.current_route = None
            self.publish_waypoints()
        else:
            self.current_route = self.calculate_route(self.start, self.goal)
        self.publish_waypoints()

    def find_ego_vehicle_actor(self, _):
        """
        Look for an carla actor with name 'ego_vehicle'
        """
        with self._update_lock:
            hero = None
            for actor in self.world.get_actors():
                if actor.attributes.get('role_name') == self.role_name:
                    hero = actor
                    break

            ego_vehicle_changed = False
            if hero is None and self.ego_vehicle is not None:
                ego_vehicle_changed = True

            if not ego_vehicle_changed and hero is not None and self.ego_vehicle is None:
                ego_vehicle_changed = True

            if not ego_vehicle_changed and hero is not None and \
                    self.ego_vehicle is not None and hero.id != self.ego_vehicle.id:
                ego_vehicle_changed = True

            if ego_vehicle_changed:
                #rospy.loginfo("Ego vehicle changed.")
                self.ego_vehicle = hero
                self.reroute()

    def calculate_route(self, start, goal):
        """
        Calculate a route from the current location to 'goal'
        """
        # rospy.loginfo("Calculating route to x={}, y={}, z={}".format(
        #     goal.location.x,
        #     goal.location.y,
        #     goal.location.z))

        dao = GlobalRoutePlannerDAO(self.world.get_map(), 1)
        grp = GlobalRoutePlanner(dao)
        grp.setup()
        route = grp.trace_route(carla.Location(start.location.x,
                                               start.location.y,
                                               start.location.z),
                                carla.Location(goal.location.x,
                                               goal.location.y,
                                               goal.location.z))

        return route

    def publish_waypoints(self):
        """
        Publish the ROS message containing the waypoints
        """
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = icvTime().time_now_o()
        if self.current_route is not None:
            for wp in self.current_route:
                pose = PoseStamped()
                pose.pose.position.x = wp[0].transform.location.x
                pose.pose.position.y = -wp[0].transform.location.y
                pose.pose.position.z = wp[0].transform.location.z

                quaternion = transformations.quaternion_from_euler(
                    0, 0, -math.radians(wp[0].transform.rotation.yaw))
                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]
                msg.poses.append(pose)

        self.waypoint_publisher.publish(msg)
        print("Published {} waypoints.".format(len(msg.poses)))


def main():
    """
    main function
    """

    first_goal = Transform()
    first_goal.location.x = 32
    first_goal.location.y = -20
    first_goal.location.z = 0
    first_goal.rotation.pitch = 0
    first_goal.rotation.yaw = 0 
    first_goal.rotation.roll = 0

    first_start = Transform()
    first_start.location.x = -10
    first_start.location.y = -96
    first_start.location.z = 0
    first_start.rotation.pitch = 0
    first_start.rotation.yaw = 180
    first_start.rotation.roll = 0

    sub1=Subscriber("/carla/world_info")    
    worldinfo=CarlaWorldInfo()
    sub1.enable()
    sub1.subscribe(worldinfo)

    host =  "127.0.0.1"
    port =  2000

    carla_client = carla.Client(host=host, port=port)
    carla_client.set_timeout(2)

    carla_world = carla_client.get_world()

    waypointConverter = CarlaToRosWaypointConverter(carla_world, first_start, first_goal)



if __name__ == "__main__":
    main()
