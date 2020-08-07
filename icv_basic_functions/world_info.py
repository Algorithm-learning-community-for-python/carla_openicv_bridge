#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Class to handle the carla map
"""

#import rospy

from carla_msgs.msg import CarlaWorldInfo
from pseudo_actor import PseudoActor
from icvPublisher import Publisher
from threading import Thread, Lock, Event
import time

class WorldInfo(PseudoActor):

    """
    Publish the map
    """

    def __init__(self, carla_world, communication):
        """
        Constructor

        :param carla_world: carla world object
        :type carla_world: carla.World
        :param communication: communication-handle
        :type communication: carla_icv_bridge.communication
        """

        super(WorldInfo, self).__init__(parent=None,
                                        communication=communication,
                                        prefix="world_info")

        self.carla_map = carla_world.get_map()

        self.map_published = False
        self.pub1=Publisher(self.get_topic_prefix() )
        self.update_loop_thread = Thread(target=self.loop)
        self.update_loop_thread.start() 

    def loop(self):
        while True:
            time.sleep(10)
            self.map_published = False
         





    def destroy(self):
        """
        Function (override) to destroy this object.

        Remove reference to carla.Map object.
        Finally forward call to super class.

        :return:
        """
        #rospy.logdebug("Destroying WorldInfo()")
        self.carla_map = None
        super(WorldInfo, self).destroy()

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.

        :return:
        """
        if not self.map_published:
            #print("send map")
            open_drive_msg = CarlaWorldInfo()
            open_drive_msg.map_name = self.carla_map.name
            open_drive_msg.opendrive = self.carla_map.to_opendrive()
            self.pub1.publish( open_drive_msg)
            self.map_published = True
