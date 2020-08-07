#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla gnsss
"""

from protocol.sensor_msgs.msg import NavSatFix

from sensor import Sensor
from icvPublisher import Publisher


class Gnss(Sensor):

    """
    Actor implementation details for gnss sensor
    """

    def __init__(self, carla_actor, parent, communication, synchronous_mode):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_icv_bridge.Parent
        :param communication: communication-handle
        :type communication: carla_icv_bridge.communication
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(Gnss, self).__init__(carla_actor=carla_actor,
                                   parent=parent,
                                   communication=communication,
                                   synchronous_mode=synchronous_mode,
                                   prefix="gnss/" + carla_actor.attributes.get('role_name'))
        self.pub1=Publisher(self.get_topic_prefix()+ "/fix")

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_gnss_event):
        """
        Function to transform a received gnss event into a icv NavSatFix message

        :param carla_gnss_event: carla gnss event object
        :type carla_gnss_event: carla.GnssEvent
        """
        navsatfix_msg = NavSatFix()
        navsatfix_msg.header = self.get_msg_header(timestamp=carla_gnss_event.timestamp)
        navsatfix_msg.latitude = carla_gnss_event.latitude
        navsatfix_msg.longitude = carla_gnss_event.longitude
        navsatfix_msg.altitude = carla_gnss_event.altitude
        self.pub1.publish(navsatfix_msg)
