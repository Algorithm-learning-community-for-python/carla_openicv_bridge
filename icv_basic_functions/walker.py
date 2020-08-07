#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla pedestrians
"""
#import rospy
from derived_object_msgs.msg import Object
import threading
from threading import Thread, Lock, Event
from traffic_participant import TrafficParticipant
from carla_msgs.msg import CarlaWalkerControl
from carla import WalkerControl
from icvSubscriber import Subscriber

class Walker(TrafficParticipant):

    """
    Actor implementation details for pedestrians
    """

    def __init__(self, carla_actor, parent, communication, prefix=None):
        """
        Constructor

        :param carla_actor: carla walker actor object
        :type carla_actor: carla.Walker
        :param parent: the parent of this
        :type parent: carla_icv_bridge.Parent
        :param communication: communication-handle
        :type communication: carla_icv_bridge.communication
        :param prefix: the topic prefix to be used for this actor
        :type prefix: string
        """
        if not prefix:
            prefix = "walker/{:03}".format(carla_actor.id)

        super(Walker, self).__init__(carla_actor=carla_actor,
                                     parent=parent,
                                     communication=communication,
                                     prefix=prefix)

        self.control_subscriber = CarlaWalkerControl()
        self.sub1=Subscriber(self.get_topic_prefix() + "/walker_control_cmd")
        self.Sec_loop=0.02
        self.update_command_thread = Thread(target=self._update_commands_thread)
        self.update_command_thread.start()


    def update(self, frame, timestamp):
        super(Walker, self).update(frame, timestamp)


    def control_command_updated(self):
        """
        Receive a CarlaWalkerControl msg and send to CARLA
        This function gets called whenever a icv message is received via
        '/carla/<role name>/walker_control_cmd' topic.
        The received icv message is converted into carla.WalkerControl command and
        sent to CARLA.
        :param icv_walker_control: current walker control input received via icv
        :type self.info.output: carla_icv_bridge.msg.CarlaWalkerControl
        :return:
        """
        self.sub1.subscribe( self.control_subscriber)
        walker_control = WalkerControl()
        walker_control.direction.x = self.control_subscriber.direction.x
        walker_control.direction.y = -self.control_subscriber.direction.y
        walker_control.direction.z = self.control_subscriber.direction.z
        walker_control.speed = self.control_subscriber.speed
        walker_control.jump = self.control_subscriber.jump
        self.carla_actor.apply_control(walker_control)

    def _update_commands_thread (self):

        if self.sub1.getstate():
            self.sub1.reset()
            self.control_command_updated()
            


    def get_classification(self):
        """
        Function (override) to get classification
        :return:
        """
        return Object.CLASSIFICATION_PEDESTRIAN
