#!/usr/bin/env python

#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Handle communication of icv topics
"""
#import rospy

#from icvgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from protocol.std_msgs.msg import Time
from icvtime import icvTime
from icvPublisher import Publisher



class Communication(object):

    """
    Handle communication of icv topics
    """

    def __init__(self):
        """
        Constructor
        """
        self.tf_to_publish = []
        self.msgs_to_publish = []

        self.timestamp = Time()

        #icvTF=TFMessage()

        # needed?
        self.pubtf=Publisher("tf")
        self.pubmarker=Publisher("/carla/marker") 
        self.pubclock=Publisher("clock")    
        self.publist=Publisher("/carla/actor_list")    
  

        

    def update_clock(self, carla_timestamp):
        """
        perform the update of the clock

        :param carla_timestamp: the current carla time
        :type carla_timestamp: carla.Timestamp
        :return:
        """
        time=icvTime()
        self.timestamp = time.from_sec(carla_timestamp.elapsed_seconds)

        self.pubclock.publish(self.timestamp)

    def get_current_time(self):
        """
        get the current icv time

        :return: the current icv time
        :rtype icv.Time
        """
        return self.timestamp
