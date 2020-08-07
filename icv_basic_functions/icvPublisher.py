import zmq
from zmq import Context
from protocol.sensor_msgs.msg import *
from protocol.std_msgs.msg import *
from geometry_msgs.msg import *
from carla_msgs.msg import *
from derived_object_msgs.msg import *
from nav_msgs.msg import *
from radar_msgs.msg import *
from shape_msgs.msg import *
from tf2_msgs.msg import *
from visualization_msgs.msg import *
import re
import os
import math
import io
import genpy
import struct



class Publisher():

    def __init__(self,topic): 
        newurl=re.sub("/","_",topic) 
        #url = "ipc://buff/b"+newurl
        filea=os.path.realpath(__file__)
        addr=os.path.dirname(filea)
        addr=os.path.dirname(addr)
        #print(addr)
        #url = "ipc://"+addr+"/buff/b"+newurl  
        url = "ipc:///tmp/buff"+newurl 
        if len(url)>90:
            url=url[:90]
            print("topic name too long")
        print("publish topic:"+url)
        self.topic_r=None
        self.create_soc(url)

    def create_soc(self,url):
        self.ctx=Context.instance()
        self.pub = self.ctx.socket(zmq.PUB)
        try:

            self.pub.bind(url)
            self.topic_r=url
        except :
            print("zmq pub socket failed")
        #self.topics= bytes(topic, 'utf-8')
    def publish(self, data):

        output=io.BytesIO()
        data.serialize(output)
        #self.pub.send_multipart([self.topics, output.getvalue()])  # WRONG: bytes(msg)
        try:
            self.pub.send( output.getvalue())  # WRONG: bytes(msg)
        except zmq.ZMQError as e:
            print(e)
        output.truncate(0)

class Publisher_Remote(Publisher):

    def __init__(self,topic): 
        url = "tcp://"+topic  
        if len(url)>90:
            url=url[:90]
            print("topic name too long")
        print("publish ip:"+url)
        self.create_soc(url)


