import zmq
import sys
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
import math
import io
import genpy
import struct
import re
import os


class Subscriber():
    def __init__(self,topic): 
        newurl=re.sub("/","_",topic) 
        filea=os.path.realpath(__file__)
        addr=os.path.dirname(filea)
        addr=os.path.dirname(addr)
        #print(addr)
        #url = "ipc://"+addr+"/buff/b"+newurl  
        url = "ipc:///tmp/buff"+newurl 
        
        if len(url)>90:
            url=url[:90]
            print("topic name too long")
        self.create_soc(url)

    def create_soc(self, url) : 
        print("subscribe topic:"+url)
        self.ctx=Context.instance()
        self.sub = self.ctx.socket(zmq.SUB)
        try :
            self.sub.connect(url)
        except :
            print("zmq recv socket failed")
        self.sub.setsockopt(zmq.SUBSCRIBE, b"")
        self.enable() 
    def reset(self):
        self.datacoming=False
    def enable(self):
        self.datacoming=True        
    def getstate(self):
        return self.datacoming    

    def subscribe(self, data):
        try:
            msg = self.sub.recv()      # ERROR: WAITS FOREVER
        except zmq.ZMQError as e:
            print(e)

        self.enable()
        #print('received: ', msg)
        data.deserialize(msg)

class Subscriber_Remote(Subscriber):
    def __init__(self,topic): 
        url = "tcp://"+topic  
        if len(url)>90:
            url=url[:90] 
        print("subscribe ip:"+url)
        self.create_soc(url)  


