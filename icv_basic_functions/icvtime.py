import sys
import time
from protocol.std_msgs.msg import Time
from protocol.std_msgs.msg import Duration

class icvTime:
      def __init__(self, *args, **kwds):
         self.time=Time() 
         self.duration=Duration() 


      def from_sec(self,float_secs):
         self.time.secs = int(float_secs)
         self.time.nsecs = int((float_secs - self.time.secs) * 1000000000)
         return self.time

      def to_secs(self):
         self.float_secs = time.time_ns()/1000000000.0
         return self.float_secs

      def set_zero(self,time1):
         self.time.secs=0
         self.time.nsecs=0
         return self.time


      def __add__(self,time1, time2):
         self.time.secs=time1.secs + time2.secs
         self.time.nsecs=time1.nsecs + time2.nsecs
         return self.time
      
      def __sub__(self,time1, time2):
         self.time.secs=time1.secs - time2.secs
         self.time.nsecs=time1.nsecs - time2.nsecs
         return time3

      def time_now_o(self):
         total_time=time.time()
         self.time.secs=int(total_time )
         self.time.nsecs = int((total_time - self.time.secs) * 1000000000)
         return self.time
        
      def time_now(self):
         total_time=time.time_ns()
         self.time.secs=int(total_time / 1000000000)
         self.time.nsecs = int((total_time - time1.secs) * 1000000000)
         return self.time

      def duration(self,dur):
         self.duration.secs = int(dur)
         self.duration.nsecs = int((float_secs - time1.secs) * 1000000000)
         return self.duration
