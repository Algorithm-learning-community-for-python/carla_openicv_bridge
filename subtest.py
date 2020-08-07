import zmq
from zmq import Context
from protocol.sensor_msgs.msg import CameraInfo
from protocol.sensor_msgs.msg import Image
from protocol.nav_msgs.msg import Odometry
import cv2
from protocol.std_msgs.msg import Header
import math
import io
from thirdparty.cv_bridge import CvBridge


cv_bridge = CvBridge()


camera_info = Odometry()
camera_ = Image()


ctx1=Context.instance()
url1 = "ipc:///tmp/buff_carla_ego_vehicle_odometry"
sub1 = ctx1.socket(zmq.SUB)

try:
	sub1.connect(url1)
except:
	print("not ok")
sub1.setsockopt(zmq.SUBSCRIBE, b'')

ctx2=Context.instance()
url2 = "ipc:///tmp/buff_carla_ego_vehicle_camera_rgb_front_image_color"
sub2 = ctx2.socket(zmq.SUB)
sub2.connect(url2)
sub2.setsockopt(zmq.SUBSCRIBE, b'')
while True:
	#print("trying")
	msg = sub1.recv()      # ERROR: WAITS FOREVER
	r=camera_info.deserialize(msg)
	print(r)

	msg1 = sub2.recv()      # ERROR: WAITS FOREVER
	#print(len(msg1))
	img=camera_.deserialize(msg1)
	pic=cv_bridge.imgmsg_to_cv2(img, "passthrough")
	cv2.imshow("depth",pic)
	cv2.waitKey(40)
