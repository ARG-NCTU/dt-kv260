#!/usr/bin/env python3

import time
from pynq_dpu import DpuOverlay
# Load DPU uart overlay
overlay = DpuOverlay("/home/argnctu/dt-kv260/overlays/dpu_uartlite/dpu_uartlite.bit", download=False)
if not overlay.is_loaded():
    start = time.time()
    overlay.download()
    end = time.time()
    print("Load overlay using {}ms.".format((end-start)))

import os
import sys
import cv2
import numpy as np
import rospy
import rospkg
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError

class Lane_follow():
    def __init__(self):
        
        self.node_name = rospy.get_name()
        self.cv_bridge = CvBridge()
        self.omega = 0
        
        # motor omega output
        self.Omega = np.array([0.1,0.17,0.24,0.305,0.37,0.44,0.505,0.73,-0.1,-0.17,-0.24,-0.305,-0.37,-0.44,-0.505,-0.73,0.0,0.0])
        
        self.initial()

        # Subscriber
        self.image_sub = rospy.Subscriber("image_raw", Image, self.img_cb, queue_size=1)
        
        # Publisher
        self.pub_car_cmd = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        rospy.loginfo("[%s] init done." % (self.node_name))

    def initial(self):
        # Load pretrained model
        overlay.load_model("/home/argnctu/dt-kv260/catkin_ws/src/lane_following/weights/lane_following.xmodel")
        
        # Use Vitis AI Runtime
        self.dpu = overlay.runner

        inputTensors = self.dpu.get_input_tensors()
        outputTensors = self.dpu.get_output_tensors()

        shapeIn = tuple(inputTensors[0].dims)
        shapeOut = tuple(outputTensors[0].dims)

        self.input_data = [np.empty(shapeIn, dtype=np.float32, order="C")]
        self.output_data = [np.empty(shapeOut, dtype=np.float32, order="C")]

    def calculate_softmax(self, data):
        return np.exp(data)

    def preprocess_fn(self, image):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = np.asarray(image/255.0, dtype=np.float32)
        return image

    def img_cb(self, data):

        cv_img = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
        self.input_data[0] = self.preprocess_fn(cv_img)
        
        job_id = self.dpu.execute_async(self.input_data, self.output_data)
        self.dpu.wait(job_id)

        probs = self.calculate_softmax(self.output_data[0])
        top1 = np.argmax(probs)

        self.omega = self.Omega[top1]

        # motor control
        car_cmd_msg = Twist()
        car_cmd_msg.linear.x = 0.123
        car_cmd_msg.angular.z = self.omega*0.64
        self.pub_car_cmd.publish(car_cmd_msg)

if __name__=="__main__":
    rospy.init_node("lane_follow", anonymous=True)
    lane_follow = Lane_follow()
    rospy.spin()
