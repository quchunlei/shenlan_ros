#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError

moveBindings = {
        'i':(1,0),
        'o':(1,-1),
        'j':(0,1),
        'l':(0,-1),
        'u':(1,1),
        ',':(-1,0),
        '.':(-1,1),
        'm':(-1,-1),
           }

speed = .6
turn = 2

class faceDetector:
    def __init__(self):
        rospy.on_shutdown(self.cleanup);

        # 创建cv_bridge
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
	self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # 获取haar特征的级联表的XML文件，文件路径在launch文件中传入
        cascade_1 = rospy.get_param("~cascade_1", "")
        cascade_2 = rospy.get_param("~cascade_2", "")

        # 使用级联表初始化haar特征检测器
        self.cascade_1 = cv2.CascadeClassifier(cascade_1)
        self.cascade_2 = cv2.CascadeClassifier(cascade_2)

        # 设置级联表的参数，优化人脸识别，可以在launch文件中重新配置
        self.haar_scaleFactor  = rospy.get_param("~haar_scaleFactor", 1.2)
        self.haar_minNeighbors = rospy.get_param("~haar_minNeighbors", 2)
        self.haar_minSize      = rospy.get_param("~haar_minSize", 40)
        self.haar_maxSize      = rospy.get_param("~haar_maxSize", 60)
        self.color = (50, 255, 50)

        # 初始化订阅rgb格式图像数据的订阅者，此处图像topic的话题名可以在launch文件中重映射
        self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback, queue_size=1)

    def image_callback(self, data):
	x = 0
        th = 0
        status = 0
        count = 0
        acc = 0.1
        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0
        #key = getKey()
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")     
            frame = np.array(cv_image, dtype=np.uint8)
        except CvBridgeError, e:
            print e

        # 创建灰度图像
        grey_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 创建平衡直方图，减少光线影响
        grey_image = cv2.equalizeHist(grey_image)

        # 尝试检测人脸
        faces_result = self.detect_face(grey_image)

        # 在opencv的窗口中框出所有人脸区域
        if len(faces_result)>0:
            for face in faces_result: 
                x, y, w, h = face
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), self.color, 2)
		mid_x=x+w/2
		mid_y=y+h/2

		# 根据脸的位置发布速度消息
		if  (mid_y<160 and 200<mid_x and mid_x<440):
		    x = moveBindings['i'][0]
		    th = moveBindings['i'][1]
		    count = 0
		# 后退
		elif (mid_y>320 and 200<mid_x and mid_x<440):
		    x = moveBindings[','][0]
		    th = moveBindings[','][1]
		    count = 0
		# 停止
		elif (mid_y>160 and mid_y<320 and 200<mid_x and mid_x<440):
		    x = 0
		    th = 0
		    control_speed = 0
		    control_turn = 0
		# 左转
		elif (mid_y>160 and mid_y<320 and mid_x<200):
		    x = moveBindings['j'][0]
		    th = moveBindings['j'][1]
		    count = 0
		# 右转
		elif (mid_y>160 and mid_y<320 and mid_x>440):
		    x = moveBindings['l'][0]
		    th = moveBindings['l'][1]
		    count = 0
		else:
		    count = count + 1
		    if count > 4:
		        x = 0
		        th = 0
		    #if (key == '\x03'):
		    	#break

		# 目标速度=速度值*方向值
		target_speed = speed * x
		target_turn = turn * th

		# 速度限位，防止速度增减过快
		if target_speed > control_speed:
		    control_speed = min( target_speed, control_speed + 0.06 )
		elif target_speed < control_speed:
		    control_speed = max( target_speed, control_speed - 0.06 )
		else:
		    control_speed = target_speed

		if target_turn > control_turn:
		    control_turn = min( target_turn, control_turn + 0.2 )
		elif target_turn < control_turn:
		    control_turn = max( target_turn, control_turn - 0.2 )
		else:
		    control_turn = target_turn

        # 将识别后的图像转换成ROS消息并发布
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        # 创建并发布twist消息
        twist = Twist()
        twist.linear.x = control_speed; 
        twist.linear.y = 0; 
        twist.linear.z = 0
        twist.angular.x = 0; 
        twist.angular.y = 0; 
        twist.angular.z = control_turn
        self.vel_pub.publish(twist)

    def detect_face(self, input_image):
        # 首先匹配正面人脸的模型
        if self.cascade_1:
            faces = self.cascade_1.detectMultiScale(input_image, 
                    self.haar_scaleFactor, 
                    self.haar_minNeighbors, 
                    cv2.CASCADE_SCALE_IMAGE, 
                    (self.haar_minSize, self.haar_maxSize))
                                         
        # 如果正面人脸匹配失败，那么就尝试匹配侧面人脸的模型
        if len(faces) == 0 and self.cascade_2:
            faces = self.cascade_2.detectMultiScale(input_image, 
                    self.haar_scaleFactor, 
                    self.haar_minNeighbors, 
                    cv2.CASCADE_SCALE_IMAGE, 
                    (self.haar_minSize, self.haar_maxSize))
        
        return faces

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("face_detector")
        faceDetector()
        rospy.loginfo("Face detector is started..")
        rospy.loginfo("Please subscribe the ROS image.")
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down face detector node."
        cv2.destroyAllWindows()
