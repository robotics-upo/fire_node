#!/usr/bin/env python
#ROS PYTHON
import rospy

#CV module and route
import cv2

#Numpy
import numpy as np

#Convert cv file ros files
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

#Save files
import os
import random

class Node(object):
    def __init__(self):
        #Package bridge
        self.br=CvBridge()

        #Node cycle
        self.loop_rate=rospy.Rate(10)

        #Publisher
        self.pub=rospy.Publisher("fire",Image,queue_size=10)
        
        #Subscriber
        rospy.Subscriber("flip_image",Image,self.processimage)

        #Default common parameters
        rospy.set_param('~path','/home/sergiod/catkin_ws/src/fireimage/images')
        rospy.set_param('~save',0)
        rospy.set_param('~threshval',200)
        rospy.set_param('~rows',2)
        rospy.set_param('~cols',1)

    #Proccesing image from channel
    def processimage(self,msg):
        try:
            #1) Confirm message and convert to opencv message
            img=self.br.imgmsg_to_cv2(msg,"bgr8")
            #showImg(img,1)

            #2) Convert to single channel image and then to binary image
            ret,thresh = cv2.threshold(cv2.cvtColor(img,cv2.COLOR_BGR2GRAY),rospy.get_param('~threshval'), 255, cv2.THRESH_BINARY)      
                
            #3) Remove the noise through erode+dilation
            kernel=np.ones((rospy.get_param('~rows'),rospy.get_param('~cols')),np.uint8)
            binimg=cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel)

            #4) Detect the fire
            imc,contours,h=cv2.findContours(binimg,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            if(contours==[]):
                ima=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            else:
                maxC=max(contours,key=lambda c: cv2.contourArea(c))
                ima=cv2.drawContours(cv2.cvtColor(cv2.cvtColor(img,cv2.COLOR_BGR2GRAY),cv2.COLOR_GRAY2BGR),[maxC],-1,(0,0,255),1)
            #showImg(ima,2)

            #Final image to be published
            rospy.loginfo("publish image on fire topic")
            self.pub.publish(self.br.cv2_to_imgmsg(ima))  
            self.loop_rate.sleep()

            #Save images for later analysis
            if(rospy.get_param('~save')==1):
                number=random.randint(1e4,2e4)
                string='fire'+str(number)+'.jpg'
                cv2.imwrite(os.path.join(rospy.get_param('~path'),string),ima)

        except Exception as err:
            print err

    def startnode(self):
        rospy.loginfo('fire siar node started')
        rospy.spin()

#Show images in different displays
def showImg(img,number):
    string='fire'+str(number)
    cv2.imshow(string,img)
    cv2.waitKey(1)
    
if __name__=="__main__":
    try:
        rospy.init_node("fire_siar")
        my_node=Node()
        my_node.startnode()
    except rospy.ROSInterruptException:
        pass
