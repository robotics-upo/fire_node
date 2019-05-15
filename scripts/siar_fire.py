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

        #Default parameters
        rospy.set_param('~method','thresh')
        rospy.set_param('~threshval',200)


    #Proccesing image from channel
    def processimage(self,msg):
        try:
            #0) Confirm message and convert to opencv message
            orig=self.br.imgmsg_to_cv2(msg,"bgr8")
            #drawimg=orig

            #1) Resize image (OPTIONAL)
            resized=cv2.resize(orig,None,fx=1.0,fy=1.0)
            drawimg=resized
            #showImg(drawimg,1)

            #2) Convert to single-channel image
            gray=cv2.cvtColor(resized,cv2.COLOR_BGR2GRAY)

            # Methods
            #   Binary image + umbral METHOD 1
            #   Sobel operator METHOD 2
                        
            if rospy.get_param('~method')=='thresh':

                #3) Convert to binary image
                ret,thresh = cv2.threshold(gray,rospy.get_param('~threshval'), 255, cv2.THRESH_BINARY)
                drawimg = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
                #showImg(drawimg,2)

                #4) Detect the largest area 
                imc,contours,h=cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                maxC=max(contours,key=lambda c: cv2.contourArea(c))

                ima=cv2.drawContours(drawimg,[maxC],-1,(0,255,0),1)
                #showImg(ima,3)

            elif rospy.get_param('~method')=='sobel':
                ddepth=cv2.CV_16S

                grad_x=cv2.Sobel(gray,ddepth,1,0,ksize=3)
                grad_y=cv2.Sobel(gray,ddepth,0,1,ksize=3)

                abs_grad_x=cv2.convertScaleAbs(grad_x)
                abs_grad_y=cv2.convertScaleAbs(grad_y)
                
                #Sum both contributions equally
                grad=cv2.addWeighted(abs_grad_x,0.5,abs_grad_y,0.5,0)
                
                #4) Detection of whitest pixel
                whitest=np.amax(grad)
                cols,rows=np.where(grad==whitest)

                #5) Draw the position of maximum gradient
                #Back to color channel to show figures in color
                drawimg=cv2.cvtColor(grad,cv2.COLOR_GRAY2BGR)
                for k in np.arange(rows.size):
                    x=rows[k]
                    y=cols[k]
                    cv2.circle(drawimg,(x,y),5,[0,0,255],1)
                    
                #showImg(drawimg,2)
                #6) Convert to gray and binary image with filtering
                gray=cv2.cvtColor(drawimg,cv2.COLOR_BGR2GRAY)

                ret,thresh=cv2.threshold(gray,rospy.get_param('~threshval'),255,cv2.THRESH_BINARY)
                drawImg = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)

                #7) Detect the largest area include the points
                imc,contours,h=cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                maxC=max(contours,key=lambda c: cv2.contourArea(c))

                ima=cv2.drawContours(drawimg,[maxC],-1,(0,255,0),2)
                #showImg(ima,3)
            else:
                print('No method used')

            #Final image to be published
            rospy.loginfo("publish image on fire topic")
            self.pub.publish(self.br.cv2_to_imgmsg(ima))  
            self.loop_rate.sleep()

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
        rospy.init_node("fire_siar") #Optional anonymous=True
        my_node=Node()
        my_node.startnode()
    except rospy.ROSInterruptException:
        pass
