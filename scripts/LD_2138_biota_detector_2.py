#!/usr/bin/env python3


from functools import reduce
#roslib.load_manifest('my_package')
import sys
import rospy
from imutils import contours
from skimage import measure
import numpy as np
import imutils
import cv2
from luminosity_drone.msg import centroid,whycons,Biolocation
from std_msgs.msg import String,Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class lifeform:
  
    def __init__(self):
        self.pub = rospy.Publisher("/video_frames",Image,queue_size=10)
        self.pub2=rospy.Publisher('/diff',centroid,queue_size=10)
        self.cen=centroid()
        self.image_flag=0
        
        self.br = CvBridge()
        self.aliens=0
        #self.image_sub = rospy.Subscriber("/swift/camera_rgb/image_raw",Image,self.callback)
        #rospy.Subscriber('/whycons',whycons,self.get_pos)
        self.pub3=rospy.Publisher('/astrobiolocation',Biolocation,queue_size=10)
        self.result=Biolocation()
        #self.image_sub2 = rospy.Subscriber("/whycon/image_out",Image,self.callback2)
    '''def callback2(self,msg):
        try:
            current_frame=self.br.imgmsg_to_cv2(msg,'bgr8')
        except CvBridgeError as e:
            print(e)
        print(current_frame.shape)'''

    def receive_message(self):
        self.image_sub = rospy.Subscriber("/swift/camera_rgb/image_raw",Image,self.callback)
        rospy.Subscriber('/whycons',whycons,self.get_pos)
        rospy.spin()

    
    def get_pos(self,msg):
        if abs(msg.x)>0.0 and abs(msg.y)>0.0 and abs(msg.z)>0.0:
            if self.aliens==2:
                self.result.organism_type="alien_a"
            elif self.aliens==3:
                self.result.organism_type="alien_b"
            elif self.aliens==4:
                self.result.organism_type="alien_c"
            self.result.whycon_x=msg.x
            self.result.whycon_y=msg.y
            self.result.whycon_z=msg.z
            self.pub3.publish(self.result)
            
  
    def callback(self,msg):
        self.image_flag=0
        try:
            current_frame=self.br.imgmsg_to_cv2(msg,"bgr8")
        except CvBridgeError as e:
            print(e)
        #print(current_frame.shape)
        #cv2.imshow('image',current_frame)
        #current_frame=cv2.cvtColor(current_frame,cv2.COLOR_RGB2BGR)
        gray=cv2.cvtColor(current_frame,cv2.COLOR_RGB2GRAY)
        blur=cv2.blur(gray,(11,11))
        # threshold the image to reveal light regions in the blurred image
        _,thresh=cv2.threshold(blur,127,255,cv2.THRESH_BINARY)

        # perform a series of erosions and dilations to remove any small blobs of noise from the thresholded image
        #kernel=np.ones((5,5),np.uint8)
        thresh=cv2.erode(thresh,None,iterations=2)
        thresh=cv2.dilate(thresh,None,iterations=4)
        # perform a connected component analysis on the thresholded image, then initialize a mask to store only the "large" components
        labels=measure.label(thresh, connectivity=2, background=0)
        mask=np.zeros(thresh.shape,dtype="uint8")
        # loop over the unique components
        for label in np.unique(labels):
            # if this is the background label, ignore it
            if label==0:
                continue

            # otherwise, construct the label mask and count the number of pixels 
            labelMask=np.zeros(thresh.shape,dtype="uint8")
            labelMask[labels==label]=255
            numPixels=cv2.countNonZero(labelMask)
            # if the number of pixels in the component is sufficiently large, then add it to our mask of "large blobs"
            if numPixels >300:
                mask=cv2.add(mask,labelMask)
            
        # find the contours in the mask, then sort them from left to right
        try:
            cnts=cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            cnts=imutils.grab_contours(cnts)
            cnts=contours.sort_contours(cnts)[0]
            # loop over the contours

            # Initialize lists to store centroid coordinates and area
            centroid=[]
            areacnt=[]
            # Loop over the contours
            for (i,c) in enumerate(cnts):


                # Calculate the area of the contour
                area=cv2.contourArea(c)

                # Draw the bright spot on the image
                (x,y,w,h)=cv2.boundingRect(c)
                ((cX,cY),radius)=cv2.minEnclosingCircle(c)
                cv2.circle(current_frame,(int(cX),int(cY)),int(radius),(0,0,255),3)
                cv2.putText(current_frame, "#{}".format(i + 1), (x, y - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)




                # Append centroid coordinates and area to the respective lists
                centroid.append((cX,cY))
                areacnt.append(area)
            # Save the output image as a PNG file
            #cv2.imwrite("LD_2138_led_detection_results.png", image)
            if len(centroid)>1:
                self.aliens=len(centroid)
                cx_avg=0
                cy_avg=0
                for i in range(len(centroid)):
                    cx_avg+=centroid[i][0]
                    cy_avg+=centroid[i][1]
                cx_avg/=len(centroid)
                cy_avg/=len(centroid)
                self.cen.x=cx_avg-(current_frame.shape[0]/2)
                self.cen.y=cy_avg-(current_frame.shape[1]/2)
                self.pub2.publish(self.cen)
                
                
                
                self.image_flag=1



            # Open a text file for writing
            
        except:
            pass
        # Close the text file

        
        if self.image_flag==0:
            self.cen.x=0.0
            self.cen.y=0.0
            self.pub2.publish(self.cen)
        
            
            
        cv2.imshow('led_image',current_frame)
        cv2.waitKey(3)
        self.pub.publish(self.br.cv2_to_imgmsg(current_frame,"bgr8"))
        
            
            
        
        

        
   
def main(args):
    
    rospy.init_node('life_form', anonymous=True)
    ic =lifeform()
    #r=rospy.Rate(17)
    try:
        ic.receive_message()
    except:
        print("Shutting down")
        cv2.destroyAllWindows()
  
if __name__ == '__main__':
    main(sys.argv)