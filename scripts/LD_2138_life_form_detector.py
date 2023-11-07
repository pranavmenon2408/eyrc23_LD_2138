#!/usr/bin/env python3

from swift_msgs.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64,String
from pid_tune.msg import PidTune
from math import fabs
import rospy
import time
from luminosity_drone.msg import centroid,whycons,Biolocation
from imutils import contours
from skimage import measure
import numpy as np
import imutils
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class swift():
    """docstring for swift"""

    def __init__(self):

        rospy.init_node('drone_control')	# initializing ros node with name drone_control

        # This corresponds to your current position of drone. This value must be updated each time in your whycon callback
        # [x,y,z]
        self.drone_position = [0.0,0.0,0.0]	

        # [x_setpoint, y_setpoint, z_setpoint]
        self.setpoint=[[0,0,30]]
        self.search_setpoints=[[-8,8,30],[-8,-8,30],[-6,-6,30],[7,-7,30],[5,-5,30],[6,8,30],[2,2,30],[11,11,35],[11,11,37]]
        self.ind=0 # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly


        #Declaring a cmd of message type swift_msgs and initializing values
        self.cmd = swift_msgs()
        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcAUX1 = 1500
        self.cmd.rcAUX2 = 1500
        self.cmd.rcAUX3 = 1500
        self.cmd.rcAUX4 = 1500


        #initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
        #after tuning and computing corresponding PID parameters, change the parameters

        self.Kp = [120*0.06, 100*0.06, 100*0.06]
        self.Ki = [5*0.0008, 0, 150*0.0008]
        self.Kd = [150*0.3, 0, 1000*0.3]

        #-----------------------Add other required variables for pid here ----------------------------------------------
        self.error=[0,0,0]
        self.prev_error=[0,0,0]
        self.min_values=[1000,1000,1000]
        self.max_values=[2000,2000,2000]
        self.diff_error=[0,0,0]
        self.iterm_error=[0,0,0]
        self.now=0.0
        self.last_time=0.0
        self.timechange=0.0
        self.out_roll=0
        self.out_pitch=0
        self.out_throttle=0
        self.goal_reached=False
        self.index=0
        self.sample_time = 0.060
        self.alien_flag=0

        self.done=0
        self.research=0
        self.disarm_flag=0
        self.align_x=0.0
        self.align_y=0.0
        self.alien_detected=0
        self.wait=0
        self.wait2=0
        self.threshold=0.20

        # Publishing /drone_command, /alt_error, /pitch_error, /roll_error
        self.command_pub = rospy.Publisher('/drone_command', swift_msgs, queue_size=1)
        #------------------------Add other ROS Publishers here-----------------------------------------------------
        self.alt_error=rospy.Publisher('/alt_error',Float64,queue_size=1)
        self.pitch_error=rospy.Publisher('/pitch_error',Float64,queue_size=1)
        self.roll_error=rospy.Publisher('/roll_error',Float64,queue_size=1)

        self.whycons=rospy.Publisher('/whycons',whycons,queue_size=10)
        self.wc=whycons()

        # Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll

        rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
        rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
        #-------------------------Add other ROS Subscribers here----------------------------------------------------
        rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
        rospy.Subscriber('/astrobiolocation',Biolocation,self.get_result)
        rospy.Subscriber('/diff',centroid,self.get_xy)

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
        self.image_sub = rospy.Subscriber("/swift/camera_rgb/image_raw",Image,self.callback)
        rospy.Subscriber('/whycons',whycons,self.get_pos)



        #------------------------------------------------------------------------------------------------------------
        self.image_flag=0
        self.arm() # ARMING THE DRONE

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
                #cv2.circle(current_frame,(int(cX),int(cY)),int(radius),(0,0,255),3)
                '''cv2.putText(current_frame, "#{}".format(i + 1), (x, y - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)'''




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
        
            
            
        #cv2.imshow('led_image',current_frame)
        #cv2.waitKey(3)
        self.pub.publish(self.br.cv2_to_imgmsg(current_frame,"bgr8"))

    def get_result(self,msg):
        rospy.loginfo(f"I heard {msg} ")

    def get_xy(self,msg):
        self.align_x=msg.x
        self.align_y=msg.y
        rospy.loginfo(msg)



    def disarm(self):
        self.cmd.rcAUX4 = 1100
        self.command_pub.publish(self.cmd)
        rospy.sleep(1)


    # Arming condition of the drone : Best practise is to disarm and then arm the drone.
    def arm(self):

        self.disarm()

        self.cmd.rcRoll = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcThrottle = 1000
        self.cmd.rcAUX4 = 1500
        self.command_pub.publish(self.cmd)	# Publishing /drone_command
        rospy.sleep(1)




    def whycon_callback(self,msg):
        self.drone_position[0] = msg.poses[0].position.x

        #--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
        self.drone_position[1]=msg.poses[0].position.y
        self.drone_position[2]=msg.poses[0].position.z



    #---------------------------------------------------------------------------------------------------------------




    def altitude_set_pid(self,alt):
        self.Kp[2] = alt.Kp * 0.06# This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[2] = alt.Ki * 0.0008
        self.Kd[2] = alt.Kd * 0.3

    #----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------
    def pitch_set_pid(self,pitch):
        self.Kp[1]=pitch.Kp*0.06
        self.Ki[1] =pitch.Ki * 0.0008
        self.Kd[1] = pitch.Kd * 0.3
    def roll_set_pid(self,roll):
        self.Kp[0]=roll.Kp*0.06
        self.Ki[0] = roll.Ki * 0.0008
        self.Kd[0] = roll.Kd * 0.3


    def pid(self):

        alien_detected=0
        #threshold=0.20

        if self.disarm_flag==1:
            self.cmd.rcRoll = 0
            self.cmd.rcYaw = 0
            self.cmd.rcPitch = 0
            self.cmd.rcThrottle = 0
            self.cmd.rcAUX4 = 0
            self.command_pub.publish(self.cmd)

        else:





            if fabs(self.align_x) > 0.0 and fabs(self.align_y)>0.0:
                #if self.wait==0:
                if fabs(self.align_x)<20 and fabs(self.align_y)<30:

                    if self.alien_flag==0:
                        self.wc.x=self.drone_position[0]
                        self.wc.y=self.drone_position[1]
                        self.wc.z=self.drone_position[2]
                        self.whycons.publish(self.wc)
                        self.alien_flag=1
                #alien_detected=1


                else:

                    self.setpoint.append([self.drone_position[0]+self.align_x/200,self.drone_position[1]+self.align_y/200,30])

                    #print("Alien detected")
                    self.wc.x=0.0
                    self.wc.y=0.0
                    self.wc.z=0.0
                    self.whycons.publish(self.wc)
                    self.alien_detected=1	


            '''if (self.alien_flag==1):
                if(abs(self.drone_position[0]-self.wc.x)>3.0):
                    self.alien_flag=0
                    self.wait=0
                else:
                    self.wait=1'''






            if((0<abs(self.prev_error[0])<=self.threshold) and (0<abs(self.prev_error[1])<=self.threshold) and (0<abs(self.prev_error[2])<=self.threshold)):

                if self.research==1:
                    #self.disarm()
                    self.cmd.rcRoll = 0
                    self.cmd.rcYaw = 0
                    self.cmd.rcPitch = 0
                    self.cmd.rcThrottle = 0
                    self.cmd.rcAUX4 = 0
                    self.command_pub.publish(self.cmd)
                    self.disarm_flag=1



                


                '''if len(self.setpoint)==1 or (self.alien_flag==1):
                    if self.ind==len(self.search_setpoints):
                        self.research=1
                        self.alien_detected=0

                    if self.ind<len(self.search_setpoints):
                        self.setpoint=[self.search_setpoints[self.ind]]
                        self.ind+=1
                        self.index=0
                        #self.alien_flag=0
                        if len(self.setpoint)==1:
                            self.alien_detected=0'''
                if self.alien_detected==1:
                    self.index+=1


                if self.alien_flag==1:
                    if self.research==0:
                        if self.index>0:
                            self.setpoint=[self.search_setpoints[-2],self.search_setpoints[-1]]
                            self.index=0
                            self.alien_detected=0

                        else:	
                            self.index+=1
                            self.research=1
                            self.threshold=0.10





                if len(self.setpoint)==1 and self.alien_flag!=1:
                    if self.ind<len(self.search_setpoints):
                        self.setpoint=[self.search_setpoints[self.ind]]
                        self.ind+=1
                        self.index=0
                        self.alien_detected=0
                    else:
                        self.research=1

                















            self.error[0]=-(self.drone_position[0]-self.setpoint[self.index][0])
            self.error[1]=self.drone_position[1]-self.setpoint[self.index][1]
            self.error[2]=self.drone_position[2]-self.setpoint[self.index][2]
            self.diff_error[0]=self.error[0]-self.prev_error[0]
            self.diff_error[1]=self.error[1]-self.prev_error[1]
            self.diff_error[2]=self.error[2]-self.prev_error[2]
            self.iterm_error[0]+=self.error[0]
            self.iterm_error[1]+=self.error[1]
            self.iterm_error[2]+=self.error[2]
            self.out_roll=self.Kp[0]*self.error[0]+self.Kd[0]*self.diff_error[0]+self.Ki[0]*self.iterm_error[0]
            self.out_pitch=self.Kp[1]*self.error[1]+self.Kd[1]*self.diff_error[1]+self.Ki[1]*self.iterm_error[1]
            self.out_throttle=self.Kp[2]*self.error[2]+self.Kd[2]*self.diff_error[2]+self.Ki[2]*self.iterm_error[2]

            self.cmd.rcRoll=int(1500+self.out_roll)
            self.cmd.rcPitch=int(1500+self.out_pitch)
            self.cmd.rcThrottle=int(1500+self.out_throttle)

            if(self.cmd.rcRoll>self.max_values[0]):
                self.cmd.rcRoll=self.max_values[0]
            if(self.cmd.rcRoll<self.min_values[0]):
                self.cmd.rcRoll=self.min_values[0]

            if(self.cmd.rcPitch>self.max_values[1]):
                self.cmd.rcPitch=self.max_values[1]
            if(self.cmd.rcPitch<self.min_values[1]):
                self.cmd.rcPitch=self.min_values[1]

            if(self.cmd.rcThrottle>self.max_values[2]):
                self.cmd.rcThrottle=self.max_values[2]
            if(self.cmd.rcThrottle<self.min_values[2]):
                self.cmd.rcThrottle=self.min_values[2]

            self.cmd.rcRoll=int(self.cmd.rcRoll)
            self.cmd.rcPitch=int(self.cmd.rcPitch)
            self.cmd.rcThrottle=int(self.cmd.rcThrottle)

            self.command_pub.publish(self.cmd)

            self.prev_error[0]=self.error[0]
            self.prev_error[1]=self.error[1]
            self.prev_error[2]=self.error[2]




            self.roll_error.publish(self.error[0])
            self.pitch_error.publish(self.error[1])
            self.alt_error.publish(self.error[2])


if __name__ == '__main__':

    swift_drone = swift()
    r = rospy.Rate(33) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        swift_drone.pid()
        r.sleep()



