#!/usr/bin/env python3

from swift_msgs.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time

class swift():
	"""docstring for swift"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint=[[0,0,23],[2,0,23],[2,2,23],[2,2,25],[-5,2,25],[-5,-3,25],[-5,-3,21],[7,-3,21],[7,0,21],[0,0,21]] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly


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

		self.Kp = [100*0.06, 100*0.06, 120*0.06]
		self.Ki = [5*0.0008, 0, 150*0.0008]
		self.Kd = [200*0.3, 0, 600*0.3]
   
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

		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', swift_msgs, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------
		self.alt_error=rospy.Publisher('/alt_error',Float64,queue_size=1)
		self.pitch_error=rospy.Publisher('/pitch_error',Float64,queue_size=1)
		self.roll_error=rospy.Publisher('/roll_error',Float64,queue_size=1)

		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll

		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)



		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE


	# Disarming condition of the drone
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
	
		self.now = int(round(time.time() * 1000))
		self.timechange=self.now-self.last_time
		if (self.timechange>self.sample_time):
			if (self.last_time!=0):	
				if((0<abs(self.prev_error[0])<=0.15) and (0<abs(self.prev_error[1])<=0.15) and (0<abs(self.prev_error[2])<=0.15)):
					self.index+=1
					if self.index>=len(self.setpoint):
						self.index-=1

				
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
				

		self.last_time=self.now

		self.roll_error.publish(self.error[0])
		self.pitch_error.publish(self.error[1])
		self.alt_error.publish(self.error[2])


if __name__ == '__main__':

	swift_drone = swift()
	r = rospy.Rate(17) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		swift_drone.pid()
		r.sleep()



