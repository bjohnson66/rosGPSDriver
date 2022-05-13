import numpy as np
import rospy
from std_msgs.msg import Float32

#RobotX Simulator gps sensor for default robot
from sensor_msgs.msg import NavSatFix

#Robot simulator (inertial measurement unit) sensor for dfault robot
from sensor_msgs.msg import Imu

#Used to convert Quaternion to euler angles
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#################################################TODO########################################################
#Fix K constants, its really unstable right now
#Add comment block above GPS Driver class
#
#################################################################################################################





class GPSDriver:
	##########################
	#Constructor:
	##########################
	def __init__(self):
		#'boat_driver' is an arbitrary name, it could be anything
		rospy.init_node('boat_driver', anonymous=True)
		
		#Let the script run at 100hz
		self.rate = rospy.Rate(100)
		
		#create basically short hand/aliases for ros commands
		self.right_thrust = 'wamv/thrusters/right_thrust_cmd'
		self.left_thrust = 'wamv/thrusters/left_thrust_cmd'
		
		self.gps = 'wamv/sensors/gps/gps/fix'
		self.imu = 'wamv/sensors/imu/imu/data'
		
		#####################################################3
		# Publisher objects from Ros for each direction/rotation, using 
		# the topics specififed above. 
		######################################################
		self.right_thrust_pub = rospy.Publisher(self.right_thrust,Float32, queue_size=10)
		self.left_thrust_pub = rospy.Publisher(self.left_thrust, Float32, queue_size=10)
			
			
		#Commented out because the real IMARC boat only has left and right thrust control	
		#self.right_angle_pub = rospy.Publisher(self.right_angle,Float32, queue_size=10)
		#self.left_angle_pub = rospy.Publisher(self.left_angle,Float32, queue_size=10)
		#self.lat_thrust_pub = rospy.Publisher(self.lat_thrust, Float32, queue_size=10)
		#self.lat_angle_pub = rospy.Publisher(self.lat_angle,Float32, queue_size=10)
		
		#not needed yet TODO
		# set p initial accelerations to 0
		# self.previousAcceleration = 0.0
		# self.currentAcceleration = 0.0
		
		
		self.e_integral_sum_x =0.0
		self.e_integral_sum_y = 0.0
		
		self.prev_y_err = 0.0
		self.prev_x_err = 0.0
		
		rospy.sleep(0.5)
		
		self.gps_sub = rospy.Subscriber(self.gps,NavSatFix,self.gps_cb, queue_size=10)
		self.imu_sub = rospy.Subscriber(self.imu,Imu,self.imu_cb, queue_size=10) 
		
		
	
		######################################################################
		#Set the home corrinates to the position when this node is started
		######################################################################
		rospy.wait_for_message(self.gps,NavSatFix)
		
		self.homeLat = self.lat
		self.homeLong = self.long
		
	
	#METHODS:
	
	
	#########################################################################
	#Updates the long and lat when a message is published on the gps topic
	#########################################################################
	def gps_cb(self, data):
		self.lat = data.latitude
		self.long = data.longitude
		
	
	#########################################################################
	#Updates the orientation quaternion when a the imu topic publishes something	
	#########################################################################
	def imu_cb(self, data):
		self.orientation = data.orientation
		#not needed yet TODO
		#self.previousAcceleration = self.currentAcceleration
		#self.currentAcceleration = data.linear_acceleration
		
		
		
	
		
	####################################################################
	#Name: drive(self, xMag, yMag)
	#Programmer: Bradley Johnson
	#Date: 3/22/2022
	#Descrption: Will move the boat by publising one thrust comand for 
	#            the left and right thrusters. 
	#Inputs: xMag: the desired magnitude of left propulsion impulse
	#	 yMag: the desired magnitude of right propulsion impulse
	####################################################################
	def drive(self, leftMag = 1.0, rightMag = 1.0, Print = False):
		rospy.init_node('boat_driver', anonymous = True)
		self.right_thrust_pub.publish(float(rightMag))
		self.left_thrust_pub.publish(float(leftMag))
		if (Print):
			print("I am at Latitude: ",self.lat, " Longitude: ", self.long)

	##############################################################################
	#Name: arrivedAtLocation(self, targetLat, targetLong, distance, Print = False)
	#Programmer: Bradley Johnson
	#Date: 4/26/2022
	#Description: Will return true if the boat is within a certain distance of the target
	#Inputs:targetLat: float representation of the targets Latitude
	#	targerLong: float representation of the target's longitude
	#	distance: defaults to what should be about 5.55 meters of the target 
	# 		  based on the conversions found here: 
	#		  https://www.usna.edu/Users/oceano/pguth/md_help/html/approx_equivalents.htm
	#	Print: boolean to determine whether or not the funciton prints diagnostic informaion to
	#	       terminal. Defaults to false
	##############################################################################
	def arrivedAtLocation(self, targetLat, targetLong, Print = False, distance = 0.00005):
		#return true if we are in the correct position (within a given radius)
		if (((self.lat - targetLat) * (self.lat - targetLat) + (self.long - targetLong) * (self.long - targetLong)) < distance*distance):
			if (Print):
				print("Arrived at ", targetLat, " , ", targetLong)
			
			return True #We've made it within a few meters of the target
			
		#else return false
		return False;	
		
		
	#####################################################################
	#Name: drive_To_Point_M(self, targetLat, targetLong)
	#Pragrammer: Bradley Johnson
	#Date: 4/26/2022
	#Description: driveToPoint takes a target position (Lattitude and Longitude)
	#	      ad uses a Rotation Matrix, and <I DONT REMEMBER THE NAME OF THE OTHER MATH>
	# 	      to calculate how much power should be sent to the thrusters in order to propell
	#	      the boat to the target. As the boat gets closer, it gives less power to the engines
	# 	       so that it comes to a slow stop.
	#Inputs:targetLat: float representation of the targets Latitude
	#	targerLong: float representation of the target's longitude
	#	Print: boolean to determine whether or not the funciton prints diagnostic informaion to
	#	       terminal. Defaults to false
	####################################################################
	def drive_To_Point_M(self, targetLat, targetLong, Print = False):
		#Consants
		KPV = 1
		KPW = 1
		KI = 1
		KD = 1
		C = 0.9
		D = 0.9

		############CONVERT RAW QUATERNION TO ELER ANGLE##############
		orientation_list = [self.orientation.x,self.orientation.y, self.orientation.z, self.orientation.w]
		(roll,pitch,yaw) = euler_from_quaternion (orientation_list)
		
		#targetX is the x position of the target in the robos frame of ref
		targetX = self.long - targetLong;
		targetY = self.lat - targetLat;

		#let Theta = yaw, Create rotation maxtrix 
		topLeft = np.cos(yaw)
		topRight = (-1*np.sin(yaw))
		bottomLeft = (np.sin(yaw))
		bottomRight = np.cos(yaw)
		Rotation_Marix = np.matrix([[topLeft, topRight],[bottomLeft, bottomRight]])
	
		#use target x and y error to create 1 by 2 vector for it's position
		target = np.matrix([targetX,targetY])
		
		
		product = np.matmul(target,np.matrix([[topLeft, topRight],[bottomLeft, bottomRight]]))
		
		Xerr = product[0,0];
		Yerr = product[0,1];
		
		#assume delat time is 0.01 seconds based off the 100Hz refresh rate set by rospy.Rate(100) in the constructor
		self.e_integral_sum_x += Xerr*0.01
		self.e_integral_sum_y += Yerr*0.01



		#TODO balance/fix constants
		#Consants TODO place at top again
		KPV = 100
		KIV = 5
		KDV = 5
		
		KPW = 100
		KIW = 0
		KDW = 0
		
		
		C = 0.9
		D = 0.9

		V = KPV*Xerr + KIV*self.e_integral_sum_x + KDV*((Xerr-self.prev_x_err)*100)
		W = KPW*Yerr + KIW*self.e_integral_sum_y + KDW*((Yerr-self.prev_y_err)*100)
		
		
		
		############################################
		#Given System of 2 equations 2 unknowns:
		#V = (Pow_L + Pow_R)*C
		#W = (Pow_L - Pow_R)*D
		#SOLVING EXPLICITLY FOR Pow_L and POw_R:
		############################################
		Pow_L = ((-W/D) + (V/C))/2
		Pow_R = ((V/C) - Pow_L)
		
		if (Print):
			print("Left: " + str(Pow_L) + " Right: " + str(Pow_R))
			rospy.sleep(0.01)

			
			
			
			
			
			
			
			
			
		#Having found how much power to give each, drive
		#multpiled by negative 1 because it kept going backwards?
		self.drive(-1*Pow_L,-1*Pow_R, Print)

		#return true if we are in the correct position (within a given radius)
		if (self.arrivedAtLocation(targetLat, targetLong,Print) == True):
			#######TEMPORARY########
			#TODO: Look into the relevemce of this reset, we need it to reset the integral
			#but I dont know if we are supposed to make sure that we hit our traget trajectory,
			# this is just the target location
			self.e_integral_x = 0.0
			self.e_integral_y = 0.0
			return True
		else:
			return False
		
################################################## END CLASS DEFINITION ###################################################
		
	
#########################################    MAIN   ############################################################	
if __name__ == '__main__':
	boat = GPSDriver()
	boolean = 1
	while(boolean == 1):
	#TODO commmented part
		#We dont need this part anymore, fr degugging the controllelr, Im just dragging the boat around
		#and having it return home. It takes less time
		#left = float(input("Enter left magnitude: "))
		#right = float(input("Enter right magnitude: "))
		#duration = int(input("Enter how long  you want it to loop"))
		
		#for i in range(duration):
		#	boat.drive(left,right)
		#	rospy.sleep(1)
		
		switch = int(input("Go home? 1 = yes, else = no"))
		if (switch == 1):
			arrivedAtTarget = False;
			while(arrivedAtTarget == False):
				arrivedAtTarget = boat.drive_To_Point_M(boat.homeLat,boat.homeLong, False)	
					
		boolean = int(input("Go again? 1 = yes, else quit"))
		
	

		
		
