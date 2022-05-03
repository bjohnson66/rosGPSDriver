import numpy as np
import rospy
from std_msgs.msg import Float32

#RobotX Simulator gps sensor for default robot
from sensor_msgs.msg import NavSatFix

#Robot simulator (inertial measurement unit) sensor for dfault robot
from sensor_msgs.msg import Imu

#Used to convert Quaternion to euler angles
from tf.transformations import euler_from_quaternion, quaternion_from_euler


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
		KPV = 6000
		KPW = 6000
		C = 1
		D = 1

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

		V = KPV*Xerr
		W = KPV*Yerr
		
		############################################
		#Given System of 2 equations 2 unknowns:
		#V = (Pow_L + Pow_R)*C
		#W = (Pow_L - Pow_R)*D
		#SOLVING EXPLICITLY FOR Pow_L and POw_R:
		############################################
		#multpiled by negative 1 because it kept going backwards?
		Pow_L = ((-W/D) + (V/C))/2
		Pow_R = ((V/C) - Pow_L)
		
		if (Print):
			print("Left: " + str(Pow_L) + " Right: " + str(Pow_R))
			rospy.sleep(0.01)
			
		#Having found how much power to give each, drive
		self.drive(-1*Pow_L,-1*Pow_R, Print)

		#return true if we are in the correct position (within a given radius)
		return(self.arrivedAtLocation(targetLat, targetLong,Print))
		
		
	#####################################################################
	#Name: drive_To_Point_A(self, targetLat, targetLong, Print)
	#Programmer: Bradley Johnson
	#Date: 04/12/2022
	#Description: An ealier implementation of driveToPoint, which calculates the angle
	#		to the target and sends commands to the thrusters based off of the sin
	#		of the angle. This implementation is more intuative right off the bat,
	#		and has survived more testing (as of 4/26/22) so it is used as a back up
	#		navigator fuction.If we detect that we are off course, we can call this function instead 
	# 		to ensure that we reach the target and dont get lost at sea (or lake)
	#Inputs:targetLat: float representation of the targets Latitude
	#	targerLong: float representation of the target's longitude
	#	Print: boolean to determine whether or not the funciton prints diagnostic informaion to
	#	       terminal: defaults to false
	####################################################################
	def drive_To_Point_A(self, targetLat, targetLong, Print = True):
		############CONVERT RAW QUATERNION TO ELER ANGLE##############
		orientation_list = [self.orientation.x,self.orientation.y, self.orientation.z, self.orientation.w]
		(roll,pitch,yaw) = euler_from_quaternion (orientation_list)
		
		#adjust angle so that 0 is north and values range from 0 to 2pi
		yaw = yaw - np.pi/2 #adjuest by pi/2 so that 0 and 2pi is facing the direction of our boat
		if (yaw <0):
			yaw = yaw +2*np.pi
			
		if (Print):
			print("Driving to Lat:", targetLat," Long: ",targetLong)
			print("Current angle from imu (yaw) from north: ", yaw)
		

		#beta is the differenve between the current angle(directoin) and the direction of the target
		angleToTarget = np.arctan2((targetLong - self.long),(targetLat - self.lat) )
		
		beta = yaw + angleToTarget
		if (Print):
			print("Angle to target: ", angleToTarget)
			print("Difference Angle Beta: ", beta)
			rospy.sleep(0.01)
		
		
		#Driving logic based on sin of the beta angle. 
		if ((np.sin(beta) +1 <  1.1) and (np.sin(beta) + 1 > 0.99)):
			self.drive(1, 1) #full Thrust on both engines if headed in teh general direction 
			if (Print):
				print("Full Power")
		elif (np.sin(beta) < 0):
			self.drive(0.0, 1) #full Thrust on right engine to turn left slightly 
		else:
			self.drive(1, 0.0) #Full Thrust on left engine to turn right slightly
	
		
		#return true if we are in the correct position (within a given radius)
		return(self.arrivedAtLocation(targetLat, targetLong,Print))
################################################## END CLASS DEFINITION ###################################################
		
	
#########################################    MAIN   ############################################################	
if __name__ == '__main__':
	boat = GPSDriver()
	boolean = 1
	while(boolean == 1):
		left = float(input("Enter left magnitude: "))
		right = float(input("Enter right magnitude: "))
		duration = int(input("Enter how long  you want it to loop"))
		
		for i in range(duration):
			boat.drive(left,right)
			rospy.sleep(1)
		switch = int(input("Go home? 1 = yes via marix navigation, 2 = yes via angle navigation"))
		if (switch == 1):
			arrivedAtTarget = False;
			while(arrivedAtTarget == False):
				arrivedAtTarget = boat.drive_To_Point_M(boat.homeLat,boat.homeLong, False)
		elif (switch == 2):
			arrivedAtTarget = False;
			while(arrivedAtTarget == False):
				arrivedAtTarget = boat.drive_To_Point_A(boat.homeLat,boat.homeLong, False)		
					
		boolean = int(input("Go again? 1 = yes, else quit"))
		
	

		
		
