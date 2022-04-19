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
	def __init__(self):
		#'boat_driver' is an arbitrary name, it could be anything
		rospy.init_node('boat_driver', anonymous=True)
		
		#Let hte script run at 100hz
		self.rate = rospy.Rate(100)
		
		#create basically short hand/aliases for ros commands
		self.right_thrust = 'wamv/thrusters/right_thrust_cmd'
		#self.right_angle =  'wamv/thrusters/right_angle'
		
		self.left_thrust = 'wamv/thrusters/left_thrust_cmd'
		#self.left_angle =  'wamv/thrusters/left_angle'
	
		#self.lat_thrust = 'wamv/thrusters/lateral_thrust_cmd'
		#self.lat_angle =  'wamv/thrusters/lateral_angle'
		
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
		
	#Updates the long and lat when a message is published on the gps topic
	def gps_cb(self, data):
		self.lat = data.latitude
		self.long = data.longitude
	
	#Updates the orientation quaternion when a the imu topic publishes something	
	def imu_cb(self, data):
		self.orientation = data.orientation
		
	
		
	####################################################################
	#Name: drive(self, xMag, yMag)
	#Descrption: Will move the boat by publising one thrust comand for 
	#            the left and right thrusters. 
	#Inputs: xMag: the desired magnitude of left propulsion impulse
	#	 yMag: the desired magnitude of right propulsion impulse
	####################################################################
	def drive(self, leftMag = 1.0, rightMag = 1.0, Print = True):
		rospy.init_node('boat_driver', anonymous = True)
		self.right_thrust_pub.publish(float(rightMag))
		self.left_thrust_pub.publish(float(leftMag))
		if (Print):
			print("I am at Latitude: ",self.lat, " Longitude: ", self.long)


	
	#####################################################################
	#Name: driveToPoint(self, targetLat, targetLong)
	#
	#
	#
	####################################################################
	def driveToPoint(self, targetLat, targetLong, Print = True):
	
		#find direction we are facing angle theta
		#thinking of lat as x and long as y

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
		
	
		
		#STACK OVERFLOW ATTEMPT FROM HERE <failed>
		#dLong = (self.long - targetLong)*(np.pi/180)
		#dLat = (self.lat - targetLat)*(np.pi/180)
		
		#y = np.sin(dLong) * np.cos(targetLat*(np.pi/180))
		#x = np.cos(self.lat*(np.pi/180)) * np.sin(targetLat*(np.pi/180)) - np.sin(self.lat*(np.pi/180)) * np.cos(targetLat*(np.pi/180)) * np.cos(dLat)
		
		#angleToTarget = np.arctan2(y,x)
		#beta = yaw - angleToTarget
	
		#beta is the differenve between the current angle(directoin) and the direction of the target
		angleToTarget = np.arctan2((targetLong - self.long),(targetLat - self.lat) )
		
		
		

		beta = yaw + angleToTarget
		if (Print):
			print("Angle to target: ", angleToTarget)
			print("Difference Angle Beta: ", beta)
		

		#Drive ing logic based on sin of the beta angle. 
		if ((np.sin(beta) +1 <  1.1) and (np.sin(beta) + 1 > 0.99)):
			self.drive(1, 1) #full Thrust on both engines if headed in teh general direction 
			if (Print):
				print("Full Power")
		elif (np.sin(beta) < 0):
			self.drive(0.0, 1) #full Thrust on right engine to turn left slightly 
		else:
			self.drive(1, 0.0) #Full Thrust on left engine to turn right slightly
	
		
		
		#return true if we are in the correct position (within a given radius)
		if (((self.lat - targetLat) * (self.lat - targetLat) + (self.long - targetLong) * (self.long - targetLong)) < 0.00005*0.00005):
			print("Arrived at ", targetLat, " , ", targetLong)
			return True #We've made it within about 11.11 meters of the target
			
		return False;	
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
		boolean = int(input("Go home? 1 = yes, else no"))
		if (boolean == 1):
			while(boat.driveToPoint(boat.homeLat,boat.homeLong) == False):
				print("rest 0.1")
				rospy.sleep(0.08)
			
		boolean = int(input("Go again? 1 = yes, else quit"))
		
	

		
		
