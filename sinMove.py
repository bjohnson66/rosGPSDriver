import numpy as np
import rospy
from std_msgs.msg import Float32

#RobotX Simulator gps sensor for default robot
#from sensor_msgs.msg import NavSatFix

class BoatDriver:
	def __init__(self):
		#'boat_driver' is an arbitrary name, it could be anything
		rospy.init_node('boat_driver', anonymous=True)
		
		#Let hte script run at 100hz
		self.rate = rospy.Rate(100)
		
		#create basically short hand/aliases for ros commands
		self.right_thrust = 'wamv/thrusters/right_thrust_cmd'
		self.right_angle =  'wamv/thrusters/right_angle'
		
		self.left_thrust = 'wamv/thrusters/left_thrust_cmd'
		self.left_angle =  'wamv/thrusters/left_angle'
	
		self.lat_thrust = 'wamv/thrusters/lateral_thrust_cmd'
		self.lat_angle =  'wamv/thrusters/lateral_angle'
		
		#####################################################3
		# Publisher objects from Ros for each direction/rotation, using 
		# the topics specififed above. 
		######################################################
		self.right_thrust_pub = rospy.Publisher(self.right_thrust,Float32, queue_size=10)
		self.right_angle_pub = rospy.Publisher(self.right_angle,Float32, queue_size=10)
		
		self.left_thrust_pub = rospy.Publisher(self.left_thrust, Float32, queue_size=10)
		self.left_angle_pub = rospy.Publisher(self.left_angle,Float32, queue_size=10)
		
		self.lat_thrust_pub = rospy.Publisher(self.lat_thrust, Float32, queue_size=10)
		self.lat_angle_pub = rospy.Publisher(self.lat_angle,Float32, queue_size=10)
		
		
		#self.gps_sub = rospy('wamv/sensors/gps/gps/fix',NavSatFix,self.gps_cb, queue_size=10)
		
		rospy.sleep(0.5)
		#def gps_cb(self,data):
		#	self.lat = data.lat
		self.drive()
		
	def drive(self):
		#rospy.init_node('boat_driver', anonymous = True)
		
		#loop while ro is runing, essentially forever
		while not rospy.is_shutdown():
			#get time for sin function
			t = rospy.get_time()
					
			#Send thrust command of magnitude 1.0
			self.right_thrust_pub.publish(3.0)
			self.left_thrust_pub.publish(3.0)
				
			#calculate angle from sin and time
			ang = np.sin(-t)
					
			#adjust angle by ang
			self.lat_angle_pub.publish(ang)	
			#self.right_angle_pub.publish(ang)
			#self.left_angle_pub.publish(ang)
			
					
			#Give thrust to lateral thruster
			self.lat_thrust_pub.publish(2.0)
		
				
		
if __name__ == '__main__':
	driver = BoatDriver()
	
	
	
	
	

		
		
