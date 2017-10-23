#!/usr/bin/env python
import time
import math
import rospy
from three_wheel_robot.msg import waypoints
from geometry_msgs.msg import PoseStamped
from three_wheel_robot.msg import robot_info
from aruco_node.msg import measurement
from Master_Settings import max_linear_speed,max_angular_speed,Kc_linear,Ti_linear,Kc_angular,Ti_angular,distance_tolerance,angle_tolerance,SF,min_vel,Kd_angular,max_acceleration

def main():
	#initialize node
	rospy.init_node('Controller',anonymous=True)
	refresh_rate = 30.0
	rate = rospy.Rate(5)
	#initialize controller
	bobControl=Velocity_Controller_PI(max_linear_speed,max_angular_speed,Kc_linear,Ti_linear,Kc_angular,Ti_angular,Kd_angular,max_acceleration)
	#initialize listener classes
	#bobWay=waypoint_listener()
	bobWaySingle = pose_listener()
	bobInfo=robot_info_listener()
	#init subscribers
	#rospy.Subscriber('goal_pos',waypoints,bobWay.callback)
	rospy.Subscriber('goal_pose',PoseStamped,bobWaySingle.callback)
	rospy.Subscriber('Pose_hat',robot_info,bobInfo.callback)
	#init publishers and publish message
	pub=rospy.Publisher('cmd_vel',robot_info,queue_size=1)
	bobPubInfo=robot_info()
	pi_pose = measurment_listener()
	rospy.Subscriber('/aruco/robot_pose',measurement,pi_pose.callback)
	#init moving average
	SMA = moving_average(refresh_rate)

	while (not rospy.is_shutdown()) :
		#--------------------- Single ----------------------------
		#With imu angles
		if bobInfo.theta !=0:
			SMA.updateTheta(bobInfo.theta)
			SMA.updateAverage()
		if bobInfo.x != 0 and SMA.count>=SMA.width:
			follow_angle = math.atan2(bobInfo.y,bobInfo.x)+3.85
			print "Current angle: "+str(SMA.getTheta()*(180/math.pi)) + "   "
			print "GOAL angle: "+str(follow_angle*(180/math.pi)) + "   "
			print "ERROR:"+str((SMA.getTheta()-follow_angle)*(180/math.pi)) + "   "
			#print "x: "+str(bobInfo.x) + "   "
			#print "y: "+str(bobInfo.y)+ "   "
			#updates the current goal pose and the current pose of the robot for the controller class to use
			bobControl.update_current_positions(bobWaySingle.x,bobWaySingle.y,follow_angle,bobInfo.x,bobInfo.y,bobInfo.theta)
			#calculates the velocities that the robot needs to go (need to specify minimum velocity in the function)
			vels=bobControl.update_velocities(min_vel)
			if getDistance(bobWaySingle.x,bobWaySingle.y,bobInfo.x,bobInfo.y)>distance_tolerance:
				#publish velocities to topic cmd_vel
				bobPubInfo.v_x = vels[0]
				bobPubInfo.v_y = vels[1]
				bobPubInfo.omega = vels[2]
				pub.publish(bobPubInfo)
			else:
				#resets Integrator sums
				bobControl.reset_Iterms()
				#publish velocities to topic cmd_vel
				bobPubInfo.v_x = 0
				bobPubInfo.v_y = 0
				bobPubInfo.omega = vels[2]
				pub.publish(bobPubInfo)
			rate.sleep()

		''' no imu
		if bobInfo.x != 0:
			if getDistance(bobWaySingle.x,bobWaySingle.y,bobInfo.x,bobInfo.y)>distance_tolerance:
				follow_angle = 0
				#updates the current goal pose and the current pose of the robot for the controller class to use
				bobControl.update_current_positions(bobWaySingle.x,bobWaySingle.y,follow_angle,bobInfo.x,bobInfo.y,-pi_pose.D2C)
				#calculates the velocities that the robot needs to go (need to specify minimum velocity in the function)
				vels=bobControl.update_velocities(min_vel)
				#publish velocities to topic cmd_vel
				bobPubInfo.v_x = vels[0]
				bobPubInfo.v_y = vels[1]
				if pi_pose.isValid:
					bobPubInfo.omega = vels[2]
				else:
					bobPubInfo.omega = 1
				print bobPubInfo.omega
				pub.publish(bobPubInfo)
			
			else:
				#resets Integrator sums
				bobControl.reset_Iterms()
				bobPubInfo.v_x = 0
				bobPubInfo.v_y = 0
				if pi_pose.isValid:
					bobPubInfo.omega = 0
				else:
					bobPubInfo.omega = 1
				pub.publish(bobPubInfo)
			'''


		#------------------------------------------- Array of points-------------------
		#checks if waypoint message is not empty
		"""
		if len(bobWay.x)>0:
			#loops through all waypoints
			for i in range(len(bobWay.x)):
				follow_angle = 0
				print i
				#resets Integrator sums
				bobControl.reset_Iterms()
				#checks if robot within the distance and angle tolerances
				while getDistance(bobWay.x[i],bobWay.y[i],bobInfo.x,bobInfo.y)>distance_tolerance:
					#updates the current goal pose and the current pose of the robot for the controller class to use
					bobControl.update_current_positions(bobWay.x[i],bobWay.y[i],follow_angle,bobInfo.x,bobInfo.y,-pi_pose.D2C)
					#calculates the velocities that the robot needs to go (need to specify minimum velocity in the function)
					vels=bobControl.update_velocities(bobWay.min_velocity[i])
					#publish velocities to topic cmd_vel
					bobPubInfo.v_x = vels[0]
					bobPubInfo.v_y = vels[1]
					if pi_pose.isValid:
						bobPubInfo.omega = vels[2]
					else:
						bobPubInfo.omega = 1.5
					pub.publish(bobPubInfo)
			#once done set velocities to zero and publish velocities
			bobPubInfo.v_x=0
			bobPubInfo.v_y=0
			bobPubInfo.omega=0				
			pub.publish(bobPubInfo)
			print 'done'
			break
			"""
	
	rospy.spin()

class moving_average(object):
	'''
	calculates the simple moving average with a width (counts) 
	'''
	def __init__(self,width):
		self.count = 0
		self.width = float(width)
		self.theta_avg = 0.0
		self.theta_current = 0.0
		self.record = []
	
	def getTheta(self):
		if self.count>=self.width:
			return self.theta_current
		else:
			return 0.0

	def updateTheta(self,upTheta):
		self.theta = upTheta

	def updateAverage(self):
		#builds the array to be used
		if(self.count<self.width):
			self.record.append(self.theta)
			self.count += 1 
		elif self.count >= self.width:
			for i in range(0,len(self.record)-1):
				self.record[i] = self.record[i+1]
			self.record[len(self.record)-1] = self.theta
			sum = 0
			for k in self.record:
				sum += k
			self.theta_current = sum / self.width
			self.count +=1




def getDistance(destX,destY,curX,curY):
	return math.sqrt((destX-curX)**2+(destY-curY)**2)

class pose_listener(object):
	""" waypoint listener"""
	def __init__(self):
		self.x = .5
		self.y = 0

	def callback(self,data):
		self.x = data.pose.position.x
		self.y = data.pose.position.y

class measurment_listener(object):
	""" measure listener"""
	def __init__(self):
		self.last_time = time.time()
		self.x = 0
		self.y = 0
		self.theta = 0
		self.cov_x = 0
		self.cov_y = 0
		self.cov_theta = 0
		self.D2C = 0
		self.markernum = 0
		self.isValid = False
	def callback(self,info):
		self.last_time = time.time()
		self.x = info.x
		self.y = info.y
		self.theta = info.theta
		self.cov_x = info.cov_x
		self.cov_y = info.cov_y
		self.cov_theta = info.cov_theta
		self.D2C = info.D2C
		self.markernum = info.markernum
		self.isValid = info.isValid

class waypoint_listener(object):
	""" waypoint listener"""
	def __init__(self):
		self.x=()
		self.y=()
		self.theta=()
		self.min_velocity=()
		

	def callback(self,data):
		self.x=data.x
		self.y=data.y
		self.theta=data.theta
		self.min_velocity=data.min_velocity

class robot_info_listener(object):
	""" robot info listener"""
	def __init__(self):
		self.x=0.0
		self.y=0.0
		self.theta=0.0
		self.v_x=0.0
		self.v_y=0.0
		self.omega=0.0
		

	def callback(self,data):
		self.x=data.x
		self.y=data.y
		self.theta=data.theta
		self.v_x=data.v_x
		self.v_y=data.v_y
		self.omega=data.omega


class Velocity_Controller_PI(object):
	"""PI Controller
	
	inputs:Linear saturation velocity, angular saturation velocity(rad/s), 
	Kc linear,Ti Linear,Kc Angular, Ti_angular
	outputs:v_x,v_y,omega(rad/s)
	
	**note: angular velocities are in radians/sec
	        Ki=Kc/Ti
	
	Dependencies:time
	"""
	
	def __init__(self,saturation_linear,saturation_angular,Kc_linear,Ti_linear,Kc_angular,Ti_angular,Kd_angular,max_acceleration):
		self.saturation_l=float(saturation_linear)
		self.saturation_a=float(saturation_angular)
		
		#if velocity input is saturated
		self.satL=False
		self.satT=False
		
		self.Ki_l=float(Kc_linear)/float(Ti_linear)
		self.Kc_l=float(Kc_linear)
		self.Ki_a=float(Kc_angular)/float(Ti_angular)
		self.Kc_a=float(Kc_angular)
		self.Ti_linear=float(Ti_linear)
		self.Ti_angular=float(Ti_angular)
		self.Kd_a=float(Kd_angular)
		self.max_accel = float(max_acceleration)
		
		self.setX=0.0
		self.setY=0.0
		self.setTheta=0.0
		
		self.currentX=0.0
		self.currentY=0.0
		self.currentTheta=0.0
		
		self.errorX=0.0
		self.errorY=0.0
		self.errorTheta=0.0
		self.last_error_theta=0.0
		
		#integrator sums
		self.IX=0.0
		self.IY=0.0
		self.ITheta=0.0

		#derivative contro
		self.DT = 0

		self.last_v_x = 0
		self.last_v_y = 0
		
		self.last_time = time.time()
		
	def update_velocities(self,minVel):
		#update errors
		self.errorX=self.setX-self.currentX
		self.errorY=self.setY-self.currentY
		self.errorTheta=self.setTheta-self.currentTheta

		#Proportional Contribution
		v_xP=self.errorX*self.Kc_l
		v_yP=self.errorY*self.Kc_l
		v_thetaP=self.errorTheta*self.Kc_a

		#calculate time from last function run
		delta_t=time.time()-self.last_time
		#resets delta_t if function has not been run for a while
		if delta_t>2:
			delta_t=.0001
		
		#Integral Contribution 
		#Sums up the error term with the delta t to remove steady state error
		#incorporates windup protection when motor is saturated
		if not self.satL:
			self.IX+=self.Ki_l*self.errorX*delta_t
			self.IY+=self.Ki_l*self.errorY*delta_t
		if not self.satT:
			self.ITheta+=self.Ki_a*self.errorTheta*delta_t
		
		#derivative Term
		self.DT=self.Kd_a*((self.errorTheta-self.last_error_theta)/delta_t)

		#set last error
		self.last_error_theta=self.errorTheta

		#Set last time
		self.last_time=time.time()
	
		#Add both contributions and multiply by SF
		v_x=(v_xP+self.IX)*SF
		v_y=(v_yP+self.IY)*SF
		v_theta=v_thetaP+self.ITheta+self.DT
		
		#motor saturtaion (sets max speed)
		mag=math.sqrt((v_x**2)+(v_y**2))
		multiplierMax=self.saturation_l/mag 
		#x saturation
		if mag>self.saturation_l:
			#if motor is saturated, apply windup protection and stops integrating
			self.satL=True
			v_x*=multiplierMax
			v_y*=multiplierMax
		else:
			#if saturation does not occur, there is no effect
			self.satL=False
		#theta saturation
		if v_theta>self.saturation_a:
			self.satT=True
			v_theta=self.saturation_a
		elif v_theta<-self.saturation_a:
			self.satT=True
			v_theta=-self.saturation_a
		else:
			self.satT=False
		
		#controls minimum speed: if velocities are too low, increase to the min velocity
		mag=math.sqrt((v_x**2)+(v_y**2))
		multiplier=minVel/mag
		if mag<minVel:
			v_x*=multiplier
			v_y*=multiplier

		#max acceleration control
		if (v_x-self.last_v_x)/delta_t > self.max_accel:
			v_x = self.last_v_x + self.max_accel * delta_t
		elif (v_x-self.last_v_x)/delta_t < self.max_accel:
			v_x = self.last_v_x - self.max_accel * delta_t
		if (v_y-self.last_v_y)/delta_t > self.max_accel:
			v_y = self.last_v_y + self.max_accel * delta_t
		elif (v_y-self.last_v_y)/delta_t < self.max_accel:
			v_y = self.last_v_y - self.max_accel * delta_t

		self.last_v_x = v_x
		self.last_v_y = v_y

		#return calculated velocities
		return [v_x,v_y,v_theta]
		
	
	def update_current_positions(self,setX,setY,setTheta,currentX,currentY,currentTheta):
		self.currentX=currentX
		self.currentY=currentY
		self.currentTheta=currentTheta
		self.setX=setX
		self.setY=setY
		self.setTheta=setTheta

		
	def reset_Iterms(self):
		self.IX=0.0
		self.IY=0.0
		self.ITheta=0.0
	
	def update_k_terms(self,Kc_linear,Ti_linear,Kc_angular,Ti_angular):
		self.Kc_l=Kc_linear
		self.Ti_linear=Ti_linear
		self.Kc_a=Kc_angular
		self.Ti_angular=Ti_angular

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
