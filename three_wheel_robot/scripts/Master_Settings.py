#! /usr/bin/python

#---------Map Scaling Factor
#     SF = Pixels:Meters
#SF = 37.0 / 0.15
SF = 1


#---------Velocity Controller------------
max_linear_speed= .4 * SF #pixels/sec
max_angular_speed = 60000000 #radians/sec
Kc_linear = 1
Ti_linear = 15 #Ki=Kc/Ti
Kc_angular = 1 
Ti_angular = 15000000000
Kd_angular=0
distance_tolerance = .05 * SF #meters
angle_tolerance = 100 #Radians
min_vel = .005 *SF #m/s
max_acceleration = .6


#---------Wheel Speed Controller----------
saturation_omega = 90
Kc = 1.5
Ti = 1000
Kd = .15

#---------Robot Definitions---------------
#distance from center to wheel
d = 0.07 * SF
#radius of wheel
r = 0.05 * SF

