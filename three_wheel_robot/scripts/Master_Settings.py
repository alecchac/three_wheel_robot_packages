#! /usr/bin/python

#---------Map Scaling Factor
#     SF = Pixels:Meters
#SF = 37.0 / 0.15
SF = 1


#---------Velocity Controller------------
max_linear_speed= .3 * SF #pixels/sec
max_angular_speed = 2 #radians/sec
Kc_linear = 1
Ti_linear = 15 #Ki=Kc/Ti
Kc_angular = .8 
Ti_angular = 150
distance_tolerance = .02 * SF #meters
angle_tolerance = 100 #Radians
min_vel = .005 *SF #m/s


#---------Wheel Speed Controller----------
saturation_omega = 90
Kc = 1.5
Ti = 1000
Kd = .15

#---------Robot Definitions---------------
#distance from center to wheel
d = 0.09 * SF
#radius of wheel
r = 0.05 * SF

