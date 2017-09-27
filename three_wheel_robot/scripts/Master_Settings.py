#! /usr/bin/python

#---------Map Scaling Factor
#     SF = Pixels:Meters
#SF = 37.0 / 0.15
SF = 1


#---------Velocity Controller------------
max_linear_speed= .3 * SF #pixels/sec
max_angular_speed = 3 #radians/sec
Kc_linear = .75
Ti_linear = 10 #Ki=Kc/Ti
Kc_angular = .0015 
Ti_angular = 10
distance_tolerance = .02 * SF #meters
angle_tolerance = .5 #Radians
min_vel = .03 *SF #m/s


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

