////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


#include "RTIMULib.h"
#include <math.h>
#include <ros/ros.h>
#include <three_wheel_robot/robot_info.h>

ros::Publisher IMU_pub;

int main(int argc,char **argv)
{
    int sampleCount = 0;
    int sampleRate = 0;
    uint64_t rateTimer;
    uint64_t displayTimer;
    uint64_t now;
	float theta;
	float theta_last = 0.0;
	three_wheel_robot::robot_info robot_pose;
	

	ros::init(argc,argv,"IMU");
	ros::NodeHandle nh;
	IMU_pub = nh.advertise<three_wheel_robot::robot_info>("/IMU/robot_info",1);

    //  Using RTIMULib here allows it to use the .ini file generated by RTIMULibDemo.
    //  Or, you can create the .ini in some other directory by using:
    //      RTIMUSettings *settings = new RTIMUSettings("<directory path>", "RTIMULib");
    //  where <directory path> is the path to where the .ini file is to be loaded/saved

    RTIMUSettings *settings = new RTIMUSettings("/home/pi/pi_catkin_ws/src/imu_node/src","RTIMULib");

    RTIMU *imu = RTIMU::createIMU(settings);

    if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL)) {
        printf("No IMU found\n");
        exit(1);
    }

    //  This is an opportunity to manually override any settings before the call IMUInit

    //  set up IMU

    imu->IMUInit();

    //  this is a convenient place to change fusion parameters

    imu->setSlerpPower(0.02);
    imu->setGyroEnable(true);
    imu->setAccelEnable(false);
    imu->setCompassEnable(true);

    //  set up for rate timer

    rateTimer = displayTimer = RTMath::currentUSecsSinceEpoch();

    //  now just process data

    while (ros::ok()) {
        //  poll at the rate recommended by the IMU

        usleep(imu->IMUGetPollInterval() * 10);

        while (imu->IMURead()) {
            RTIMU_DATA imuData = imu->getIMUData();
            sampleCount++;

            now = RTMath::currentUSecsSinceEpoch();

            //  display 100 times per second used to b3 30000
            if ((now - displayTimer) > 30000) {
                //printf("Sample rate %d: %s\r", sampleRate, RTMath::displayDegrees("", imuData.fusionPose));
                //printf("Sample rate %d: %s\r", sampleRate, RTMath::displayDegrees("", imuData.compass.normalize());
				theta = atan2(imuData.compass.z(),imuData.compass.y())+3.14159;               
				//Note from Aaron - I modified to match limits on the robots max rotational velocity
				// The max rotational vel is 180d/s = .105 rad every 30th of a second
				if (theta-theta_last > .06){
					theta = theta_last + .06;
				}else if (theta-theta_last < -.06){
					theta = theta_last - .06;
				}
				theta_last = theta;
				printf("Magnetometer degree:%f \n fusion degree:%s  \n", theta*180/3.14, RTMath::displayDegrees("",imuData.fusionPose));
				printf("sampling rate: %llu \n", now-displayTimer);//float(1/(now-displayTimer)));
                fflush(stdout);
                displayTimer = now;
                robot_pose.theta = theta;
                IMU_pub.publish(robot_pose);
            }

            //  update rate every second

            //if ((now - rateTimer) > 100000) {
            //    sampleRate = sampleCount;
            //    sampleCount = 0;
            //    rateTimer = now;
            //}
        }
    }
	ros::spin();
}
