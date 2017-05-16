/*
 * testgimbal_alone.cpp
 *
 *  Created on: May 14, 2017
 *      Author: cj
 */
#include <ros/ros.h>
#include <stdio.h>
#include <stdint.h>
#include <cstdlib>
#include <stdlib.h>

#include <dji_sdk/dji_drone.h>

int main(int argc,char **argv)
{
	ros::init(argc,argv,"test_gimbal_alone");
	ros::NodeHandle nh;

	DJIDrone *drone = new DJIDrone(nh);
	drone->request_sdk_permission_control();

    while(ros::ok())
    {
        drone->gimbal_angle_control(0, 0, 1800, 20);
        sleep(2);
        drone->gimbal_angle_control(0, 0, -1800, 20);
        sleep(2);
        drone->gimbal_angle_control(300, 0, 0, 20);
        sleep(2);
        drone->gimbal_angle_control(-300, 0, 0, 20);
        sleep(2);
        drone->gimbal_angle_control(0, 300, 0, 20);
        sleep(2);
        drone->gimbal_angle_control(0, -300, 0, 20);
        sleep(2);
        drone->gimbal_speed_control(100, 0, 0);
        sleep(2);
        drone->gimbal_speed_control(-100, 0, 0);
        sleep(2);
        drone->gimbal_speed_control(0, 0, 200);
        sleep(2);
        drone->gimbal_speed_control(0, 0, -200);
        sleep(2);
        drone->gimbal_speed_control(0, 200, 0);
        sleep(2);
        drone->gimbal_speed_control(0, -200, 0);
        sleep(2);
        drone->gimbal_angle_control(0, 0, 0, 20);

        ros::spinOnce();
    }
}
