/**
 * MIT License

 * Copyright (c) 2018 Nithish Sanjeev Kumar

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**@file control.cpp
 *
 * @brief To publish message in a topic
 *
 * @author Nithish Sanjeev Kumar
 * @copyright 2018 , Nithish Sanjeev Kumar All rights reserved

 */

#include "ros/ros.h"
#include "control.hpp"
#include "depthData.hpp"

control::control() {

  vel = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",100);
  laser = nh.subscribe<sensor_msgs::LaserScan>("/scan", 100, &depthData::scanCallback, &depth);

  info.linear.x = 0.0;
  info.linear.y = 0.0;
  info.linear.z = 0.0;
  info.angular.x = 0.0;
  info.angular.y = 0.0;
  info.angular.z = 0.0;
  vel.publish(info);	
}

control::~control() {

  info.linear.x = 0.0;
  info.linear.y = 0.0;
  info.linear.z = 0.0;
  info.angular.x = 0.0;
  info.angular.y = 0.0;
  info.angular.z = 0.0;
  vel.publish(info);	
}

void control::command() {
  ros::Rate loop_rate(10);
  while(ros::ok()) {
  	if(depth.obstacleCheck() < .5) {
    	info.linear.x = 0;
    	info.angular.z = 1;
		}
		else {
      info.linear.x = 1;
      info.angular.z = 0;
    }
  vel.publish(info);
  loop_rate.sleep();
  }
}
