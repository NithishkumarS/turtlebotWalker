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

/**@file control.hpp
 *
 * @brief To use the depth data and move around the environment
 *
 * @author Nithish Sanjeev Kumar
 * @copyright 2018 , Nithish Sanjeev Kumar All rights reserved

 */
#pragma once
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "depthData.hpp"

/** @brief Class to implement the control of turtlebot
 *  @param nh NodeHandle for the node
 *  @param depth Class object of depthData
 *  @param vel Publisher object
 *  @param laser Subsriber object
 *  @param info  variable to send velocities
 *  @return bool
 */
class control {
 private:
  /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
ros::NodeHandle nh;
depthData depth;
ros::Publisher vel;
ros::Subscriber laser;
geometry_msgs::Twist info;

 public:
 /**@brief constructor
  * @param none
  * @return none
  */
control();

 /**@brief destructor
  * @param zStart
  * @return none
  */
~control();

 /**@brief function to control the turtlebot
  * @param none
  * @return none
  */
void command();
};
