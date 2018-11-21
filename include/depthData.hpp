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

/**@file depthData.hpp
 *
 * @brief To get the depth from sensor tppic
 *
 * @author Nithish Sanjeev Kumar
 * @copyright 2018 , Nithish Sanjeev Kumar All rights reserved

 */
#pragma once
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

/** @brief Class to collect data from the laser
 *  @param obstacle If near a obstacle its set true
 *  @return bool
 */
class depthData {
 private:
bool obstacle;

 public:
 /**@brief constructor
   * @param none
   * @return none
   */
depthData();

 /**@brief destructor
   * @param zStart
   * @return none
   */
~depthData();

 /**@brief callback function
   * @param temp sensor_msgs/LaserScan type
   * @return none
   */
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& temp);

  /**@brief returns the value of obstacle
   * @param none
   * @return obstacle bool value
   */
bool obstacleCheck();
};
