/**
*
*Copyright (c) 2019 Nagireddi Jagadesh Nischal
*
*Redistribution and use in source and binary forms, with or without modification, are permitted *provided that the following conditions are met:
*
*1. Redistributions of source code must retain the above copyright notice, this list of conditions and *the following disclaimer.
*
*2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions *and the following disclaimer in the documentation and/or other materials provided with the *distribution.
*
*3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse *or promote products derived from this software without specific prior written permission.
*
*THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR *IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND *FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR *CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL *DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, *DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER *IN *CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT *OF THE *USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/


/*
 * @copyright 2019
 * @copyright BSD 3-Clause
 * @file Navigation.cpp
 * @author Nagireddi Jagadesh Nischal
 * @date 11/17/2019
 * @brief cpp implementation file for navigation class which moves the robot
 */
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <iterator>
#include <algorithm>
#include "Navigation.h"

Navigation::Navigation() {

	//Initialization of the obstacle status value
	obsStatus = false;
	//Initialization of the proximity value
	proximity = 0.0;
}

void Navigation::initialPublisherTwist() {
  //Initializing the publisher to publish the twist messages
	publishVel = nh.advertise<geometry_msgs::Twist>(
      "/mobile_base/commands/velocity", 100);
  //Streaming the publisher status message
  ROS_INFO_STREAM("Publisher active");
}
/**
 * @brief Initialization of the Subscriber for the laser scan
 * @param None
 * @return void
 */
void Navigation::initialSubscriberScan() {
  subScan = nh.subscribe("scan",100,&Navigation::scanCallback,this);
  //Streaming the subscriber status message.
  ROS_INFO_STREAM("Subscriber Active");
}
/**
 * @brief Call back function for the laser scan
 * @param msgScan - The scan values from the laser
 * @return void
 */
void Navigation::scanCallback(const sensor_msgs::LaserScan::ConstPtr &msgScan) {
  obsStatus = false;
  proximity = *(msgScan->ranges.begin());
  // To get the nearest value of the obstacle by iterating through the values.
  for(auto i :msgScan->ranges) {
    if(i < proximity && !std::isnan(i)) {
      //Set the smallest value to the proximity.
      proximity = i;
    }
  }
  //Check if the obstacle not in the search space.
  if (std::isnan(proximity)) {
    ROS_INFO_STREAM("Path clear,no Obstacle ahead");
  } else {
    ROS_INFO_STREAM("The Minimum Distance to an Obstacle is : \t" << proximity);
  }
  // Check proximity to obstacle for the robot.
  if (proximity < msgScan->range_min + 0.8 && !std::isnan(proximity)) {
    obsStatus = true;
    ROS_WARN_STREAM("Object found");
    ROS_INFO_STREAM("Object found ahead. Making a turn to proceed");
  }
  //Function to move the robot
  moveRobot(obsStatus);

}
/**
 * @brief Function to move the robot according to the object status.
 * @param obsStatus bool to check object status
 * @return void
 */
void Navigation::moveRobot(bool obsStatus) {
  if (obsStatus) {
    //Angular movement of the robot.
    twistMsg.linear.x = 0;
    twistMsg.angular.z = 1.0;
  } else {
    //Move in the forward direction.
    twistMsg.linear.x = 0.5;
    twistMsg.angular.z = 0.0;
  }
  //publish the values to the robot to follow.
  publishVel.publish(twistMsg);
}




Navigation::~Navigation() {
}

