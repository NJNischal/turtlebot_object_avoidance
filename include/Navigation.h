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
 * @file Navigation.h
 * @author Nagireddi Jagadesh Nischal
 * @date 11/17/2019
 * @brief header file for navigation class of the robot
 */

#ifndef INCLUDE_NAVIGATION_H_
#define INCLUDE_NAVIGATION_H_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

class Navigation {
private:

	// ROS node handle
	ros::NodeHandle nh;
	// ROS subscriber for scan
	ros::Subscriber subScan;
	//ROS publisher to the twist function
	ros::Publisher publishVel;
	// Boolean operator for the obstacle check
	bool obsStatus;
	// Float value with the distance measurement of obstacle
	float proximity;
	// twist message
	geometry_msgs::Twist twistMsg;

public:

	/**
	 * @brief Constructor for the Navigation class.
	 */
	Navigation();
	/**
	 * @brief Destructor for the Navigation class.
	 */
	~Navigation();

	/**
	 * @brief Initialization for the publisher for twist message
	 * @param None
	 * @return void
	 */
	void initialPublisherTwist();

	/**
	 * @brief Initialization for the scan subscriber
	 * @param None
	 * @return void
	 */
	void initialSubscriberScan();

	/**
	 * @brief check for clear path and move robot
	 * @param obsStatus Status of obstacle as a bool
	 * @return void
	 */
	void moveRobot(bool obsStatus);

	/**
	 * @brief Callback for the laser scan of the robot
	 * @param msgScan Scan pointer from the sensor message
	 * @return void
	 */
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msgScan);

};

#endif /* INCLUDE_NAVIGATION_H_ */
