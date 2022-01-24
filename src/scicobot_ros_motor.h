/***********************************************************************************
 *  @file       scicobot_ros_motor.h
 *  Project     scicobot_hardware
 *  @brief      *****
 *
 *  @author     Otávio Augusto Rocha da Cruz
 *  @bug 		 No known bugs.
 *  License     MIT
 *
 *  @section License
 *
 * Copyright (c) 2021 SciCoBot
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
**********************************************************************************/
#ifndef SCICOBOT_ROS_MOTOR_H
#define SCICOBOT_ROS_MOTOR_H

#include <ros_init/scicobot_ros_init.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>

#define SCICOBOT_ROS_MOTOR_DEBUG 0 // Debug mode
#if SCICOBOT_ROS_MOTOR_DEBUG == 1
	#include <serial_debug.h>
#endif

#define SCICOBOT_ROS_MOTOR_BEST_EFFORT 0

class ScicobotRosMotor
{
	public:
	
	void initRosMotorSubscriber(ScicobotRos* scicobotRos, void (*subscriberMotorCallback)(const void *));
	
	rclc_executor_t* get_rclc_executor();
		
	private:
	
	void initSubscriberMotor(ScicobotRos* scicobotRos, char* topicName);
	void initExecutorMotor(ScicobotRos* scicobotRos, int numberHandles);
	void addExecutorsMotor(void (*subscriberMotorCallback)(const void *));
	
	rcl_subscription_t subscriberMotor;
	geometry_msgs__msg__Twist msgMotor;
	rclc_executor_t executorMotor;

};
#endif //SCICOBOT_ROS_MOTOR_H
