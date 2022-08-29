/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>

 * The TrajectorySetpoint message and the OFFBOARD mode in general are under an ongoing update.
 * Please refer to PR: https://github.com/PX4/PX4-Autopilot/pull/16739 for more info. 
 * As per PR: https://github.com/PX4/PX4-Autopilot/pull/17094, the format
 * of the TrajectorySetpoint message shall change.
 */

#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/sensor_data_computed.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class SensorFusion : public rclcpp::Node {
public:
	SensorFusion() : Node("sensor_fusion") {
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					timestamp_.store(msg->timestamp);
					//std::cout << "timestamp: " << msg->timestamp << std::endl;
				});

		vehicle_position_sub_ =
			this -> create_subscription<px4_msgs::msg::VehicleOdometry>("fmu/vehicle_odometry/out", 10,
				[this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
					vehicle_x = msg -> x;
					vehicle_y = msg -> y;
					vehicle_z = msg -> z;
					vehicle_xv = msg -> vx;
					vehicle_yv = msg -> vy;
					vehicle_zv = msg -> vz;
					gimbal_x = msg -> x;
					gimbal_y = msg -> y;
					gimbal_z = msg -> z + 0.5;
					//std::cout <<"x position:" << vehicle_x << "\t" << "y position:" << vehicle_y << "\t" << "z position:" << vehicle_z << std::endl;
					//std::cout <<"-------------------------------"<< std::endl;
					return;
				});

		target_position_sub_ = 
				this -> create_subscription<nav_msgs::msg::Odometry>("target/odom", 10,
				[this](const nav_msgs::msg::Odometry::SharedPtr msg) {
					target_x = msg -> pose.pose.position.x;
					target_y = msg -> pose.pose.position.y;
					target_z = msg -> pose.pose.position.z;
					//std::cout <<"target x position:" << target_x << "\t" << "target y position:" << target_y << "\t" << "target z position:" << target_z << std::endl;
					//std::cout <<"-------------------------------"<< std::endl;
					return;
				});

		publisher_ = this->create_publisher<px4_msgs::msg::SensorDataComputed>("target/sensor_combined", 10);
		auto timer_callback = [this]() -> void {
	
			auto message = px4_msgs::msg::SensorDataComputed ();
		
			message.vehicle_x = vehicle_x;
			message.vehicle_y = vehicle_y;
			message.vehicle_z = vehicle_z;
			message.vehicle_x_v = vehicle_xv;
			message.vehicle_y_v = vehicle_yv;
			message.vehicle_z_v = vehicle_zv;

			message.gimbal_x = gimbal_x;
			message.gimbal_y = gimbal_y;
			message.gimbal_z = gimbal_z;

			message.target_x = target_x;
			message.target_y = target_y;
			message.target_z = target_z;
			
			message.vehicle_x_d = target_x;
			message.vehicle_y_d = target_y + 5;
			message.vehicle_z_d = -2.500000;

			publisher_ -> publish(message);
			return;
		};
		timer_ = this -> create_wall_timer(100ms, timer_callback);	
	}
private:
	float vehicle_x, vehicle_xv;
	float vehicle_y, vehicle_yv;
	float vehicle_z, vehicle_zv;
	float target_x;
	float target_y;
	float target_z;
	float gimbal_x;
	float gimbal_y;
	float gimbal_z;
	int count = 0;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_position_sub_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_position_sub_;
	rclcpp::Publisher<px4_msgs::msg::SensorDataComputed>::SharedPtr publisher_;	
	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
};

int main(int argc, char* argv[]) {
	std::cout << "Starting sensor combined node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SensorFusion>());

	rclcpp::shutdown();
	return 0;
}
