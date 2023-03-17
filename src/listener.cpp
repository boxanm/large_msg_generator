//
// Created by mbo on 10/03/23.
//
// Copyright 2014 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>


static const rmw_qos_profile_t rcl_qos_profile_big_data =
	{
		RMW_QOS_POLICY_HISTORY_KEEP_LAST,
		1,
		RMW_QOS_POLICY_RELIABILITY_RELIABLE,
		RMW_QOS_POLICY_DURABILITY_VOLATILE,
		RMW_QOS_DEADLINE_DEFAULT,
		{10, 0},
		RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC,
		RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
		false
	};

class LargeMsgGeneratorListener : public rclcpp::Node
{
public:
	LargeMsgGeneratorListener()
		: Node("large_msg_generator_listener")
	{
		// Create a callback function for when messages are received.
		// Variations of this function also exist using, for example UniquePtr for zero-copy transport.
		setvbuf(stdout, NULL, _IONBF, BUFSIZ);
		auto callback =
			[this](sensor_msgs::msg::PointCloud2 ::ConstSharedPtr msg) -> void
			{
				RCLCPP_INFO(this->get_logger(), "I heard: new message at [%d.%d]", msg->header.stamp.sec, msg->header.stamp.nanosec);
			};
		// Create a subscription to the topic which can be matched with one or more compatible ROS
		// publishers.
		// Note that not all publishers on the same topic with the same type will be compatible:
		// they must have compatible Quality of Service policies.
		auto qos = rclcpp::QoS(rclcpp::KeepLast(1000), rcl_qos_profile_big_data);
		sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("large_msg", qos, callback);
	}

private:
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LargeMsgGeneratorListener>());
	rclcpp::shutdown();
	return 0;
}