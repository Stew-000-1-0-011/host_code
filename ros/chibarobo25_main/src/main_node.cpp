#include <cmath>
#include <iostream>
#include <print>
#include <stop_token>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "../include/host_code/beautiful_world_without_ros.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/utilities.hpp"

struct Node : rclcpp::Node {
	std::atomic<double> * x;
	std::atomic<double> * y;
	std::atomic<double> * th;

	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub;

	Node(std::atomic<double> * x, std::atomic<double> * y, std::atomic<double> * th)
	: rclcpp::Node{"main_node"}
	, x{x}
	, y{y}
	, th{th}
	, sub{this->create_subscription<geometry_msgs::msg::PoseStamped> (
		"pose"
		, 1
		, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
			this->set_position(msg);
		}
	)}
	{}

	void set_position(const geometry_msgs::msg::PoseStamped::SharedPtr& msg) {
		constexpr auto yaw_from_quaternion = [](const double (&q)[4]) -> double {
			constexpr auto x = 0;
			constexpr auto y = 1;
			constexpr auto z = 2;
			constexpr auto w = 3;
			return std::atan2(2 * (q[w] * q[z] - q[x] * q[y]), 1 - 2 * (q[y] * q[y] + q[z] * q[z]));
		};

		*this->x = msg->pose.position.x;
		*this->y = msg->pose.position.y;
		const auto& q = msg->pose.orientation;
		*this->th = yaw_from_quaternion({q.x, q.y, q.z, q.w});
	}
};

int main(int argc, char * argv[]) {
	double vxymax, vthmax, axymax, athmax;
	std::cin >> vxymax >> vthmax >> axymax >>athmax;

	std::stop_source ssrc{};
	
	std::atomic<double> x;
	std::atomic<double> y;
	std::atomic<double> th;

	std::jthread world{[&] {
		host_code::world_without_ros::world_without_ros (
			ssrc.get_token()
			, &x
			, &y
			, &th
			, vxymax
			, vthmax
			, axymax
			, athmax
		);
		std::println("end of the world.");
	}};

	std::println("Hi");
	
	rclcpp::init(argc, argv);
	rclcpp::on_shutdown([&] {
		std::println("on_shutdown!!!");
		ssrc.request_stop();
	});
	std::println("start to spin.");
	rclcpp::spin(std::make_shared<Node>(&x, &y, &th));
	std::println("main_node is end.");
}