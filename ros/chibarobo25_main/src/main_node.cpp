#include <cmath>
#include <iostream>
#include <print>
#include <stop_token>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "../include/host_code/beautiful_world_without_ros.hpp"
#include "host_code/stdtypes.hpp"

using namespace host_code::stdtypes;

struct Node : rclcpp::Node {
	std::atomic<double> * x;
	std::atomic<double> * y;
	std::atomic<double> * th;
	std::atomic<double> * vx;
	std::atomic<double> * vy;
	std::atomic<double> * vth;
	std::atomic<u8> * buttons;
	double vxymax;
	double vthmax;
	bool is_auto{false};

	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;

	Node (
		std::atomic<double> * x
		, std::atomic<double> * y
		, std::atomic<double> * th
		, std::atomic<double> * vx
		, std::atomic<double> * vy
		, std::atomic<double> * vth
		, std::atomic<u8> * buttons
		, const double vxymax
		, const double vthmax
	)
	: rclcpp::Node{"main_node"}
	, x{x}
	, y{y}
	, th{th}
	, vx{vx}
	, vy{vy}
	, vth{vth}
	, buttons{buttons}
	, vxymax{vxymax}
	, vthmax{vthmax}
	, pose_sub{this->create_subscription<geometry_msgs::msg::PoseStamped> (
		"pose"
		, 1
		, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
			this->set_position(msg);
		}
	)}
	, joy_sub{this->create_subscription<sensor_msgs::msg::Joy> (
		"joy"
		, 1
		, [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
			this->joy_callback(msg);
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

	void joy_callback(const sensor_msgs::msg::Joy::SharedPtr& msg) {
		u8 state = 0;
		constexpr u8 emergncy_stop = 0b0001;
		constexpr u8 calibration = 0b0010;
		constexpr u8 shoot = 0b0100;
		constexpr u8 is_auto = 0b1000;

		// back -> 非常停止
		if(msg->buttons[6] == 1) {
			state |= emergncy_stop;
		}

		if(this->is_auto) {
			std::println("in auto.");
			// B -> 手動モードへ
			if(msg->buttons[1] == 1) {
				this->is_auto = false;
				state &= ~is_auto;
			}

			this->vx->store(0.);
			this->vy->store(0.);
			this->vth->store(0.);
		}
		else {
			std::println("in manual.");
			// A -> 自動モードへ
			if(msg->buttons[0] == 1) {
				this->is_auto = true;
				state |= is_auto;
			}

			// start -> キャリブレーション
			if(msg->buttons[7] == 1) {
				state |= calibration;
			}

			// Y -> 射出
			if(msg->buttons[3] == 1) {
				state |= shoot;
			}

			// 左スティック -> 前後左右
			this->vx->store(msg->axes[0] * -vxymax * 0.8);
			this->vy->store(msg->axes[1] * vxymax * 0.8);

			// 右スティック -> 左右回転
			this->vth->store(msg->axes[3] * vthmax * 0.8);
		}
		
		this->buttons->store(state);
		std::println("{}", int(state));
	}
};

int main(int argc, char * argv[]) {
	double vxymax, vthmax, axymax, athmax, x_init, y_init, th_init;
	std::cin >> vxymax >> vthmax >> axymax >> athmax >> x_init >> y_init >> th_init;

	std::stop_source ssrc{};
	
	std::atomic<double> x{x_init};
	std::atomic<double> y{y_init};
	std::atomic<double> th{th_init};
	std::atomic<double> man_vx{0};
	std::atomic<double> man_vy{0};
	std::atomic<double> man_vth{0};
	std::atomic<u8> buttons{0};

	std::jthread world{[&] {
		host_code::world_without_ros::world_without_ros (
			ssrc.get_token()
			, &x
			, &y
			, &th
			, &man_vx
			, &man_vy
			, &man_vth
			, &buttons
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
	rclcpp::spin(std::make_shared<Node>(&x, &y, &th, &man_vx, &man_vy, &man_vth, &buttons, vxymax, vthmax));
	std::println("main_node is end.");
}