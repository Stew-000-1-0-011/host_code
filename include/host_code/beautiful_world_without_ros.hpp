#pragma once

#include <atomic>
#include <stop_token>

#include "stdtypes.hpp"

namespace host_code::world_without_ros {
	void world_without_ros (
		const std::stop_token& stok
		, std::atomic<double> *const x
		, std::atomic<double> *const y
		, std::atomic<double> *const th
		, std::atomic<double> *const man_vx
		, std::atomic<double> *const man_vy
		, std::atomic<double> *const man_vth
		, std::atomic<u8> *const buttons
		, const double vxymax
		, const double vthmax
		, const double axymax
		, const double athmax
	);
}