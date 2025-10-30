#pragma once

#include <atomic>
#include <stop_token>

namespace host_code::world_without_ros {
	void world_without_ros (
		const std::stop_token& stok
		, std::atomic<double> *const x
		, std::atomic<double> *const y
		, std::atomic<double> *const th
		, const double vxymax
		, const double vthmax
		, const double axymax
		, const double athmax
	);
}