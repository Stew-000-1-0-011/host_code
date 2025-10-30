#pragma once

#include <variant>

#include "stdtypes.hpp"

namespace host_code::robot_task {
	enum class TaskKind : u8 {
		MoveTo,
		Shoot,
	};

	struct MoveTo final {
		double x;
		double y;
		double theta;
		double xy_thr;
		double th_thr;
		double t;
	};

	struct Shoot final {};

	struct Task final {
		std::variant<MoveTo, Shoot> v;

		static auto moveto(const double x, const double y, const double th, double xy_thr, double th_thr, double t) -> Task {
			return Task{MoveTo{x, y, th, xy_thr, th_thr, t}};
		}

		static auto shoot() -> Task {
			return Task{Shoot{}};
		}
	};
}