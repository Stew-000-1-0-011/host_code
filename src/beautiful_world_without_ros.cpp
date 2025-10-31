#include <cmath>
#include <cstddef>
#include <cstring>

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <chrono>
#include <iostream>
#include <print>
#include <stop_token>
#include <thread>

// #include "cobs_bytestuff/cobs.hpp"
#include "spawn_coro/spawn_coro.hpp"
#include "spawn_coro/work_steal_executor.hpp"

#include "host_code/robot_task.hpp"
#include "host_code/usb_serial.hpp"

using namespace std::chrono_literals;

using spawn_coro::spawn_coro::SpawnCoro;
using spawn_coro::spawn_coro::SuspendToSpawn;
using spawn_coro::work_steal_executor::WorkStealExecutor;
using spawn_coro::work_steal_executor::Spawner;

using namespace host_code::stdtypes;
using host_code::robot_task::MoveTo;
using host_code::robot_task::Task;
using host_code::robot_task::TaskKind;
using host_code::usb_serial::UsbSerial;
using host_code::usb_serial::DataBitSize;

template<class Ret_>
using Coro = SpawnCoro<Spawner, Ret_>;

using Suspend = SuspendToSpawn<Spawner>;

using Clock = std::chrono::system_clock;

auto range_convert(const double x, const double vmax) -> u32 {
	return std::bit_cast<u32>(static_cast<u32>(std::clamp (
		static_cast<i32>((x / vmax) * 128.)
		, -128
		, 127
	)));
}

auto norm2(const double x, const double y) -> double {
	return x * x + y * y;
}

auto calc_vel(const double err, const double amax) -> double {
	return (std::signbit(err) ? -1. : 1.) * std::sqrt(std::abs(2 * amax * err));
}

template<usize n1_, usize n2_>
requires (n1_ < 253 && n2_ >= n1_ + 2)
auto cobs_encode(const std::span<const std::byte, n1_> from, const std::span<std::byte, n2_> to) -> u32 {
	u32 last_zero = 0;
	for(u32 i = 1; i <= n1_; ++i) {
		if(from[i - 1] == std::byte(0)) {
			to[last_zero] = std::byte(i - last_zero);
			last_zero = i;
		}
		else {
			to[i] = from[i - 1];
		}
	}
	to[last_zero] = std::byte(n1_ + 1 - last_zero);
	to[n1_ + 1] = std::byte(0);
	return n1_ + 2;
}

namespace host_code::world_without_ros {
	/// say hello to the world without ros!
	void world_without_ros (
		const std::stop_token& stok
		, std::atomic<double> *const x
		, std::atomic<double> *const y
		, std::atomic<double> *const th
		, const double vxymax
		, const double vthmax
		, const double axymax
		, const double athmax
	) {
		auto exec = WorkStealExecutor(2);
		std::atomic<double> vx{0.};
		std::atomic<double> vy{0.};
		std::atomic<double> vth{0.};
		std::atomic<bool> shoot{false};

		std::atomic<u32> done_count{2};

		auto usb_serial = UsbSerial::open(auto(stok), "/dev/ttyUSB0", false, false, DataBitSize::Cs8);
		if(!usb_serial) {
			const auto& e = usb_serial.error();
			std::println("cannot open usb serial. {}: {}", e.what(), std::strerror(e.err));
			return;
		}

		std::vector<Task> tasks {
			Task::moveto(2.10, 1.55, 0., 0.1, std::numbers::pi / 4., 0.)
			, Task::moveto(2.10, 2.50, 0., 0.1, std::numbers::pi / 4., 0.)
			, Task::moveto(0.35, 2.50, 0., 0.1, std::numbers::pi / 4., 0.)
			, Task::moveto(0.35, 3.70, 0., 0.1, std::numbers::pi / 4., 0.)
			, Task::moveto(2.10, 3.70, 0., 0.1, std::numbers::pi / 4., 0.)
			, Task::moveto(2.10, 3.70, 0., 0.1, std::numbers::pi / 4., 0.)
			, Task::moveto(2.10, 5.20, 0., 0.1, std::numbers::pi / 4., 0.)
			, Task::moveto(2.90, 5.20, 0., 0.1, std::numbers::pi / 4., 1.)
			, Task::shoot()
		};

		// USB read/write.
		exec.add([&, stok = auto(stok)]() -> Coro<void> {
			co_await Suspend{};

			while(!stok.stop_requested()) {
				const auto vx_ = vx.load();
				const auto vy_ = vy.load();
				const auto vth_ = vth.load();
				const auto shoot_ = shoot.load();

				const auto vx2 = range_convert(vx_, vxymax);
				const auto vy2 = range_convert(vy_, vxymax);
				const auto vth2 = range_convert(vth_, vthmax);
				// std::println("v is {} {} {}", vx_, vy_, vth_);
				static_assert(std::endian::native == std::endian::little);
				u32 msg1{};
				msg1 |= 0;
				msg1 |= vx2 << 8;
				msg1 |= vy2 << 16;
				msg1 |= vth2 << 24;
				// std::println("msg1: {:x}", msg1);

				u16 msg2{};
				msg2 |= 1;
				msg2 |= (shoot_ ? 0 : 1) << 8;

				// COBS
				std::byte buf[6]{};
				cobs_encode(std::span<const std::byte, 4>{std::bit_cast<std::array<std::byte, 4>>(msg1)}, std::span<std::byte, 6>{buf});
				// for(u32 i = 0; i < 6; ++i) {
				// 	std::print("{} ", int(buf[i]));
				// }
				// std::println();
				if(const auto ret = usb_serial->write_all(std::span<std::byte>{buf, 6}); !ret) {
					const auto e = ret.error();
					std::println(std::cerr, "at write msg1: {}, errno: {}", e.what(), std::strerror(e.err));
				}
				// if(const auto ret = usb_serial->read_all(std::span<std::byte>{buf, 6}); !ret) {
				// 	const auto e = ret.error();
				// 	std::println(std::cerr, "at read msg1: {}, errno: {}", e.what(), std::strerror(e.err));
				// }
				// for(int i = 0; i < 6; ++i) {
				// 	std::print("{}   ", int(buf[i]));
				// }
				// std::println();

				cobs_encode(std::span<const std::byte, 2>{std::bit_cast<std::array<std::byte, 2>>(msg2)}, std::span<std::byte, 6>{buf});
				// for(u32 i = 0; i < 6; ++i) {
				// 	std::print("{} ", int(buf[i]));
				// }
				// std::println();
				if(const auto ret = usb_serial->write_all(std::span<std::byte>{buf, 4}); !ret) {
					const auto e = ret.error();
					std::println(std::cerr, "at write msg2: {}, errno: {}", e.what(), std::strerror(e.err));
				}
				// if(const auto ret = usb_serial->read_all(std::span<std::byte>{buf, 4}); !ret) {
				// 	const auto e = ret.error();
				// 	std::println(std::cerr, "at read msg2: {}, errno: {}", e.what(), std::strerror(e.err));
				// }
				// for(int i = 0; i < 4; ++i) {
				// 	std::print("{} ", int(buf[i]));
				// }
				// std::println();

				std::this_thread::sleep_for(1ms);
			}

			done_count.fetch_sub(1);
			done_count.notify_all();
		});

		// main
		exec.add([&, stok = auto(stok)]() -> Coro<void> {
			co_await Suspend{};

			for(u32 i = 0; i < tasks.size() && !stok.stop_requested(); ++i) {
				if(const auto task = tasks[i]; task.v.index() == std::to_underlying(TaskKind::MoveTo)) {
					const auto [gx, gy, gth, xy_thr, th_thr, t] = std::get<MoveTo>(task.v);
					const auto cx = x->load();
					const auto cy = y->load();
					const auto cth = th->load();
					auto last_far_time = Clock::now();


					while(!stok.stop_requested()) {
						const auto now = Clock::now();
						if(norm2(cx - gx, cy - gy) > xy_thr * xy_thr || norm2(cth - gth, 0.) > th_thr * th_thr) {
							last_far_time = now;
						}

						if(last_far_time - now >= std::chrono::duration<double>(t)) {
							break;
						}
						else {
							const auto vx_ = calc_vel(gx - cx, axymax);
							const auto vy_ = calc_vel(gy - cy, axymax);
							const auto vth_ = calc_vel(gth - cth, athmax);
							// std::println("sender v is {} {} {}.", vx_, vy_, vth_);

							vx.store(vx_);
							vy.store(vy_);
							vth.store(vth_);
						}
					}
				}
				else {
					shoot.store(true);
					break;
				}
			
				std::this_thread::sleep_for(1ms);
			}

			done_count.fetch_sub(1);
			done_count.notify_all();
		});

		auto old = done_count.load();
		while(old != 0) {
			done_count.wait(old);
			old = done_count.load();
		}
	}
}