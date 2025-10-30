#include <cstring>

#include <filesystem>
#include <iostream>
#include <print>
#include <stop_token>
#include <thread>

#include <spawn_coro/spawn_coro.hpp>
#include <spawn_coro/work_steal_executor.hpp>

#include "host_code/usb_serial.hpp"

using namespace std::chrono_literals;

using spawn_coro::spawn_coro::SpawnCoro;
using spawn_coro::spawn_coro::SuspendToSpawn;
using spawn_coro::work_steal_executor::Spawner;
using spawn_coro::work_steal_executor::WorkStealExecutor;
using namespace host_code::usb_serial;

template<class Ret_>
using Coro = SpawnCoro<Spawner, Ret_>;
using Suspend = SuspendToSpawn<Spawner>;

int main() {
	// std::println("input usb path.");
	// std::filesystem::path usb_path{};
	// std::cin >> usb_path;

	std::filesystem::path usb_path = "/dev/ttyUSB0";
	auto ssource = std::stop_source{};

	const auto usb_serial = UsbSerial::open(ssource.get_token(), usb_path, false, false, DataBitSize::Cs8);
	if(!usb_serial) {
		const auto& e = usb_serial.error();
		std::println("failed to open. {}, errno: {}", e.what(), std::strerror(e.err));
		return 0;
	}

	auto exec = WorkStealExecutor{3};

	constexpr auto& msg = "Hello USB!";

	// write
	exec.add([&, stok = ssource.get_token()]() -> Coro<void> {
		co_await Suspend{};  // 別スレッドに切り替える

		while(!stok.stop_requested()) {
			std::println(std::cerr, "in write_loop");
			std::string input{};
			std::cin >> input;
			if(input == "q") {
				ssource.request_stop();
				break;
			}
			else {
				const auto ret = usb_serial->write_all(std::span<const std::byte>{reinterpret_cast<const std::byte *>(msg), sizeof(msg)});
				if(!ret) {
					const auto& e = ret.error();
					std::println(std::cerr, "fail to write_all: {}, errno: {}", e.what(), std::strerror(e.err));
					break;
				}
			}

			std::this_thread::sleep_for(100ms);
		}

		std::println(std::cerr, "write end.");
	});

	// read
	exec.add([&, stok = ssource.get_token()]() -> Coro<void> {
		co_await Suspend{};  // 別スレッドに切り替える

		while(!stok.stop_requested()) {
			std::println(std::cerr, "in read_loop");
			std::byte buf[512];
			const auto ret = usb_serial->read_all(std::span{buf, sizeof(msg)});
			if(!ret) {
				const auto& e = ret.error();
				std::println(std::cerr, "fail to read_all: {}, errno: {}", e.what(), strerror(e.err));
				break;
			}
			else {
				std::println("OK! {}", reinterpret_cast<const char *>(buf));
			}

			// どう考えても`await Timer{100ms};`などとすべきだが、実装する余裕がないのでやむなしwait
			std::this_thread::sleep_for(100ms);
		}

		std::println(std::cerr, "read end.");
	});
}