#pragma once

#include <cerrno>
#include <cstddef>

#include <exception>
#include <expected>
#include <filesystem>
#include <span>
#include <stdexcept>
#include <stop_token>

#include <spawn_coro/spawn_coro.hpp>
#include <spawn_coro/work_steal_executor.hpp>

#include "stdtypes.hpp"

namespace host_code::usb_serial::detail {
	struct UsbError final : std::runtime_error {
		int err{0};
		UsbError(const char * msg, int err = 0)
		: std::runtime_error{msg}
		, err{err}
		{
			if(err == 0) {
				this->err = errno;
			}
		}

		UsbError(const std::exception& e, int err = 0)
		: std::runtime_error{e.what()}
		, err{err}
		{
			if(err == 0) {
				this->err = errno;
			}
		}
	};

	template<class Ret_>
	using Coro = spawn_coro::spawn_coro::SpawnCoro<spawn_coro::work_steal_executor::Spawner, Ret_>;

	enum class DataBitSize : unsigned char {
		Cs5,
		Cs6,
		Cs7,
		Cs8,
	};

	struct UsbSerial final {
		int fd{-1};
		std::stop_token stok;

		UsbSerial() = delete;
		UsbSerial(const UsbSerial&) = delete;
		auto operator=(const UsbSerial&) -> UsbSerial& = delete;
		UsbSerial(UsbSerial&& other) noexcept
		: fd{other.fd}
		, stok{std::move(other.stok)} {
			other.fd = -1;
		}

		auto operator=(UsbSerial&& other) -> UsbSerial& {
			if(this->fd >= 0) UsbSerial::close(this->fd);
			this->fd = other.fd;
			this->stok = std::move(other.stok);
			other.fd = -1;
			return *this;
		}

		~UsbSerial() noexcept {
			if(this->fd >= 0) UsbSerial::close(this->fd);
		}

		private:
		UsbSerial(int fd, std::stop_token&& stok) noexcept
		: fd{fd}
		, stok{std::move(stok)}
		{}

		static void close(int fd) noexcept;
		
		public:
		static auto open(std::stop_token&& stok, const std::filesystem::path& device, const bool parity, const bool stop_2bit, const DataBitSize bit_size) noexcept -> std::expected<UsbSerial, UsbError>;
		auto write(const std::span<const std::byte>& bytes) const noexcept -> std::expected<usize, UsbError>;
		auto read(const std::span<std::byte>& output) const noexcept -> std::expected<usize, UsbError>;

		auto write_all(std::span<const std::byte> bytes) const noexcept -> std::expected<void, UsbError> {
			do {
				const auto ret = this->write(bytes);
				if(!ret) return ret.transform([](auto&&){});
				else {
					const auto n = bytes.size();
					bytes = bytes.last(n - *ret);
				}
			} while(!bytes.empty() && !this->stok.stop_requested());
			return std::expected<void, UsbError>{};
		}

		auto read_all(std::span<std::byte> output) const noexcept -> std::expected<void, UsbError> {
			do {
				const auto ret = this->read(output);
				if(!ret) return ret.transform([](auto&&){});
				else {
					const auto n = output.size();
					output = output.last(n - *ret);
				}
			} while(!output.empty() && !this->stok.stop_requested());
			return std::expected<void, UsbError>{};
		}
	};
}

namespace host_code::usb_serial {
	using detail::DataBitSize;
	using detail::UsbError;
	using detail::UsbSerial;
}