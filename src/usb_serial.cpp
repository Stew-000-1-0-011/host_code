#include <expected>
#include <stop_token>
#include <utility>

#include <cerrno>
#include <csignal>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include "host_code/stdtypes.hpp"

#include "host_code/usb_serial.hpp"

namespace host_code::usb_serial::detail {
	void UsbSerial::close(int fd) noexcept {
		::close(fd);
	}

	auto UsbSerial::open(std::stop_token&& stok, const std::filesystem::path& device, const bool parity, const bool stop_2bit, const DataBitSize bit_size) noexcept -> std::expected<UsbSerial, UsbError> {
		int fd;
		struct termios tio;

		// デバイスを非ブロッキングで開く
		fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
		if (fd < 0) {
			return std::unexpected<UsbError>{"UsbError: unable to open."};
		}

		// 現在の設定取得
		if (tcgetattr(fd, &tio) < 0) {
			close(fd);
			return std::unexpected<UsbError>{"UsbError: unable to tcgetattr."};
		}

		// rawモード的に設定
		cfmakeraw(&tio);

		// 速度設定 (9600bps)
		cfsetispeed(&tio, B9600);
		cfsetospeed(&tio, B9600);

		if(parity) tio.c_cflag |= PARENB;
		else tio.c_cflag &= ~PARENB;
		if(stop_2bit) tio.c_cflag |= CSTOPB;
		else tio.c_cflag &= ~CSTOPB; 
		tio.c_cflag &= ~CSIZE;
		switch(bit_size) {
			case DataBitSize::Cs5:
			tio.c_cflag |= CS5;
			break;
			case DataBitSize::Cs6:
			tio.c_cflag |= CS6;
			break;
			case DataBitSize::Cs7:
			tio.c_cflag |= CS7;
			break;
			case DataBitSize::Cs8:
			tio.c_cflag |= CS8;
		}
		// ハードフロー制御無効 (必要なら変更)
		tio.c_cflag &= ~CRTSCTS;

		// 制御系 (ブロッキング動作のしきい値)
		tio.c_cc[VMIN]  = 0; // 非ブロッキング read に重要
		tio.c_cc[VTIME] = 0;

		// 設定を即時反映
		if (tcsetattr(fd, TCSANOW, &tio) < 0) {
			const auto err = errno;
			close(fd);
			return std::unexpected<UsbError>{UsbError{"UsbError: unable to tcsetattr.", err}};
		}

		return std::expected<UsbSerial, UsbError>{UsbSerial{fd, std::move(stok)}};
	}

	auto UsbSerial::write(const std::span<const std::byte>& bytes) const noexcept -> std::expected<usize, UsbError> {
		isize ret = ::write(this->fd, bytes.data(), bytes.size());
		if(ret == -1) {
			const auto err = errno;
			if(err != EAGAIN && err != EWOULDBLOCK) {
				return std::unexpected<UsbError>{UsbError{"UsbError: in write().", err}};
			}
			ret = 0;
		}
		return std::expected<usize, UsbError>{std::in_place, ret};
	}

	auto UsbSerial::read(const std::span<std::byte>& output) const noexcept -> std::expected<usize, UsbError> {
		isize ret = ::read(this->fd, output.data(), output.size());
		if(ret == -1) {
			const auto err = errno;
			if(err != EAGAIN && err != EWOULDBLOCK) {
				return std::unexpected<UsbError>{UsbError{"UsbError: in read().", err}};
			}
			ret = 0;
		}
		return std::expected<usize, UsbError>{std::in_place, ret};
	}
}
