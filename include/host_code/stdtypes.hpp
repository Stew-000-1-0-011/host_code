#pragma once


#include <cstddef>
#include <cstdint>
#include <atomic>

namespace host_code::stdtypes {
	using usize = std::size_t;
	using isize = std::ptrdiff_t;
	using u8 = std::uint8_t;
	using u16  = std::uint16_t;
	using u32 = std::uint32_t;
	using i32 = std::int32_t;
}

namespace host_code {
	using namespace stdtypes;
}