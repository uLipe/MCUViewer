#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "EspProbeSession.hpp"

static uint32_t parse_u32(const char* s)
{
	return static_cast<uint32_t>(std::strtoul(s, nullptr, 0));
}

int main(int argc, char* argv[])
{
	const char* chip = argc > 1 ? argv[1] : "esp32c6";
	const uint32_t speed_khz = argc > 2 ? parse_u32(argv[2]) : 24000u;
	if (argc < 4)
	{
		std::fprintf(stderr, "usage: %s [chip] [speed_khz] <address_hex>\n", argv[0]);
		return 1;
	}

	const uint32_t address = parse_u32(argv[3]);

	esp_probe::EspProbeSession session;
	if (!session.connect("", chip, speed_khz))
	{
		std::fprintf(stderr, "connect failed: %s\n", session.lastError().c_str());
		return 1;
	}

	uint8_t buf[4]{};
	if (!session.readMemory(address, buf, 4))
	{
		std::fprintf(stderr, "read failed: %s\n", session.lastError().c_str());
		return 2;
	}

	const uint32_t raw = buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);
	std::printf("mem @ 0x%08" PRIx32 " = %08" PRIx32 "\n", address, raw);
	return 0;
}
