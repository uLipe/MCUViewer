#ifndef ESP_RISCV_REGS_HPP
#define ESP_RISCV_REGS_HPP

#include <cstdint>
#include <string_view>

namespace esp_probe
{

constexpr uint8_t kIrBypass = 0x1f;
constexpr uint8_t kIrIdcode = 0x01;
constexpr uint8_t kIrDtmControl = 0x10;
constexpr uint8_t kIrDbus = 0x11;

constexpr uint32_t kDmiOpNop = 0;
constexpr uint32_t kDmiOpRead = 1;
constexpr uint32_t kDmiOpWrite = 2;

constexpr uint32_t kDmiStatusSuccess = 0;
constexpr uint32_t kDmiStatusFailed = 2;
constexpr uint32_t kDmiStatusBusy = 3;

constexpr uint32_t kDmDmControl = 0x10;
constexpr uint32_t kDmDmStatus = 0x11;
constexpr uint32_t kDmSbCs = 0x38;
constexpr uint32_t kDmSbAddress0 = 0x39;
constexpr uint32_t kDmSbData0 = 0x3c;

constexpr uint32_t kDmControlDmActive = 1u;
constexpr uint32_t kDmControlHaltReq = 0x80000000u;
constexpr uint32_t kDmControlResumeReq = 0x40000000u;
constexpr uint32_t kDmControlHartSelLo = 0x3ff0000u;
constexpr uint32_t kDmControlHartSelHi = 0xffc0u;
constexpr uint32_t kDmControlHaSel = 0x4000000u;

constexpr uint32_t kDmSbCsSbAccess = 0xe0000u;
constexpr uint32_t kDmSbCsSbReadOnAddr = 1u << 20;
constexpr uint32_t kDmSbCsSbSingleRead = 1u << 20;

constexpr uint32_t kDmSbCsSbVersion = 0xe0000000u;
constexpr uint32_t kDmSbCsSbVersion10 = 1u;

constexpr uint32_t kDtmDtmcsVersion = 0xfu;
constexpr uint32_t kDtmDtmcsAbits = 0x3f0u;
constexpr uint32_t kDtmDtmcsIdle = 0x7000u;
constexpr uint32_t kDtmDtmcsVersion10 = 1u;

constexpr unsigned kDmiOpOffset = 0;
constexpr unsigned kDmiDataOffset = 2;
constexpr unsigned kDmiAddressOffset = 34;

constexpr uint32_t kDtmDmiOpLength = 2;
constexpr uint32_t kDtmDmiDataLength = 32;

struct EspChipProfile
{
	const char* name;
	uint32_t idcode;
	uint8_t tap_count;
	uint8_t ir_len;
	uint8_t cpu_tap_index;
	bool is_riscv;
};

inline const EspChipProfile* find_chip_profile(const char* name)
{
	static const EspChipProfile profiles[] = {
		{"esp32c6", 0x0000dc25, 1, 5, 0, true},
		{"esp32p4", 0x00012c25, 2, 5, 1, true},
		{"esp32c3", 0x00005c25, 1, 5, 0, true},
		{"esp32h2", 0x0000c825, 1, 5, 0, true},
		{"esp32s3", 0x120034e5, 2, 5, 0, false},
	};
	for (const auto& p : profiles)
	{
		if (name && p.name && std::string_view(name) == p.name)
			return &p;
	}
	return &profiles[0];
}

}  // namespace esp_probe

#endif
