#include "EspXtensaDm.hpp"

#include "EspJtagTap.hpp"
#include "esp_bit_utils.hpp"
#include "esp_riscv_regs.hpp"

#include <cstring>
#include <unordered_map>
#include <vector>

namespace esp_probe
{

namespace
{

constexpr uint8_t kXtIrBypass = 0x1f;
constexpr uint8_t kXtIrIdcode = 0x1e;
constexpr uint8_t kXtIrNarsel = 0x1c;
constexpr uint8_t kXtIrPwrctl = 0x08;

constexpr uint8_t kNarOcdId = 0x40;
constexpr uint8_t kNarDcrClr = 0x42;
constexpr uint8_t kNarDcrSet = 0x43;
constexpr uint8_t kNarDsr = 0x44;
constexpr uint8_t kNarDdr = 0x45;
constexpr uint8_t kNarDir0Exec = 0x47;

constexpr uint32_t kOcdDcrEnableOcd = 1u << 0;
constexpr uint32_t kOcdDcrDebugInterrupt = 1u << 1;
constexpr uint32_t kOcdDsrStopped = 1u << 4;

constexpr uint8_t kXtSrDdr = 0x68;
constexpr uint8_t kXtRegA3 = 0x03;

constexpr uint32_t kInsRsrDdrToA3 = 0x030000u | (static_cast<uint32_t>(kXtSrDdr) << 12) | (static_cast<uint32_t>(kXtRegA3) << 4);
constexpr uint32_t kInsLddr32pA3 = 0x0070e0u | (static_cast<uint32_t>(kXtRegA3) << 8);
constexpr uint32_t kInsRfdo = 0xf1e000u;

constexpr uint8_t kPwrWakeup = (1u << 0) | (1u << 1) | (1u << 2);
constexpr uint8_t kPwrJtagDebugUse = 1u << 7;

constexpr int kHaltPollMax = 200;

bool readIdcodeXtensa(EspJtagTap& tap, const EspChipProfile& profile, uint32_t* idcode, std::string* err)
{
	std::vector<uint8_t> ir;
	ir.reserve(profile.tap_count);
	for (unsigned t = 0; t < profile.tap_count; ++t)
		ir.push_back(t == profile.cpu_tap_index ? kXtIrIdcode : kXtIrBypass);
	if (!tap.irScan(ir, profile.ir_len))
	{
		if (err)
			*err = "IDCODE IR scan failed";
		return false;
	}

	const unsigned dr_bits = (profile.tap_count > 1 ? profile.tap_count - 1 : 0) + 32;
	std::vector<uint8_t> out_buf((dr_bits + 7) / 8, 0);
	std::vector<uint8_t> in_buf(out_buf.size(), 0);
	if (!tap.drScan(dr_bits, out_buf.data(), in_buf.data()))
	{
		if (err)
			*err = "IDCODE DR scan failed";
		return false;
	}

	const unsigned bypass = profile.tap_count > 1 ? profile.tap_count - 1 : 0;
	*idcode = buf_get_u32(in_buf.data(), bypass, 32);
	return true;
}

uint32_t packRawValue(uint32_t address, uint8_t size, uint32_t word)
{
	const uint8_t shift = static_cast<uint8_t>((address & 3u) * 8u);
	if (size == 1)
		return (word >> shift) & 0xffu;
	if (size == 2)
		return (word >> shift) & 0xffffu;
	return word;
}

}  // namespace

EspXtensaDm::EspXtensaDm(EspJtagTap& tap, const EspChipProfile& profile) : tap_(tap), profile_(profile) {}

unsigned EspXtensaDm::bypassDrBits() const
{
	unsigned bits = 0;
	for (unsigned t = 0; t < profile_.tap_count; ++t)
	{
		if (t != profile_.cpu_tap_index)
			bits += 1;
	}
	return bits;
}

bool EspXtensaDm::selectIr(uint8_t ir)
{
	std::vector<uint8_t> irs;
	irs.reserve(profile_.tap_count);
	for (unsigned t = 0; t < profile_.tap_count; ++t)
		irs.push_back(t == profile_.cpu_tap_index ? ir : kXtIrBypass);
	if (!tap_.irScan(irs, profile_.ir_len))
		return false;
	tap_.runIdle(1);
	return true;
}

bool EspXtensaDm::pwrWrite(uint8_t value)
{
	if (!selectIr(kXtIrPwrctl))
		return false;

	const unsigned dr_bits = bypassDrBits() + 8;
	std::vector<uint8_t> out_buf((dr_bits + 7) / 8, 0);
	buf_set_u32(out_buf.data(), bypassDrBits(), 8, value);
	return tap_.drScan(dr_bits, out_buf.data(), nullptr);
}

bool EspXtensaDm::narRead(uint8_t nar, uint32_t* value)
{
	if (!selectIr(kXtIrNarsel))
		return false;

	const unsigned bypass = bypassDrBits();
	const uint8_t nar_cmd = static_cast<uint8_t>(nar << 1);

	std::vector<uint8_t> out_sel((bypass + 8 + 7) / 8, 0);
	std::vector<uint8_t> in_sel(out_sel.size(), 0);
	buf_set_u32(out_sel.data(), bypass, 8, nar_cmd);
	if (!tap_.drScan(bypass + 8, out_sel.data(), in_sel.data()))
		return false;

	const unsigned dr_bits = bypass + 32;
	std::vector<uint8_t> out_data((dr_bits + 7) / 8, 0);
	std::vector<uint8_t> in_data(out_data.size(), 0);
	if (!tap_.drScan(dr_bits, out_data.data(), in_data.data()))
		return false;

	*value = buf_get_u32(in_data.data(), bypass, 32);
	tap_.runIdle(1);
	return true;
}

bool EspXtensaDm::narWrite(uint8_t nar, uint32_t value)
{
	if (!selectIr(kXtIrNarsel))
		return false;

	const unsigned bypass = bypassDrBits();
	const uint8_t nar_cmd = static_cast<uint8_t>((nar << 1) | 1u);

	std::vector<uint8_t> out_sel((bypass + 8 + 7) / 8, 0);
	buf_set_u32(out_sel.data(), bypass, 8, nar_cmd);
	if (!tap_.drScan(bypass + 8, out_sel.data(), nullptr))
		return false;

	std::vector<uint8_t> out_data((bypass + 32 + 7) / 8, 0);
	buf_set_u32(out_data.data(), bypass, 32, value);
	if (!tap_.drScan(bypass + 32, out_data.data(), nullptr))
		return false;

	tap_.runIdle(1);
	return true;
}

bool EspXtensaDm::execIns(uint32_t ins)
{
	return narWrite(kNarDir0Exec, ins);
}

bool EspXtensaDm::readDsr(uint32_t* dsr)
{
	return narRead(kNarDsr, dsr);
}

bool EspXtensaDm::ensureHalted()
{
	uint32_t dsr = 0;
	if (!readDsr(&dsr))
		return false;
	if (dsr & kOcdDsrStopped)
		return true;

	if (!narWrite(kNarDcrSet, kOcdDcrEnableOcd | kOcdDcrDebugInterrupt))
		return false;
	tap_.runIdle(2);

	for (int i = 0; i < kHaltPollMax; ++i)
	{
		if (!readDsr(&dsr))
			return false;
		if (dsr & kOcdDsrStopped)
			return true;
		tap_.runIdle(1);
	}

	last_error_ = "Xtensa halt timeout";
	return false;
}

bool EspXtensaDm::resumeIfHalted()
{
	uint32_t dsr = 0;
	if (!readDsr(&dsr))
		return false;
	if (!(dsr & kOcdDsrStopped))
		return true;

	if (!execIns(kInsRfdo))
		return false;
	tap_.runIdle(2);
	return true;
}

bool EspXtensaDm::init()
{
	last_error_.clear();
	tap_.reset();
	tap_.runIdle(10);

	uint32_t idcode = 0;
	if (!readIdcodeXtensa(tap_, profile_, &idcode, nullptr))
	{
		last_error_ = "IDCODE verify failed";
		return false;
	}
	if (idcode != profile_.idcode)
	{
		last_error_ = "IDCODE mismatch";
		return false;
	}

	if (!pwrWrite(kPwrWakeup))
		return false;
	if (!pwrWrite(static_cast<uint8_t>(kPwrWakeup | kPwrJtagDebugUse)))
		return false;
	tap_.runIdle(2);

	if (!narWrite(kNarDcrSet, kOcdDcrEnableOcd))
		return false;

	uint32_t ocdid = 0;
	if (!narRead(kNarOcdId, &ocdid) || ocdid == 0 || ocdid == 0xffffffffu)
	{
		last_error_ = "Xtensa debug module offline";
		return false;
	}

	return true;
}

void EspXtensaDm::shutdown()
{
	uint32_t dsr = 0;
	if (readDsr(&dsr) && (dsr & kOcdDsrStopped))
	{
		execIns(kInsRfdo);
		narWrite(kNarDcrClr, kOcdDcrDebugInterrupt);
	}
	tap_.runIdle(5);
}

bool EspXtensaDm::readMemory32(uint32_t address, uint32_t* value)
{
	if (!ensureHalted())
		return false;

	if (!narWrite(kNarDdr, address))
		goto fail;
	if (!execIns(kInsRsrDdrToA3))
		goto fail;
	if (!execIns(kInsLddr32pA3))
		goto fail;
	if (!narRead(kNarDdr, value))
		goto fail;

	if (!resumeIfHalted())
		goto fail;
	return true;

fail:
	resumeIfHalted();
	if (last_error_.empty())
		last_error_ = "Xtensa memory read failed";
	return false;
}

bool EspXtensaDm::readMemory(uint32_t address, uint8_t* buf, uint32_t size)
{
	if (size == 0 || size > 4)
		return false;

	uint32_t word = 0;
	if (!readMemory32(address & ~3u, &word))
		return false;

	const uint8_t shift = static_cast<uint8_t>((address & 3u) * 8u);
	for (uint32_t i = 0; i < size; ++i)
		buf[i] = static_cast<uint8_t>((word >> (shift + i * 8u)) & 0xffu);
	return true;
}

bool EspXtensaDm::readMemoryBatch(const std::vector<MemoryEntry>& entries,
	std::unordered_map<uint32_t, uint32_t>& values)
{
	values.clear();
	if (entries.empty())
		return true;

	if (!ensureHalted())
		return false;

	std::unordered_map<uint32_t, uint32_t> words;
	words.reserve(entries.size());
	for (const auto& [address, size] : entries)
	{
		(void)size;
		words[address & ~3u] = 0;
	}

	bool ok = true;
	for (const auto& [aligned, _] : words)
	{
		uint32_t word = 0;
		if (!narWrite(kNarDdr, aligned))
		{
			ok = false;
			break;
		}
		if (!execIns(kInsRsrDdrToA3))
		{
			ok = false;
			break;
		}
		if (!execIns(kInsLddr32pA3))
		{
			ok = false;
			break;
		}
		if (!narRead(kNarDdr, &word))
		{
			ok = false;
			break;
		}
		words[aligned] = word;
	}

	if (!resumeIfHalted())
		ok = false;

	if (!ok)
	{
		last_error_ = "Memory batch read failed";
		return false;
	}

	for (const auto& [address, size] : entries)
		values[address] = packRawValue(address, size, words[address & ~3u]);
	return true;
}

bool EspXtensaDm::writeMemory(uint32_t address, const uint8_t* buf, uint32_t size)
{
	(void)address;
	(void)buf;
	(void)size;
	last_error_ = "Xtensa memory write not implemented";
	return false;
}

}  // namespace esp_probe
