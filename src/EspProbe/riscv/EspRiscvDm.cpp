#include "EspRiscvDm.hpp"

#include "EspJtagTap.hpp"
#include "esp_bit_utils.hpp"
#include "esp_riscv_regs.hpp"

#include <cstdio>
#include <cstring>
#include <string>
#include <unordered_map>
#include <vector>

namespace esp_probe
{

namespace
{

void log_step(const char* msg, uint32_t a = 0, uint32_t b = 0)
{
	if (!std::getenv("ESP_PROBE_VERBOSE"))
		return;
	std::fprintf(stderr, "[esp_probe] %s", msg);
	if (a || b)
		std::fprintf(stderr, " 0x%08x 0x%08x", a, b);
	std::fprintf(stderr, "\n");
}

bool readIdcode(EspJtagTap& tap, const EspChipProfile& profile, uint32_t* idcode, std::string* err)
{
	std::vector<uint8_t> ir;
	ir.reserve(profile.tap_count);
	for (unsigned t = 0; t < profile.tap_count; ++t)
		ir.push_back(t == profile.cpu_tap_index ? kIrIdcode : kIrBypass);
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

EspRiscvDm::EspRiscvDm(EspJtagTap& tap, const EspChipProfile& profile) : tap_(tap), profile_(profile) {}

bool EspRiscvDm::beginDmiBatch()
{
	batch_active_ = selectDbusIr();
	return batch_active_;
}

void EspRiscvDm::endDmiBatch()
{
	batch_active_ = false;
}

bool EspRiscvDm::selectDbusIr()
{
	std::vector<uint8_t> ir;
	ir.reserve(profile_.tap_count);
	for (unsigned t = 0; t < profile_.tap_count; ++t)
		ir.push_back(t == profile_.cpu_tap_index ? kIrDbus : kIrBypass);
	if (!tap_.irScan(ir, profile_.ir_len))
		return false;
	tap_.runIdle(2);
	return true;
}

bool EspRiscvDm::selectDtmControlIr()
{
	std::vector<uint8_t> ir;
	ir.reserve(profile_.tap_count);
	for (unsigned t = 0; t < profile_.tap_count; ++t)
		ir.push_back(t == profile_.cpu_tap_index ? kIrDtmControl : kIrBypass);
	if (!tap_.irScan(ir, profile_.ir_len))
		return false;
	tap_.runIdle(2);
	return true;
}

unsigned EspRiscvDm::bypassDrBits() const
{
	unsigned bits = 0;
	for (unsigned t = 0; t < profile_.tap_count; ++t)
	{
		if (t != profile_.cpu_tap_index)
			bits += 1;
	}
	return bits;
}

unsigned EspRiscvDm::dmiDrBits() const
{
	return bypassDrBits() + abits_ + kDtmDmiDataLength + kDtmDmiOpLength;
}

bool EspRiscvDm::dtmControlScan(uint32_t out, uint32_t* in)
{
	if (!selectDtmControlIr())
		return false;

	const unsigned dr_bits = bypassDrBits() + 32;
	std::vector<uint8_t> out_buf((dr_bits + 7) / 8, 0);
	std::vector<uint8_t> in_buf(out_buf.size(), 0);
	buf_set_u32(out_buf.data(), bypassDrBits(), 32, out);

	if (!tap_.drScan(dr_bits, out_buf.data(), in_buf.data()))
		return false;

	selectDbusIr();
	if (in)
		*in = buf_get_u32(in_buf.data(), bypassDrBits(), 32);
	return true;
}

bool EspRiscvDm::init()
{
	last_error_.clear();
	tap_.reset();
	tap_.runIdle(10);

	uint32_t idcode = 0;
	if (!readIdcode(tap_, profile_, &idcode, nullptr))
	{
		last_error_ = "IDCODE verify failed";
		return false;
	}
	log_step("idcode", idcode, profile_.idcode);
	if ((idcode & 0xfffffffu) != (profile_.idcode & 0xfffffffu))
	{
		last_error_ = "IDCODE mismatch (wrong chip profile?)";
		return false;
	}

	uint32_t dtmcs = 0;
	if (!dtmControlScan(0, &dtmcs))
	{
		last_error_ = "DTMCS read failed";
		return false;
	}
	if ((dtmcs & kDtmDtmcsVersion) != kDtmDtmcsVersion10 && ((dtmcs >> 1) & kDtmDtmcsVersion) == kDtmDtmcsVersion10)
		dtmcs >>= 1;
	log_step("dtmcs", dtmcs, 0);

	if (dtmcs == 0)
	{
		last_error_ = "DTMCS is zero (JTAG not connected?)";
		return false;
	}

	const uint32_t version = get_field_u32(dtmcs, kDtmDtmcsVersion);
	if (version != kDtmDtmcsVersion10)
	{
		last_error_ = "Unsupported DTM version";
		return false;
	}

	abits_ = get_field_u32(dtmcs, kDtmDtmcsAbits) >> 4;
	if (abits_ == 0)
		abits_ = 7;

	dmi_busy_delay_ = get_field_u32(dtmcs, kDtmDtmcsIdle) >> 12;

	if (!selectDbusIr())
	{
		last_error_ = "Failed to select DBUS IR";
		return false;
	}
	tap_.runIdle(2);

	if (!dmiWrite(kDmDmControl, 0))
	{
		last_error_ = "dmcontrol clear failed";
		return false;
	}

	if (!dmiWrite(kDmDmControl, kDmControlDmActive))
	{
		last_error_ = "dmactive set failed";
		return false;
	}

	const uint32_t dmcontrol_sel = kDmControlDmActive | kDmControlHartSelLo | kDmControlHartSelHi | kDmControlHaSel;
	if (!dmiWrite(kDmDmControl, dmcontrol_sel))
	{
		last_error_ = "dmcontrol hartsel failed";
		return false;
	}

	uint32_t dmcontrol = 0;
	if (!dmiRead(kDmDmControl, &dmcontrol))
	{
		last_error_ = "dmcontrol read failed";
		return false;
	}
	log_step("dmcontrol", dmcontrol, 0);

	if ((dmcontrol & kDmControlDmActive) == 0)
	{
		char msg[80];
		std::snprintf(msg, sizeof(msg), "Debug module did not activate (dmcontrol=0x%08x)", dmcontrol);
		last_error_ = msg;
		return false;
	}

	uint32_t dmstatus = 0;
	if (!dmiRead(kDmDmStatus, &dmstatus))
	{
		last_error_ = "dmstatus read failed";
		return false;
	}
	log_step("dmstatus", dmstatus, 0);

	uint32_t sbcs = 0;
	if (!dmiRead(kDmSbCs, &sbcs))
	{
		last_error_ = "sbcs read failed";
		return false;
	}
	log_step("sbcs", sbcs, 0);

	sb_sba_v1_ = (get_field_u32(sbcs, kDmSbCsSbVersion) >> 29) == kDmSbCsSbVersion10;

	return true;
}

void EspRiscvDm::shutdown()
{
	if (!selectDbusIr())
		return;

	const uint32_t resume = kDmControlDmActive | kDmControlResumeReq;
	dmiWrite(kDmDmControl, resume);
	tap_.runIdle(10);
	dmiWrite(kDmDmControl, 0);
}

uint32_t EspRiscvDm::dmiScan(uint32_t op, uint32_t address, uint32_t data_out, uint32_t* data_in)
{
	if (!selectDbusIr())
		return kDmiStatusFailed;

	const unsigned dr_bits = dmiDrBits();
	std::vector<uint8_t> out_buf((dr_bits + 7) / 8, 0);
	std::vector<uint8_t> in_buf(out_buf.size(), 0);

	const unsigned dmi_offset = bypassDrBits();
	buf_set_u32(out_buf.data(), dmi_offset + kDmiOpOffset, kDtmDmiOpLength, op);
	buf_set_u32(out_buf.data(), dmi_offset + kDmiDataOffset, kDtmDmiDataLength, data_out);
	buf_set_u32(out_buf.data(), dmi_offset + kDmiAddressOffset, abits_, address);

	if (!tap_.drScan(dr_bits, out_buf.data(), in_buf.data()))
		return kDmiStatusFailed;

	if (dmi_busy_delay_ > 0)
		tap_.runIdle(dmi_busy_delay_);

	const uint32_t status = buf_get_u32(in_buf.data(), dmi_offset + kDmiOpOffset, kDtmDmiOpLength);
	if (data_in)
		*data_in = buf_get_u32(in_buf.data(), dmi_offset + kDmiDataOffset, kDtmDmiDataLength);
	return status;
}

bool EspRiscvDm::dmiOp(uint32_t op, uint32_t address, uint32_t data_out, uint32_t* data_in)
{
	for (unsigned attempt = 0; attempt < 32; ++attempt)
	{
		uint32_t read_back = 0;
		const uint32_t status = dmiScan(op, address, data_out, &read_back);
		if (status == kDmiStatusBusy)
		{
			dmi_busy_delay_ += dmi_busy_delay_ / 10 + 1;
			continue;
		}
		if (status != kDmiStatusSuccess)
		{
			last_error_ = "DMI op failed status=" + std::to_string(status);
			return false;
		}

		uint32_t nop_data = 0;
		const uint32_t nop_status = dmiScan(kDmiOpNop, 0, 0, &nop_data);
		if (nop_status == kDmiStatusBusy)
		{
			dmi_busy_delay_ += dmi_busy_delay_ / 10 + 1;
			continue;
		}
		if (nop_status != kDmiStatusSuccess)
			return false;

		if (data_in)
			*data_in = nop_data;
		return true;
	}
	return false;
}

bool EspRiscvDm::dmiRead(uint32_t address, uint32_t* value)
{
	return dmiOp(kDmiOpRead, address, 0, value);
}

bool EspRiscvDm::dmiWrite(uint32_t address, uint32_t value)
{
	return dmiOp(kDmiOpWrite, address, value, nullptr);
}

bool EspRiscvDm::readMemory32(uint32_t address, uint32_t* value)
{
	if (sb_sba_v1_)
	{
		uint32_t sbcs_write = set_field_u32(0, kDmSbCsSbReadOnAddr, 1u);
		sbcs_write = set_field_u32(sbcs_write, kDmSbCsSbAccess, 2u);
		if (!dmiWrite(kDmSbCs, sbcs_write))
			return false;
		if (!dmiWrite(kDmSbAddress0, address))
			return false;
	}
	else
	{
		if (!dmiWrite(kDmSbAddress0, address))
			return false;

		uint32_t sbcs = set_field_u32(0, kDmSbCsSbAccess, 2u);
		sbcs = set_field_u32(sbcs, kDmSbCsSbSingleRead, 1u);
		if (!dmiWrite(kDmSbCs, sbcs))
			return false;
	}

	return dmiRead(kDmSbData0, value);
}

bool EspRiscvDm::readMemoryBatch(const std::vector<MemoryEntry>& entries, std::unordered_map<uint32_t, uint32_t>& values)
{
	values.clear();
	if (entries.empty())
		return true;

	if (!beginDmiBatch())
	{
		last_error_ = "DMI batch start failed";
		return false;
	}

	bool ok = false;
	std::unordered_map<uint32_t, uint32_t> words;

	if (sb_sba_v1_)
	{
		uint32_t sbcs_write = set_field_u32(0, kDmSbCsSbReadOnAddr, 1u);
		sbcs_write = set_field_u32(sbcs_write, kDmSbCsSbAccess, 2u);
		if (!dmiWrite(kDmSbCs, sbcs_write))
			goto done;
	}

	for (const auto& [address, size] : entries)
	{
		(void)size;
		const uint32_t aligned = address & ~3u;
		if (words.contains(aligned))
			continue;

		if (!sb_sba_v1_)
		{
			if (!dmiWrite(kDmSbAddress0, aligned))
				goto done;
			uint32_t sbcs = set_field_u32(0, kDmSbCsSbAccess, 2u);
			sbcs = set_field_u32(sbcs, kDmSbCsSbSingleRead, 1u);
			if (!dmiWrite(kDmSbCs, sbcs))
				goto done;
		}
		else if (!dmiWrite(kDmSbAddress0, aligned))
		{
			goto done;
		}

		uint32_t word = 0;
		if (!dmiRead(kDmSbData0, &word))
			goto done;
		words[aligned] = word;
	}

	ok = true;
done:
	endDmiBatch();
	if (!ok)
	{
		last_error_ = "Memory batch read failed";
		return false;
	}

	for (const auto& [address, size] : entries)
		values[address] = packRawValue(address, size, words[address & ~3u]);
	return true;
}

bool EspRiscvDm::writeMemory32(uint32_t address, uint32_t value)
{
	uint32_t sbcs = 0;
	if (!dmiRead(kDmSbCs, &sbcs))
		return false;

	if (!dmiWrite(kDmSbAddress0, address))
		return false;

	sbcs = set_field_u32(sbcs, kDmSbCsSbAccess, 2u);
	if (!dmiWrite(kDmSbCs, sbcs))
		return false;

	return dmiWrite(kDmSbData0, value);
}

bool EspRiscvDm::readMemory(uint32_t address, uint8_t* buf, uint32_t size)
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

bool EspRiscvDm::writeMemory(uint32_t address, const uint8_t* buf, uint32_t size)
{
	if (size == 0 || size > 4)
		return false;

	uint32_t word = 0;
	if (size < 4 || (address & 3u) != 0)
	{
		if (!readMemory32(address & ~3u, &word))
			return false;
	}

	const uint8_t shift = static_cast<uint8_t>((address & 3u) * 8u);
	for (uint32_t i = 0; i < size; ++i)
	{
		const uint32_t mask = ~(0xffu << (shift + i * 8u));
		word = (word & mask) | (static_cast<uint32_t>(buf[i]) << (shift + i * 8u));
	}
	return writeMemory32(address & ~3u, word);
}

}  // namespace esp_probe
