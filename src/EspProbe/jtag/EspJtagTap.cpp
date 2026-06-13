#include "EspJtagTap.hpp"

#include <cstring>

#include "EspUsbJtagTransport.hpp"

namespace esp_probe
{

EspJtagTap::EspJtagTap(EspUsbJtagTransport& transport) : transport_(transport) {}

void EspJtagTap::clockBit(int tms, int tdi, bool capture, int* tdo_out)
{
	transport_.clk(tms, tdi, capture);
	if (capture && tdo_out)
	{
		if (transport_.flush() != 0)
			*tdo_out = 0;
		else
			*tdo_out = transport_.readTdoBit();
	}
}

void EspJtagTap::reset()
{
	transport_.tapReset();
	state_ = State::TestLogicReset;
}

void EspJtagTap::runIdle(unsigned cycles)
{
	moveToRunIdle();
	for (unsigned i = 0; i < cycles; ++i)
		transport_.clk(0, 0, false);
	transport_.flush();
	state_ = State::RunIdle;
}

void EspJtagTap::moveToRunIdle()
{
	if (state_ == State::RunIdle)
		return;
	if (state_ == State::TestLogicReset)
		transport_.clk(0, 0, false);
	else
	{
		transport_.clk(1, 0, false);
		transport_.clk(1, 0, false);
		transport_.clk(0, 0, false);
	}
	transport_.flush();
	state_ = State::RunIdle;
}

void EspJtagTap::moveToShiftIr()
{
	moveToRunIdle();
	transport_.clk(1, 0, false);
	transport_.clk(1, 0, false);
	transport_.clk(0, 0, false);
	transport_.clk(0, 0, false);
}

void EspJtagTap::moveToCaptureDr()
{
	moveToRunIdle();
	transport_.clk(1, 0, false);
	transport_.clk(0, 0, false);
}

void EspJtagTap::moveToShiftDr()
{
	moveToRunIdle();
	transport_.clk(1, 0, false);
	transport_.clk(0, 0, false);
	transport_.clk(0, 0, false);
}

void EspJtagTap::exitShiftToRunIdle()
{
	transport_.clk(1, 0, false);
	transport_.clk(0, 0, false);
	transport_.flush();
	state_ = State::RunIdle;
}

bool EspJtagTap::irScan(const std::vector<uint8_t>& ir_per_tap, unsigned ir_len)
{
	if (ir_len == 0 || ir_per_tap.empty())
		return false;

	moveToShiftIr();
	for (unsigned t = 0; t < ir_per_tap.size(); ++t)
	{
		const uint8_t ir = ir_per_tap[t];
		for (unsigned b = 0; b < ir_len; ++b)
		{
			const bool last = (t == ir_per_tap.size() - 1) && (b == ir_len - 1);
			const int tdi = (ir >> b) & 1;
			transport_.clk(last ? 1 : 0, tdi, false);
		}
	}

	transport_.flush();
	exitShiftToRunIdle();
	return true;
}

bool EspJtagTap::drScan(unsigned dr_bits, const uint8_t* out_bits, uint8_t* in_bits)
{
	if (dr_bits == 0)
		return false;

	if (in_bits)
		std::memset(in_bits, 0, (dr_bits + 7) / 8);

	if (dr_bits == 32)
		moveToCaptureDr();
	else
		moveToShiftDr();

	const bool align_tdo = in_bits != nullptr && dr_bits == 32;
	const unsigned capture_bits = align_tdo ? dr_bits + 1 : dr_bits;
	for (unsigned i = 0; i < capture_bits; ++i)
	{
		const bool last = (i == capture_bits - 1);
		const int tdi = (out_bits && i < dr_bits) ? ((out_bits[i / 8] >> (i % 8)) & 1) : 0;
		transport_.clk(last ? 1 : 0, tdi, in_bits != nullptr);
	}

	if (transport_.flush() != 0)
		return false;

	if (in_bits)
	{
		if (align_tdo && transport_.readTdoBit() < 0)
			return false;
		for (unsigned i = 0; i < dr_bits; ++i)
		{
			const int tdo = transport_.readTdoBit();
			if (tdo < 0)
				return false;
			if (tdo > 0)
				in_bits[i / 8] |= static_cast<uint8_t>(1u << (i % 8));
		}
	}

	exitShiftToRunIdle();
	return true;
}

}  // namespace esp_probe
