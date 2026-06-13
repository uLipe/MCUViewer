#ifndef ESP_JTAG_TAP_HPP
#define ESP_JTAG_TAP_HPP

#include <cstdint>
#include <vector>

namespace esp_probe
{

class EspUsbJtagTransport;

class EspJtagTap
{
   public:
	explicit EspJtagTap(EspUsbJtagTransport& transport);

	void reset();
	void runIdle(unsigned cycles);
	bool irScan(const std::vector<uint8_t>& ir_per_tap, unsigned ir_len);
	bool drScan(unsigned dr_bits, const uint8_t* out_bits, uint8_t* in_bits);

   private:
	enum class State
	{
		TestLogicReset,
		RunIdle,
		Other,
	};

	void moveToRunIdle();
	void moveToShiftIr();
	void moveToCaptureDr();
	void moveToShiftDr();
	void exitShiftToRunIdle();
	void clockBit(int tms, int tdi, bool capture, int* tdo_out = nullptr);

	EspUsbJtagTransport& transport_;
	State state_ = State::TestLogicReset;
};

}  // namespace esp_probe

#endif
