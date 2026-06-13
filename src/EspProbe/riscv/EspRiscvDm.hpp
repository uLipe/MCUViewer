#ifndef ESP_RISCV_DM_HPP
#define ESP_RISCV_DM_HPP

#include <cstdint>
#include <string>

namespace esp_probe
{

struct EspChipProfile;
class EspJtagTap;

class EspRiscvDm
{
   public:
	EspRiscvDm(EspJtagTap& tap, const EspChipProfile& profile);

	bool init();
	void shutdown();
	bool readMemory(uint32_t address, uint8_t* buf, uint32_t size);
	bool writeMemory(uint32_t address, const uint8_t* buf, uint32_t size);
	const std::string& lastError() const { return last_error_; }

   private:
	bool selectDbusIr();
	bool selectDtmControlIr();
	unsigned bypassDrBits() const;
	unsigned dmiDrBits() const;
	bool dtmControlScan(uint32_t out, uint32_t* in);
	uint32_t dmiScan(uint32_t op, uint32_t address, uint32_t data_out, uint32_t* data_in);
	bool dmiOp(uint32_t op, uint32_t address, uint32_t data_out, uint32_t* data_in);
	bool dmiRead(uint32_t address, uint32_t* value);
	bool dmiWrite(uint32_t address, uint32_t value);
	bool readMemory32(uint32_t address, uint32_t* value);
	bool writeMemory32(uint32_t address, uint32_t value);

	EspJtagTap& tap_;
	const EspChipProfile& profile_;
	unsigned abits_ = 7;
	unsigned dmi_busy_delay_ = 0;
	std::string last_error_;
};

}  // namespace esp_probe

#endif
