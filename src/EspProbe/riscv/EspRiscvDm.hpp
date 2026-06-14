#ifndef ESP_RISCV_DM_HPP
#define ESP_RISCV_DM_HPP

#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

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
	using MemoryEntry = std::pair<uint32_t, uint8_t>;

	bool readMemory(uint32_t address, uint8_t* buf, uint32_t size);
	bool readMemoryBatch(const std::vector<MemoryEntry>& entries, std::unordered_map<uint32_t, uint32_t>& values);
	bool writeMemory(uint32_t address, const uint8_t* buf, uint32_t size);
	const std::string& lastError() const { return last_error_; }

   private:
	bool beginDmiBatch();
	void endDmiBatch();
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
	bool batch_active_ = false;
	bool sb_sba_v1_ = false;
	std::string last_error_;
};

}  // namespace esp_probe

#endif
