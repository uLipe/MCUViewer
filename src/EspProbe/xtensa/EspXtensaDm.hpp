#ifndef ESP_XTENSA_DM_HPP
#define ESP_XTENSA_DM_HPP

#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace esp_probe
{

struct EspChipProfile;
class EspJtagTap;

class EspXtensaDm
{
   public:
	EspXtensaDm(EspJtagTap& tap, const EspChipProfile& profile);

	bool init();
	void shutdown();
	using MemoryEntry = std::pair<uint32_t, uint8_t>;

	bool readMemory(uint32_t address, uint8_t* buf, uint32_t size);
	bool readMemoryBatch(const std::vector<MemoryEntry>& entries, std::unordered_map<uint32_t, uint32_t>& values);
	bool writeMemory(uint32_t address, const uint8_t* buf, uint32_t size);
	const std::string& lastError() const { return last_error_; }

   private:
	unsigned bypassDrBits() const;
	bool selectIr(uint8_t ir);
	bool pwrWrite(uint8_t value);
	bool narRead(uint8_t nar, uint32_t* value);
	bool narWrite(uint8_t nar, uint32_t value);
	bool execIns(uint32_t ins);
	bool readDsr(uint32_t* dsr);
	bool ensureHalted();
	bool resumeIfHalted();
	bool readMemory32(uint32_t address, uint32_t* value);

	EspJtagTap& tap_;
	const EspChipProfile& profile_;
	std::string last_error_;
};

}  // namespace esp_probe

#endif
