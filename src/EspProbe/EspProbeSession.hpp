#ifndef ESP_PROBE_SESSION_HPP
#define ESP_PROBE_SESSION_HPP

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "esp_riscv_regs.hpp"
#include "jtag/EspJtagTap.hpp"
#include "riscv/EspRiscvDm.hpp"
#include "usb/EspUsbJtagTransport.hpp"
#include "xtensa/EspXtensaDm.hpp"

namespace esp_probe
{

class EspProbeSession
{
   public:
	bool connect(const std::string& serial, const std::string& chip_name, uint32_t speed_khz);
	void disconnect();
	bool isConnected() const { return connected_; }

	bool readMemory(uint32_t address, uint8_t* buf, uint32_t size);
	bool readMemoryBatch(const std::vector<std::pair<uint32_t, uint8_t>>& entries,
		std::unordered_map<uint32_t, uint32_t>& values);
	bool writeMemory(uint32_t address, const uint8_t* buf, uint32_t size);

	const std::string& lastError() const { return last_error_; }

   private:
	std::unique_ptr<EspUsbJtagTransport> transport_;
	std::unique_ptr<EspJtagTap> tap_;
	std::unique_ptr<EspRiscvDm> riscv_dm_;
	std::unique_ptr<EspXtensaDm> xtensa_dm_;
	const EspChipProfile* profile_ = nullptr;
	bool connected_ = false;
	std::string last_error_;
};

}  // namespace esp_probe

#endif
