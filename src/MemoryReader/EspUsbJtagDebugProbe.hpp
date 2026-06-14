#ifndef ESP_USB_JTAG_DEBUG_PROBE_HPP
#define ESP_USB_JTAG_DEBUG_PROBE_HPP

#include <mutex>
#include <string>
#include <vector>

#include "EspProbeSession.hpp"
#include "IDebugProbe.hpp"
#include "spdlog/spdlog.h"

class EspUsbJtagDebugProbe : public IDebugProbe
{
   public:
	explicit EspUsbJtagDebugProbe(spdlog::logger* logger);

	bool startAcqusition(const DebugProbeSettings& probeSettings, std::vector<std::pair<uint32_t, uint8_t>>& addressSizeVector, uint32_t samplingFreqency) override;
	bool stopAcqusition() override;
	bool isValid() const override;
	std::string getTargetName() override;

	std::optional<varEntryType> readSingleEntry() override;
	bool readMemory(uint32_t address, uint8_t* buf, uint32_t size) override;
	bool writeMemory(uint32_t address, uint8_t* buf, uint32_t size) override;

	bool supportsBatchRead() const override { return true; }
	bool readMemoryBatch(const std::vector<std::pair<uint32_t, uint8_t>>& entries,
		std::unordered_map<uint32_t, uint32_t>& values) override;

	std::string getLastErrorMsg() const override;
	std::vector<std::string> getConnectedDevices() override;

   private:
	esp_probe::EspProbeSession session_;
	spdlog::logger* logger;
	std::string chip_name = "esp32c6";
};

#endif
