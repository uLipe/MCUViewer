#include "EspUsbJtagDebugProbe.hpp"

EspUsbJtagDebugProbe::EspUsbJtagDebugProbe(spdlog::logger* logger) : logger(logger) {}

bool EspUsbJtagDebugProbe::startAcqusition(const DebugProbeSettings& probeSettings, std::vector<std::pair<uint32_t, uint8_t>>&, uint32_t)
{
	std::lock_guard<std::mutex> lock(mtx);
	isRunning = false;
	lastErrorMsg.clear();

	chip_name = probeSettings.device.empty() ? "esp32c6" : probeSettings.device;

	std::string serial = probeSettings.serialNumber;
	if (serial.find("No debug probes found") != std::string::npos)
		serial.clear();

	if (!session_.connect(serial, chip_name, probeSettings.speedkHz))
	{
		lastErrorMsg = session_.lastError();
		logger->error("ESP USB-JTAG connect failed: {}", lastErrorMsg);
		return false;
	}

	logger->info("ESP USB-JTAG connected (chip={}, speed={} kHz)", chip_name, probeSettings.speedkHz);
	isRunning = true;
	return true;
}

bool EspUsbJtagDebugProbe::stopAcqusition()
{
	std::lock_guard<std::mutex> lock(mtx);
	session_.disconnect();
	isRunning = false;
	return true;
}

bool EspUsbJtagDebugProbe::isValid() const
{
	std::lock_guard<std::mutex> lock(mtx);
	return isRunning;
}

std::string EspUsbJtagDebugProbe::getTargetName()
{
	return chip_name;
}

std::optional<IDebugProbe::varEntryType> EspUsbJtagDebugProbe::readSingleEntry()
{
	return std::nullopt;
}

bool EspUsbJtagDebugProbe::readMemory(uint32_t address, uint8_t* buf, uint32_t size)
{
	std::lock_guard<std::mutex> lock(mtx);
	if (!isRunning)
		return false;

	if (!session_.readMemory(address, buf, size))
	{
		lastErrorMsg = session_.lastError();
		return false;
	}
	return true;
}

bool EspUsbJtagDebugProbe::readMemoryBatch(const std::vector<std::pair<uint32_t, uint8_t>>& entries,
	std::unordered_map<uint32_t, uint32_t>& values)
{
	std::lock_guard<std::mutex> lock(mtx);
	if (!isRunning)
		return false;

	if (!session_.readMemoryBatch(entries, values))
	{
		lastErrorMsg = session_.lastError();
		return false;
	}
	return true;
}

bool EspUsbJtagDebugProbe::writeMemory(uint32_t address, uint8_t* buf, uint32_t size)
{
	std::lock_guard<std::mutex> lock(mtx);
	if (!isRunning)
		return false;

	if (!session_.writeMemory(address, buf, size))
	{
		lastErrorMsg = session_.lastError();
		return false;
	}
	return true;
}

std::string EspUsbJtagDebugProbe::getLastErrorMsg() const
{
	return lastErrorMsg;
}

std::vector<std::string> EspUsbJtagDebugProbe::getConnectedDevices()
{
	esp_probe::EspUsbJtagTransport transport;
	return transport.listDevices();
}
