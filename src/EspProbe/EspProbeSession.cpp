#include "EspProbeSession.hpp"

namespace esp_probe
{

bool EspProbeSession::connect(const std::string& serial, const std::string& chip_name, uint32_t speed_khz)
{
	disconnect();
	last_error_.clear();

	profile_ = find_chip_profile(chip_name.c_str());
	if (!profile_->is_riscv)
	{
		last_error_ = "Chip architecture not supported yet (Xtensa pending)";
		return false;
	}

	transport_ = std::make_unique<EspUsbJtagTransport>();
	if (!transport_->open(serial))
	{
		last_error_ = transport_->lastError();
		if (last_error_.empty())
			last_error_ = "ESP USB-JTAG device not found";
		return false;
	}
	if (!transport_->setSpeedKhz(speed_khz))
	{
		last_error_ = "Failed to set JTAG speed";
		return false;
	}

	tap_ = std::make_unique<EspJtagTap>(*transport_);
	dm_ = std::make_unique<EspRiscvDm>(*tap_, *profile_);

	if (!dm_->init())
	{
		last_error_ = dm_->lastError();
		if (last_error_.empty())
			last_error_ = "RISC-V debug module init failed";
		disconnect();
		return false;
	}

	connected_ = true;
	return true;
}

void EspProbeSession::disconnect()
{
	if (dm_)
		dm_->shutdown();
	dm_.reset();
	tap_.reset();
	if (transport_)
		transport_->close();
	transport_.reset();
	connected_ = false;
}

bool EspProbeSession::readMemoryBatch(const std::vector<std::pair<uint32_t, uint8_t>>& entries,
	std::unordered_map<uint32_t, uint32_t>& values)
{
	if (!connected_ || !dm_)
	{
		last_error_ = "Not connected";
		return false;
	}
	if (!dm_->readMemoryBatch(entries, values))
	{
		last_error_ = dm_->lastError();
		if (last_error_.empty())
			last_error_ = "Memory batch read failed";
		return false;
	}
	return true;
}

bool EspProbeSession::readMemory(uint32_t address, uint8_t* buf, uint32_t size)
{
	if (!connected_ || !dm_)
	{
		last_error_ = "Not connected";
		return false;
	}
	if (!dm_->readMemory(address, buf, size))
	{
		last_error_ = "Memory read failed";
		return false;
	}
	return true;
}

bool EspProbeSession::writeMemory(uint32_t address, const uint8_t* buf, uint32_t size)
{
	if (!connected_ || !dm_)
	{
		last_error_ = "Not connected";
		return false;
	}
	if (!dm_->writeMemory(address, buf, size))
	{
		last_error_ = "Memory write failed";
		return false;
	}
	return true;
}

}  // namespace esp_probe
