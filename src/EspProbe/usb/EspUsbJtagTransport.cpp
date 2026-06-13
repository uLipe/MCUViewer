// SPDX-License-Identifier: GPL-3.0-or-later
// Adapted from OpenOCD src/jtag/drivers/esp_usb_jtag.c

#include "EspUsbJtagTransport.hpp"

#include <libusb-1.0/libusb.h>

#include <cstring>

namespace esp_probe
{

namespace
{

constexpr unsigned kLibusbTimeoutMs = 1000;
constexpr unsigned kCmdRepMaxReps = 1024;
constexpr unsigned kVendJtagSetDiv = 0;

constexpr unsigned CMD_CLK(unsigned cap, unsigned tdi, unsigned tms)
{
	return (cap ? 4u : 0u) | (tms ? 2u : 0u) | (tdi ? 1u : 0u);
}
constexpr unsigned CMD_RST(unsigned srst) { return 0x8u | (srst ? 1u : 0u); }
constexpr unsigned CMD_FLUSH = 0xA;
constexpr unsigned CMD_REP(unsigned r) { return 0xCu | ((r) & 3u); }

struct JtagProtoCapsHdr
{
	uint8_t proto_ver;
	uint8_t length;
} __attribute__((packed));

struct JtagGenHdr
{
	uint8_t type;
	uint8_t length;
} __attribute__((packed));

struct JtagProtoCapsSpeedApb
{
	uint8_t type;
	uint8_t length;
	uint8_t apb_speed_10khz[2];
	uint8_t div_min[2];
	uint8_t div_max[2];
} __attribute__((packed));

constexpr uint8_t kJtagProtoCapsSpeedApbType = 1;
constexpr unsigned kJtagBuiltinDescrStartOff = 0;
constexpr unsigned kJtagProtoCapsDataLen = 255;

bool isPlaceholderSerial(const std::string& serial)
{
	return serial.empty() || serial.find("No debug probes found") != std::string::npos;
}

void detachKernelIfNeeded(libusb_device_handle* handle, uint8_t interface)
{
	if (libusb_kernel_driver_active(handle, interface) == 1)
		libusb_detach_kernel_driver(handle, interface);
}

}  // namespace

EspUsbJtagTransport::~EspUsbJtagTransport()
{
	close();
}

bool EspUsbJtagTransport::ensureContext()
{
	if (ctx_ != nullptr)
		return true;
	const int rc = libusb_init(&ctx_);
	if (rc != 0)
	{
		last_error_ = std::string("libusb_init failed: ") + libusb_strerror(static_cast<libusb_error>(rc));
		return false;
	}
	return true;
}

void EspUsbJtagTransport::close()
{
	if (handle_)
	{
		libusb_release_interface(handle_, jtag_interface_);
		if (libusb_kernel_driver_active(handle_, jtag_interface_) == 0)
			libusb_attach_kernel_driver(handle_, jtag_interface_);
		libusb_close(handle_);
		handle_ = nullptr;
	}
	if (ctx_)
	{
		libusb_exit(ctx_);
		ctx_ = nullptr;
	}
	jtag_interface_ = 0;
	read_ep_ = 0;
	write_ep_ = 0;
}

std::vector<std::string> EspUsbJtagTransport::listDevices()
{
	std::vector<std::string> serials;
	if (!ensureContext())
		return serials;

	libusb_device** list = nullptr;
	const ssize_t count = libusb_get_device_list(ctx_, &list);
	for (ssize_t i = 0; i < count; ++i)
	{
		libusb_device_descriptor desc{};
		if (libusb_get_device_descriptor(list[i], &desc) != 0)
			continue;
		if (desc.idVendor != kVid || desc.idProduct != kPid)
			continue;

		libusb_device_handle* dev = nullptr;
		if (libusb_open(list[i], &dev) != 0)
			continue;

		char serial[256]{};
		if (desc.iSerialNumber != 0)
			libusb_get_string_descriptor_ascii(dev, desc.iSerialNumber, reinterpret_cast<unsigned char*>(serial), sizeof(serial));

		if (serial[0] != '\0')
			serials.emplace_back(serial);
		else
			serials.emplace_back("esp-usb-jtag");

		libusb_close(dev);
	}

	libusb_free_device_list(list, 1);
	return serials;
}

bool EspUsbJtagTransport::ensureConfiguration(libusb_device* dev)
{
	libusb_config_descriptor* config = nullptr;
	if (libusb_get_config_descriptor(dev, 0, &config) != 0 || config == nullptr)
	{
		last_error_ = "Failed to read USB configuration descriptor";
		return false;
	}

	int current = -1;
	const int cfg_rc = libusb_get_configuration(handle_, &current);
	if (cfg_rc != 0)
	{
		libusb_free_config_descriptor(config);
		last_error_ = std::string("libusb_get_configuration failed: ") + libusb_strerror(static_cast<libusb_error>(cfg_rc));
		return false;
	}

	if (current != static_cast<int>(config->bConfigurationValue))
	{
		const int set_rc = libusb_set_configuration(handle_, config->bConfigurationValue);
		if (set_rc != 0 && set_rc != LIBUSB_ERROR_BUSY)
		{
			libusb_free_config_descriptor(config);
			last_error_ = std::string("libusb_set_configuration failed: ") + libusb_strerror(static_cast<libusb_error>(set_rc));
			return false;
		}
	}

	libusb_free_config_descriptor(config);
	return true;
}

bool EspUsbJtagTransport::claimJtagInterface(libusb_device* dev)
{
	libusb_config_descriptor* config = nullptr;
	if (libusb_get_active_config_descriptor(dev, &config) != 0)
	{
		if (libusb_get_config_descriptor(dev, 0, &config) != 0 || config == nullptr)
		{
			last_error_ = "Failed to read active USB configuration";
			return false;
		}
	}

	read_ep_ = 0;
	write_ep_ = 0;
	jtag_interface_ = 0;

	for (int i = 0; i < config->bNumInterfaces; ++i)
	{
		const auto& alt = config->interface[i].altsetting[0];
		if (alt.bInterfaceClass != kJtagIfaceClass || alt.bInterfaceSubClass != kJtagIfaceSubclass ||
			alt.bInterfaceProtocol != kJtagIfaceProtocol)
			continue;

		unsigned in_ep = 0;
		unsigned out_ep = 0;
		for (int ep = 0; ep < alt.bNumEndpoints; ++ep)
		{
			const auto& endpoint = alt.endpoint[ep];
			if ((endpoint.bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) != LIBUSB_TRANSFER_TYPE_BULK)
				continue;
			if (endpoint.bEndpointAddress & LIBUSB_ENDPOINT_IN)
				in_ep = endpoint.bEndpointAddress;
			else
				out_ep = endpoint.bEndpointAddress;
		}

		if (in_ep == 0 || out_ep == 0)
			continue;

		detachKernelIfNeeded(handle_, alt.bInterfaceNumber);

		const int claim_rc = libusb_claim_interface(handle_, alt.bInterfaceNumber);
		if (claim_rc != 0)
		{
			last_error_ = std::string("libusb_claim_interface failed: ") + libusb_strerror(static_cast<libusb_error>(claim_rc)) +
				" (close idf.py monitor / openocd if running)";
			libusb_free_config_descriptor(config);
			return false;
		}

		jtag_interface_ = alt.bInterfaceNumber;
		read_ep_ = in_ep;
		write_ep_ = out_ep;
		libusb_free_config_descriptor(config);
		return true;
	}

	libusb_free_config_descriptor(config);
	last_error_ = "ESP USB-JTAG bulk interface not found";
	return false;
}

bool EspUsbJtagTransport::readCapsDescriptor()
{
	uint8_t caps[kJtagProtoCapsDataLen]{};
	const int caps_len = libusb_control_transfer(handle_,
		static_cast<uint8_t>(LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_STANDARD | LIBUSB_RECIPIENT_DEVICE),
		LIBUSB_REQUEST_GET_DESCRIPTOR,
		kCapsDescriptor,
		0,
		caps,
		kJtagProtoCapsDataLen,
		kLibusbTimeoutMs);

	base_speed_khz_ = 24000;
	div_min_ = 1;
	div_max_ = 255;

	if (caps_len <= 0)
		return true;

	unsigned p = kJtagBuiltinDescrStartOff;
	if (p + sizeof(JtagProtoCapsHdr) > static_cast<unsigned>(caps_len))
		return true;

	p += reinterpret_cast<JtagProtoCapsHdr*>(caps + p)->length;
	while (p + sizeof(JtagGenHdr) <= static_cast<unsigned>(caps_len))
	{
		const auto* hdr = reinterpret_cast<const JtagGenHdr*>(caps + p);
		if (hdr->type == kJtagProtoCapsSpeedApbType && hdr->length >= sizeof(JtagProtoCapsSpeedApb))
		{
			const auto* speed = reinterpret_cast<const JtagProtoCapsSpeedApb*>(caps + p);
			const uint32_t apb_10khz = static_cast<uint32_t>(speed->apb_speed_10khz[0]) | (static_cast<uint32_t>(speed->apb_speed_10khz[1]) << 8);
			base_speed_khz_ = apb_10khz * 10 / 2;
			div_min_ = static_cast<uint16_t>(speed->div_min[0]) | (static_cast<uint16_t>(speed->div_min[1]) << 8);
			div_max_ = static_cast<uint16_t>(speed->div_max[0]) | (static_cast<uint16_t>(speed->div_max[1]) << 8);
			break;
		}
		p += hdr->length;
	}
	return true;
}

bool EspUsbJtagTransport::open(const std::string& serial)
{
	close();
	last_error_.clear();

	if (!ensureContext())
		return false;

	libusb_device** list = nullptr;
	const ssize_t count = libusb_get_device_list(ctx_, &list);

	for (ssize_t i = 0; i < count; ++i)
	{
		libusb_device_descriptor desc{};
		if (libusb_get_device_descriptor(list[i], &desc) != 0)
			continue;
		if (desc.idVendor != kVid || desc.idProduct != kPid)
			continue;

		libusb_device_handle* dev = nullptr;
		const int open_rc = libusb_open(list[i], &dev);
		if (open_rc != 0)
		{
			last_error_ = std::string("libusb_open failed: ") + libusb_strerror(static_cast<libusb_error>(open_rc));
			continue;
		}

		if (!isPlaceholderSerial(serial))
		{
			char dev_serial[256]{};
			if (desc.iSerialNumber != 0)
				libusb_get_string_descriptor_ascii(dev, desc.iSerialNumber, reinterpret_cast<unsigned char*>(dev_serial), sizeof(dev_serial));
			if (serial != dev_serial)
			{
				libusb_close(dev);
				continue;
			}
		}

		handle_ = dev;
		if (!ensureConfiguration(list[i]) || !claimJtagInterface(list[i]))
		{
			libusb_close(dev);
			handle_ = nullptr;
			continue;
		}

		readCapsDescriptor();

		out_nibbles_ = 0;
		in_rd_ = in_wr_ = in_pos_bits_ = 0;
		pending_in_bits_ = 0;
		prev_cmd_ = 0;
		prev_cmd_rep_ = 0;
		std::memset(in_buf_bits_, 0, sizeof(in_buf_bits_));

		setSpeedKhz(base_speed_khz_);
		libusb_free_device_list(list, 1);
		return true;
	}

	libusb_free_device_list(list, 1);
	if (last_error_.empty())
		last_error_ = "ESP USB-JTAG device not found (303a:1001)";
	return false;
}

bool EspUsbJtagTransport::setSpeedKhz(uint32_t speed_khz)
{
	if (!handle_)
		return false;

	if (speed_khz == 0)
		speed_khz = base_speed_khz_;

	uint32_t div = base_speed_khz_ / speed_khz;
	if (div < div_min_)
		div = div_min_;
	if (div > div_max_)
		div = div_max_;

	const uint16_t div16 = static_cast<uint16_t>(div);
	return libusb_control_transfer(handle_,
		static_cast<uint8_t>(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_OUT),
		kVendJtagSetDiv,
		div16,
		jtag_interface_,
		nullptr,
		0,
		kLibusbTimeoutMs) >= 0;
}

bool EspUsbJtagTransport::sendOutBuffer()
{
	unsigned ct = out_nibbles_ / 2;
	unsigned written = 0;
	while (written < ct)
	{
		int transferred = 0;
		const int ret = libusb_bulk_transfer(handle_, static_cast<unsigned char>(write_ep_), out_buf_ + written, ct - written, &transferred, kLibusbTimeoutMs);
		if (ret != 0)
			return false;
		written += static_cast<unsigned>(transferred);
	}
	out_nibbles_ = 0;

	while (pending_in_bits_ > (kInBufSize + hw_in_fifo_len_ - 1) * 8)
	{
		if (!recvInBuffer())
			return false;
	}
	return true;
}

bool EspUsbJtagTransport::recvInBuffer()
{
	if (in_buf_bits_[in_wr_] != 0)
		return false;

	unsigned ct = (pending_in_bits_ + 7) / 8;
	if (ct > kInBufSize)
		ct = kInBufSize;
	if (ct == 0)
		return true;

	in_buf_bits_[in_wr_] = 0;
	unsigned recvd = 0;
	unsigned retries = 0;
	while (recvd < ct)
	{
		int transferred = 0;
		const int ret = libusb_bulk_transfer(handle_, static_cast<unsigned char>(read_ep_), in_buf_[in_wr_] + recvd, ct - recvd, &transferred, kLibusbTimeoutMs);
		if (ret != 0)
		{
			last_error_ = std::string("libusb bulk read failed: ") + libusb_strerror(static_cast<libusb_error>(ret));
			return false;
		}
		if (transferred == 0)
		{
			if (++retries > 10)
				return false;
			continue;
		}
		retries = 0;

		unsigned bits_in_buf = pending_in_bits_;
		if (bits_in_buf > static_cast<unsigned>(transferred) * 8)
			bits_in_buf = static_cast<unsigned>(transferred) * 8;
		pending_in_bits_ -= bits_in_buf;
		in_buf_bits_[in_wr_] += bits_in_buf;
		recvd += static_cast<unsigned>(transferred);
	}

	in_wr_ = (in_wr_ + 1) % kInBufCount;
	return true;
}

int EspUsbJtagTransport::commandAddRaw(unsigned cmd)
{
	if ((out_nibbles_ & 1u) == 0)
		out_buf_[out_nibbles_ / 2] = static_cast<uint8_t>(cmd << 4);
	else
		out_buf_[out_nibbles_ / 2] |= static_cast<uint8_t>(cmd & 0xf);
	out_nibbles_++;

	if (out_nibbles_ == kOutBufSize * 2)
	{
		if (!sendOutBuffer())
			return -1;
	}
	if (out_nibbles_ != 0 && out_nibbles_ % (kOutEpSize * 2) == 0 &&
		pending_in_bits_ > (kInBufSize + hw_in_fifo_len_ - 1) * 8)
	{
		if (!sendOutBuffer())
			return -1;
	}
	return 0;
}

int EspUsbJtagTransport::writeRleStream(unsigned cmd, int count)
{
	if (cmd == CMD_FLUSH)
		count = 1;

	if (commandAddRaw(cmd) != 0)
		return -1;
	count--;

	while (count > 0)
	{
		if (commandAddRaw(CMD_REP(count & 3)) != 0)
			return -1;
		count >>= 2;
	}
	return 0;
}

int EspUsbJtagTransport::commandAdd(unsigned cmd)
{
	if (cmd == prev_cmd_ && prev_cmd_rep_ < static_cast<int>(kCmdRepMaxReps))
	{
		prev_cmd_rep_++;
		return 0;
	}

	if (prev_cmd_rep_ > 0)
	{
		if (writeRleStream(prev_cmd_, prev_cmd_rep_) != 0)
			return -1;
	}
	prev_cmd_ = cmd;
	prev_cmd_rep_ = 1;
	return 0;
}

int EspUsbJtagTransport::out(int tms, int tdi, bool tdo_req)
{
	if (commandAdd(CMD_CLK(tdo_req, tdi, tms)) != 0)
		return -1;
	if (tdo_req)
		pending_in_bits_++;
	return 0;
}

int EspUsbJtagTransport::flush()
{
	if (prev_cmd_rep_ > 0)
	{
		if (writeRleStream(prev_cmd_, prev_cmd_rep_) != 0)
			return -1;
		prev_cmd_rep_ = 0;
	}

	if (commandAddRaw(CMD_FLUSH) != 0)
		return -1;
	if (out_nibbles_ & 1u)
	{
		if (commandAddRaw(CMD_FLUSH) != 0)
			return -1;
	}

	if (!sendOutBuffer())
	{
		last_error_ = "USB bulk write failed";
		return -1;
	}

	while (pending_in_bits_ > 0)
	{
		if (!recvInBuffer())
			return -1;
	}
	return 0;
}

int EspUsbJtagTransport::readTdoBit()
{
	if (in_rd_ == in_wr_ && in_buf_bits_[in_rd_] == 0)
		return -1;

	const int bit = (in_buf_[in_rd_][in_pos_bits_ / 8] & (1u << (in_pos_bits_ % 8))) ? 1 : 0;
	in_pos_bits_++;
	if (in_pos_bits_ == in_buf_bits_[in_rd_])
	{
		in_pos_bits_ = 0;
		in_buf_bits_[in_rd_] = 0;
		in_rd_ = (in_rd_ + 1) % kInBufCount;
	}
	return bit;
}

void EspUsbJtagTransport::clk(int tms, int tdi, bool capture_tdo)
{
	out(tms, tdi, capture_tdo);
}

void EspUsbJtagTransport::tapReset()
{
	for (int i = 0; i < 6; ++i)
		clk(1, 0, false);
	clk(1, 0, false);
	clk(0, 0, false);
	flush();
}

}  // namespace esp_probe
