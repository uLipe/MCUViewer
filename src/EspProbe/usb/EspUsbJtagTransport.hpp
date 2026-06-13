// SPDX-License-Identifier: GPL-3.0-or-later
// USB-JTAG transport adapted from OpenOCD esp_usb_jtag.c (Espressif, GPL-2.0-or-later).

#ifndef ESP_USB_JTAG_TRANSPORT_HPP
#define ESP_USB_JTAG_TRANSPORT_HPP

#include <cstdint>
#include <string>
#include <vector>

struct libusb_context;
struct libusb_device;
struct libusb_device_handle;

namespace esp_probe
{

class EspUsbJtagTransport
{
   public:
	EspUsbJtagTransport() = default;
	~EspUsbJtagTransport();

	EspUsbJtagTransport(const EspUsbJtagTransport&) = delete;
	EspUsbJtagTransport& operator=(const EspUsbJtagTransport&) = delete;

	bool open(const std::string& serial = {});
	void close();
	bool isOpen() const { return handle_ != nullptr; }
	const std::string& lastError() const { return last_error_; }

	bool setSpeedKhz(uint32_t speed_khz);
	std::vector<std::string> listDevices();

	void clk(int tms, int tdi, bool capture_tdo);
	int flush();
	int readTdoBit();

	void tapReset();

   private:
	static constexpr uint16_t kVid = 0x303a;
	static constexpr uint16_t kPid = 0x1001;
	static constexpr uint16_t kCapsDescriptor = 0x2000;
	static constexpr uint8_t kJtagIfaceClass = 0xff;
	static constexpr uint8_t kJtagIfaceSubclass = 0xff;
	static constexpr uint8_t kJtagIfaceProtocol = 1;
	static constexpr unsigned kOutEpSize = 64;
	static constexpr unsigned kOutBufSize = kOutEpSize * 32;
	static constexpr unsigned kInBufSize = 64;
	static constexpr unsigned kInBufCount = 8;

	libusb_context* ctx_ = nullptr;
	libusb_device_handle* handle_ = nullptr;
	uint8_t jtag_interface_ = 0;
	unsigned read_ep_ = 0;
	unsigned write_ep_ = 0;
	std::string last_error_;

	uint32_t base_speed_khz_ = 24000;
	uint16_t div_min_ = 1;
	uint16_t div_max_ = 255;

	uint8_t out_buf_[kOutBufSize]{};
	unsigned out_nibbles_ = 0;

	uint8_t in_buf_[kInBufCount][kInBufSize]{};
	unsigned in_buf_bits_[kInBufCount]{};
	unsigned in_rd_ = 0;
	unsigned in_wr_ = 0;
	unsigned in_pos_bits_ = 0;
	unsigned pending_in_bits_ = 0;

	unsigned prev_cmd_ = 0;
	int prev_cmd_rep_ = 0;
	unsigned hw_in_fifo_len_ = 4;

	bool ensureContext();
	bool ensureConfiguration(libusb_device* dev);
	bool claimJtagInterface(libusb_device* dev);
	bool readCapsDescriptor();
	bool sendOutBuffer();
	bool recvInBuffer();
	int commandAddRaw(unsigned cmd);
	int commandAdd(unsigned cmd);
	int writeRleStream(unsigned cmd, int count);
	int out(int tms, int tdi, bool tdo_req);
};

}  // namespace esp_probe

#endif
