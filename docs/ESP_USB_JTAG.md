# ESP USB-JTAG (Variable Viewer)

MCUViewer can sample global variables on Espressif RISC-V chips (ESP32-C3/C6/H2/P4) through the built-in USB-JTAG debug interface (`303a:1001`). No OpenOCD or external probe is required for Variable Viewer.

Trace Viewer (SWO) is not supported on ESP32 — use ST-Link/J-Link targets for SWO.

## Supported targets

| Chip     | IDCODE     | TAPs | Notes              |
|----------|------------|------|--------------------|
| esp32c6  | `0x0000dc25` | 1  | Default in GUI     |
| esp32c3  | `0x00005c25` | 1  |                    |
| esp32h2  | `0x0000c825` | 1  |                    |
| esp32p4  | `0x00012c25` | 2  | HP core on TAP 1   |

Xtensa chips (ESP32, ESP32-S2/S3) are not implemented yet.

## Requirements

### Linux

- `libusb-1.0-0-dev`
- `riscv32-esp-elf-gdb` (or compatible GDB 12.1+) for ELF parsing only
- User in the `plugdev` group (or udev rules for `303a:1001`)
- Firmware running on the target (MCUViewer reads RAM while the CPU runs)

### Host conflicts

Only one program may own the USB-JTAG interface at a time. Close before starting MCUViewer:

- `idf.py monitor`
- OpenOCD
- Another MCUViewer instance

After stopping acquisition, MCUViewer resumes the core and clears the debug module so `idf.py monitor` works again. If the serial port (`/dev/ttyACM0`) is missing, unplug and replug USB.

## Quick start

1. Build and flash your ESP-IDF project in **debug** mode (`idf.py build flash`).
2. **Options → Acquisition**
   - Probe: **ESP_USB_JTAG**
   - Chip target: match your SoC (e.g. `esp32c6`)
   - JTAG speed: `24000` kHz (default)
   - ELF: path to your `build/<project>.elf`
   - GDB: `riscv32-esp-elf-gdb`
3. Import variables by symbol name and **Update variable addresses**.
4. Drag variables to a plot and press **START**.

Close `idf.py monitor` before starting acquisition.

## Firmware constraints

Variable Viewer reads RAM directly via the debug bus. Variables must meet the same rules as for ST-Link/J-Link:

- **Global** scope (file scope or `static` globals) — stack locals and heap objects are not stable
- Address fixed for the lifetime of the firmware image (rebuild + update ELF if layout changes)
- Debug symbols present in the ELF (`-g`, no strip)

To find a symbol address from your build:

```bash
riscv32-esp-elf-nm build/<project>.elf | grep my_variable
# or
riscv32-esp-elf-gdb build/<project>.elf -ex "print &my_variable" -ex quit
```

## Hardware smoke test (optional)

Validates USB-JTAG + DMI + memory read without the GUI. Pass a 32-bit address from your ELF map:

```bash
cd build
cmake .. && cmake --build . -j$(nproc)   # if not built yet

g++ -std=c++20 -O0 \
  -Isrc/EspProbe -Isrc/EspProbe/usb -Isrc/EspProbe/jtag -Isrc/EspProbe/riscv \
  tools/esp_probe_smoke.cpp \
  src/EspProbe/usb/EspUsbJtagTransport.cpp \
  src/EspProbe/jtag/EspJtagTap.cpp \
  src/EspProbe/riscv/EspRiscvDm.cpp \
  src/EspProbe/EspProbeSession.cpp \
  -lusb-1.0 -o /tmp/esp_probe_smoke

/tmp/esp_probe_smoke esp32c6 24000 0x4080xxxx
```

Arguments: `[chip] [speed_khz] [address_hex]`. Defaults: `esp32c6`, `24000`, address required.

## Architecture

```
EspUsbJtagDebugProbe (IDebugProbe)
  └── EspProbeSession
        ├── EspUsbJtagTransport   USB bulk protocol (OpenOCD esp_usb_jtag.c)
        ├── EspJtagTap            IR/DR scans
        └── EspRiscvDm            DMI + System Bus Access (memory read/write)
```

Protocol and TAP sequencing are adapted from [openocd-esp32](https://github.com/espressif/openocd-esp32); MCUViewer does not shell out to OpenOCD.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|----------------|-----|
| `libusb_claim_interface failed` | Monitor/OpenOCD holding USB | Close other tools |
| `RISC-V debug module init failed` | Wrong chip profile or USB busy | Match chip target; replug USB |
| Bootloader OK, no app logs after MCUViewer | Core left halted (old builds) | Replug USB or `openocd … -c "init; reset run; shutdown"` |
| `/dev/ttyACM0` missing | Kernel driver detached from CDC | Replug USB; use current build with graceful shutdown |
| Variables `NOT FOUND` | Wrong ELF or non-global vars | Rebuild debug ELF; globals only |
| Flat line at zero | Acquisition stopped or wrong address | START acquisition; update addresses |

## Upstream note

This backend targets the open-source MCUViewer 1.1.0 tree. Windows support for ESP USB-JTAG is not wired in CMake yet (Linux-only path).
