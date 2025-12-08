# LEDRX - ESP32 WS2811 LED Controller

MQTT-controlled WS2811 LED strip driver for ESP32, supporting parallel output channels for synchronized LED control.

## Hardware Requirements

- ESP32-WROOM module
- WS2811 LED strips
- USB serial adapter for flashing/debugging

## Development Environment Setup

### Prerequisites

This project requires ESP-IDF v5.5.1. The IDF should be installed at `~/esp-idf`.

### Initial Setup

1. Clone and install ESP-IDF v5.5.1:
```bash
cd ~
git clone -b v5.5.1 --recursive https://github.com/espressif/esp-idf.git esp-idf
cd esp-idf
./install.sh esp32
```

2. Source the ESP-IDF environment (add to `~/.bashrc` for persistence):
```bash
export IDF_PATH="${HOME}/esp-idf"
source "${IDF_PATH}/export.sh"
```

3. Clone this repository:
```bash
git clone <repository-url> ledrx
cd ledrx
```

## Configuration

### Project Configuration

Configure the project using the menuconfig tool:

```bash
idf.py menuconfig
```

### Key Configuration Settings

#### Serial Flasher Config
**Location:** `Serial flasher config` (top-level menu)

Critical settings for ESP32-WROOM:
- **Flash SPI mode:** `DIO` (default, safest for most ESP32-WROOM modules)
- **Flash SPI speed:** `40 MHz` (stable) or `80 MHz` (faster, test stability)
- **Flash size:** `4 MB` (verify your module's flash size)
- **Detect flash size:** `Enabled` (recommended)
- **Before flashing:** `Reset to bootloader`
- **After flashing:** `Reset after flashing`
- **Monitor baud rate:** `115200` (default) or `921600` (faster logs)

#### WiFi Configuration
**Location:** `Component config → WiFi Configuration`

Set your WiFi credentials in `sdkconfig` or via menuconfig.

#### MQTT Configuration
**Location:** `Component config → MQTT Configuration`

Configure MQTT broker settings:
- Broker URI
- Username/Password (if required)
- Topics for LED control

**Note:** MQTT credentials are stored in `sdkconfig`, which is in `.gitignore` to prevent credential leaks.

#### CPU Performance
**Location:** `Component config → ESP System Settings → CPU frequency`

Current setting: **240 MHz** (maximum performance)
- Use 240 MHz for best LED update performance
- Lower frequencies (160/80 MHz) reduce power consumption

#### Build Optimization
**Location:** `Compiler options → Optimization Level`

Current setting: **Debug (-Og)** - suitable for development
- **Release (-O2):** Maximum performance, harder to debug
- **Size (-Os):** Minimize binary size (can save 50-100 KB)
- **Debug (-Og):** Easier debugging, larger binary

## Building

### Full Build

```bash
idf.py build
```

### Clean Build

```bash
idf.py fullclean
idf.py build
```

### Build Output

- Binary location: `build/ledrx.bin`
- Map file: `build/ledrx.map`
- Current binary size: ~903 KB (1 MB partition, ~145 KB free)

## Flashing

### Automatic Flash and Monitor

```bash
idf.py flash monitor
```

### Flash Only

```bash
idf.py flash
```

### Flash to Specific Port

```bash
idf.py -p /dev/ttyUSB0 flash
```

### Common Flash Issues

**"Failed to connect":**
- Hold BOOT button while connecting
- Check USB cable and driver
- Verify correct port with `ls /dev/tty*`

**"Wrong boot mode detected":**
- ESP32 is not in bootloader mode
- Press and hold BOOT, press RESET, release RESET, release BOOT

**"Flash size mismatch":**
- Update flash size in menuconfig under `Serial flasher config → Flash size`

## Monitoring and Debugging

### Serial Monitor

```bash
idf.py monitor
```

Exit monitor: `Ctrl+]`

### Monitor with Custom Baud Rate

```bash
idf.py -b 921600 monitor
```

### Useful Monitor Filters

```bash
# Filter by log level (ERROR only)
idf.py monitor --print-filter "E:*"

# Filter by tag
idf.py monitor --print-filter "WS2811:V"

# Multiple filters
idf.py monitor --print-filter "WS2811:V LED:D MQTT:I"
```

### GDB Debugging

Start GDB debugging session:

```bash
idf.py gdb
```

For JTAG debugging, configure OpenOCD:

```bash
idf.py openocd
# In another terminal:
idf.py gdb
```

### Log Levels

**Location:** `Component config → Log output → Default log verbosity`

Current setting: **Info**
- **None:** No logs (smallest binary)
- **Error:** Only errors
- **Warning:** Errors and warnings
- **Info:** General information (current)
- **Debug:** Detailed debugging
- **Verbose:** Everything (largest binary)

## Project Structure

```
ledrx/
├── main/
│   ├── app_main.c           # Application entry point
│   └── CMakeLists.txt       # Main component build config
├── components/
│   ├── led/
│   │   ├── led.c            # LED controller logic
│   │   ├── ws2811.c         # WS2811 RMT driver
│   │   ├── include/         # Public headers
│   │   └── CMakeLists.txt
│   └── wifi/
│       ├── wifi.c           # WiFi connection management
│       ├── include/
│       └── CMakeLists.txt
├── CMakeLists.txt           # Project-level build config
├── sdkconfig                # Project configuration (gitignored)
└── README.md                # This file
```

## Component Architecture

### LED Component (`components/led/`)

Handles WS2811 LED control using ESP32's RMT peripheral:
- **Multi-channel support:** Parallel output for synchronized strips
- **RMT TX encoder:** Hardware-accelerated bit timing
- **RGB color control:** Per-LED color setting

### WiFi Component (`components/wifi/`)

Manages WiFi connectivity with automatic reconnection.

### WS2811 Driver

Uses ESP-IDF 5.5.1's RMT TX encoder API:
- 10 MHz resolution for precise timing
- Configurable channels via `ws2811_init()`
- Synchronous transmission across all channels

## Common Development Tasks

### Change WiFi Credentials

```bash
idf.py menuconfig
# Navigate to WiFi configuration
# Update SSID and password
idf.py build flash
```

### Add New LED Patterns

1. Edit `components/led/led.c`
2. Implement pattern function
3. Wire up to MQTT message handler
4. Build and flash

### Adjust LED Channel Configuration

Edit `main/app_main.c`:
```c
int gpio_pins[] = {GPIO_NUM_X, GPIO_NUM_Y};  // Add/modify pins
ws2811_init(gpio_pins, sizeof(gpio_pins)/sizeof(int));
```

### Monitor Flash Usage

```bash
idf.py size
idf.py size-components  # Per-component breakdown
```

### Erase Flash Completely

```bash
idf.py erase-flash
```

## Performance Optimization

### Current Configuration
- CPU: 240 MHz
- Optimization: -Og (Debug)
- Partition: Single app (1 MB)
- Binary size: ~903 KB

### Size Optimization Options

1. **Switch to -Os optimization:**
   - Menuconfig → Compiler options → Optimization Level → Optimize for size
   - Saves ~50-100 KB

2. **Reduce log verbosity:**
   - Menuconfig → Log output → Default log verbosity → Warning
   - Saves ~20-30 KB

3. **Disable unused components:**
   - Already excluded: esp_https_ota, esp_http_server, fatfs, spiffs
   - Edit `CMakeLists.txt` to add more to `EXCLUDE_COMPONENTS`

## Troubleshooting

### Build Errors

**"Component not found":**
- Run `idf.py fullclean` and rebuild
- Verify ESP-IDF is sourced: `echo $IDF_PATH`

**"sdkconfig version mismatch":**
- Delete `sdkconfig` and `sdkconfig.old`
- Run `idf.py menuconfig` to regenerate

### Runtime Issues

**LEDs not updating:**
- Check GPIO pin configuration
- Verify WS2811 power supply
- Check RMT channel allocation in logs

**WiFi won't connect:**
- Verify credentials in sdkconfig
- Check WiFi signal strength
- Monitor logs: `idf.py monitor --print-filter "WIFI:V"`

**MQTT connection fails:**
- Verify broker is reachable
- Check credentials
- Monitor: `idf.py monitor --print-filter "MQTT:V"`

## Technical Details

### WS2811 Timing

The driver uses precise RMT timing (0.1μs resolution):
- **Bit 0:** 350ns high, 850ns low
- **Bit 1:** 750ns high, 450ns low
- **Reset:** >50μs low

### Memory Layout

- **Flash:** 4 MB total
  - App partition: 1 MB
  - OTA partition: None (single app config)
  - NVS: Default
- **RAM:** 520 KB total
  - DRAM: ~200 KB available for heap
  - IRAM: ~128 KB for interrupt handlers

## License

See LICENSE file for details.

## Contributing

This is a derived work based on ESP-IDF examples and components.
