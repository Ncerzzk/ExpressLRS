# ExpressLRS `stream_bridge`

This branch is a receiver-side UART-to-WiFi streaming variant based on `3.x.x-maintenance`.

The goal of this branch is narrow:

- keep normal `RX -> FC` CRSF RC output
- reuse the receiver UART as a high-rate `FC -> RX` byte input
- forward that byte stream from the receiver over WiFi TCP to a single client
- keep Web UI and OTA available on the receiver

It is not a generic upstream-ready feature branch. It is a specialized bridge profile for ESP8285 RX testing.

## What Changed

The current branch adds a dedicated TCP stream bridge on the receiver:

- TCP port `5763` is used for the raw WiFi stream
- `FC -> RX` UART input is consumed by a custom raw path in `SerialCRSF`
- received UART bytes are queued into `BinaryStreamTCP`
- the queued data is forwarded to a single TCP client on `5763`

Main files:

- `src/src/rx-serial/BinaryStreamTCP.h`
- `src/src/rx-serial/BinaryStreamTCP.cpp`
- `src/src/rx-serial/SerialCRSF.cpp`
- `src/src/rx-serial/SerialCRSF.h`
- `src/src/rx-serial/SerialIO.h`
- `src/lib/WIFI/devWIFI.cpp`
- `src/lib/FIFO/FIFO.h`
- `src/lib/Telemetry/telemetry.cpp`
- `src/src/rx_main.cpp`
- `src/python/test_tools/wifi_uart_stream_compare.py`

## Data Path

Current receiver data path:

1. FC sends bytes into the RX UART.
2. `SerialCRSF::processSerialInput()` reads UART data on ESP8266/ESP8285 using `peekBuffer()` when available.
3. `SerialCRSF::processBytes()` forwards incoming bytes into `BinaryStreamTCP` only when a TCP client is connected.
4. `BinaryStreamTCP` stores data in a heap FIFO and pushes it out through AsyncTCP on port `5763`.

Important behavior:

- `RX -> FC` CRSF RC output is still handled by the normal serial output path.
- `FC -> RX` input is treated as a raw stream for the bridge path.
- This branch is built around a single-client assumption.
- The stream FIFO lives on heap and is allocated once, then kept for the service lifetime.

## ESP8285 Minimal Bridge Profile

For `TARGET_RX + PLATFORM_ESP8266`, this branch enables a reduced WiFi runtime profile:

- keep Web UI
- keep OTA update
- keep `5763` bridge
- disable old `5762` TCP serial bridge path
- disable `wifi2tcp` MSP-over-WiFi
- disable mDNS
- disable captive DNS portal

This is done to reduce runtime pressure and leave more RAM and CPU budget for the UART stream bridge.

Related logic is mainly in:

- `src/lib/WIFI/devWIFI.cpp`
- `src/lib/Telemetry/telemetry.cpp`
- `src/src/rx_main.cpp`

## Runtime Diagnostics

The receiver exposes a bridge statistics endpoint:

- `http://<receiver-ip>/5763stats`

Current stats include:

- queued bytes
- dropped bytes
- added bytes
- peak FIFO occupancy
- send call count
- ACK callback count
- timeout callback count
- max client space
- max queued chunk
- max ACK length
- total ACK bytes
- heap information

This endpoint is used to distinguish:

- UART/front-end loss
- local FIFO overflow
- TCP send-side starvation

## UART and WiFi Settings Used During Testing

The tested setup on this branch used:

- target: `Unified_ESP8285_2400_RX_via_WIFI`
- hardware target JSON entry: `generic.rx_2400.plain`
- receiver UART baud: typically `921600` for the best result
- WiFi bridge port: `5763`

Notes:

- `src/user_defines.txt` is intentionally not documented here with personal values such as SSID, password, UID, or bind phrase.
- This branch has been exercised with custom local build settings for ESP8285 throughput tuning.

## Throughput-Related Implementation Notes

The current bridge implementation depends on the following design choices:

- ESP8285 UART input path uses a dedicated `SerialCRSF` fast path
- `BinaryStreamTCP` uses an `8192`-byte FIFO on heap
- send path is ACK-driven
- AsyncTCP is used for the bridge server
- `WiFi.setSleepMode(WIFI_NONE_SLEEP)` is retained for ESP8266 bridge operation
- ESP8266 serial RX buffer is explicitly set to `1024`

This branch also relies on local throughput-oriented network behavior during testing:

- higher-bandwidth lwIP configuration
- larger TCP send buffer / queue sizing
- AsyncTCP behavior tuned for partial ACK progression

Those changes may live outside the git tree in local framework or library state. If you need to reproduce the same result, verify your local PlatformIO packages and AsyncTCP behavior instead of assuming upstream defaults are identical.

## Test Script

The host-side verification tool for this branch is:

- `src/python/test_tools/wifi_uart_stream_compare.py`

It can:

- send deterministic payload over USB-TTL into the RX UART
- connect to the RX WiFi bridge on `5763`
- validate raw byte equality or application-frame behavior
- run both short tests and long soak tests

Typical form:

```bash
cd src
python3 python/test_tools/wifi_uart_stream_compare.py \
  --serial-port /dev/ttyACM0 \
  --baud 921600 \
  --host 192.168.3.103 \
  --port 5763 \
  --mode app \
  --frame-payload-size 64 \
  --send-rate-bytes 25600 \
  --total-bytes 1843200 \
  --post-send-timeout 30
```

## Measured Status

These are branch-specific measurements from the tested ESP8285 setup, not a general ExpressLRS guarantee.

Observed behavior:

- `921600` baud is the best tested UART setting on the current hardware path
- `460800` and `420000` were worse on the tested USB-TTL path
- `25KB/s` sustained for about one minute was measured with `0` frame count loss
- `30KB/s` was close, but repeated long tests still showed small residual end-of-run loss on some runs
- `35KB/s` was not stable

So the current branch status is:

- good at `25KB/s` sustained
- near the target at `30KB/s`
- not yet a guaranteed `30KB/s` finite-transfer `0`-loss solution on every run

If the stream is continuous and the application can tolerate a small end-of-test drain artifact, `30KB/s` may still be usable in practice. For strict finite-transfer accounting, the branch still needs more work.

## What Was Intentionally Removed

This branch no longer carries the older `TCP Serial` receiver protocol path that used:

- `PROTOCOL_TCP_SERIAL`
- Web UI serial-protocol option for `TCP Serial`
- old `SerialTCP`-driven `5762` workflow

The current branch is focused on the new `5763` stream bridge only.

## Build and OTA Notes

Build from `src/` using the normal PlatformIO environment:

```bash
cd src
pio run -e Unified_ESP8285_2400_RX_via_WIFI
```

For unified OTA packaging and upload, use the helper flow already used in this repo:

```bash
python3 /home/ncer/.codex/skills/expresslrs-unified-ota-json/scripts/package_and_upload.py \
  --repo /home/ncer/ExpressLRS/src \
  --pio /home/ncer/.platformio/penv/bin/pio \
  --env Unified_ESP8285_2400_RX_via_WIFI \
  --target generic.rx_2400.plain \
  --ip <receiver-ip>
```

## Scope

This README describes the current `stream_bridge` branch only. It is not a general description of upstream ExpressLRS features.
