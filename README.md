# ExpressLRS `stream_bridge`

This branch is a receiver-side UART-to-WiFi streaming variant based on `3.x.x-maintenance`.

The scope is narrow:

- keep normal `RX -> FC` CRSF RC output
- reuse the receiver UART as a high-rate `FC -> RX` byte input
- forward that byte stream from the receiver over WiFi UDP to a single client
- keep Web UI and OTA available on the receiver

This is not a generic upstream feature branch. It is a specialized ESP8285 bridge profile for local testing.

## What Changed

The current branch adds a dedicated UDP stream bridge on the receiver:

- UDP port `5764` is used for the raw WiFi stream
- `FC -> RX` UART input is consumed by a custom raw path in `SerialCRSF`
- received UART bytes are queued into `BinaryStreamUDP`
- the queued data is forwarded to a single UDP peer on `5764`

Main files:

- `src/src/rx-serial/BinaryStreamUDP.h`
- `src/src/rx-serial/BinaryStreamUDP.cpp`
- `src/src/rx-serial/SerialCRSF.cpp`
- `src/src/rx-serial/SerialCRSF.h`
- `src/src/rx-serial/SerialIO.h`
- `src/lib/WIFI/devWIFI.cpp`
- `src/lib/FIFO/FIFO.h`
- `src/src/rx_main.cpp`
- `src/python/test_tools/wifi_uart_stream_compare.py`

## Data Path

Current receiver data path:

1. FC sends bytes into the RX UART.
2. `SerialCRSF::processSerialInput()` reads UART data on ESP8266/ESP8285.
3. `SerialCRSF::processBytes()` forwards incoming bytes into `BinaryStreamUDP` only when a UDP peer is known.
4. `BinaryStreamUDP` stores data in a heap FIFO and forwards it on UDP port `5764`.

Important behavior:

- `RX -> FC` CRSF RC output is still handled by the normal serial output path.
- `FC -> RX` input is treated as a raw stream for the bridge path.
- This branch is built around a single-peer assumption.
- The UDP stream FIFO lives on heap and is lazily allocated.

## ESP8285 WiFi Profile

For `TARGET_RX + PLATFORM_ESP8266`, this branch keeps a reduced WiFi runtime profile:

- keep Web UI
- keep OTA update
- keep UDP bridge `5764`
- old `5762` TCP serial bridge is not part of the intended path
- `wifi2tcp` and other older WiFi serial workflows are not the focus of this branch

This is done to leave more RAM and CPU budget for the UART stream bridge.

## Runtime Diagnostics

The receiver exposes a UDP bridge statistics endpoint:

- `http://<receiver-ip>/5764stats`

Current stats include:

- queued bytes
- dropped bytes
- sent bytes
- peak FIFO occupancy
- send call count
- max queued chunk
- peer port

For the current UDP route, the primary receiver-side correctness metric is:

- `dropped_bytes == 0`

If `dropped_bytes` stays at `0`, the RX itself did not lose bytes locally. Any additional loss seen at the PC side is then downstream of the RX.

## UART and WiFi Settings Used During Testing

Tested setup on this branch:

- target: `Unified_ESP8285_2400_RX_via_WIFI`
- hardware target JSON entry: `generic.rx_2400.plain`
- receiver UART baud: typically `921600`
- UDP bridge port: `5764`

Notes:

- `src/user_defines.txt` is intentionally not documented here with personal SSID, password, UID, or bind phrase values.
- This branch uses custom local build settings for ESP8285 throughput tuning.

## Throughput-Related Implementation Notes

The current branch depends on:

- ESP8285 UART raw ingest path in `SerialCRSF`
- `BinaryStreamUDP` FIFO size `10240` bytes on heap
- `WiFiUDP` / `UdpContext` send path
- higher-bandwidth lwIP configuration
- local ESP8266 lwIP override artifacts vendored into this repo

Repository-local throughput overrides:

- vendored `ESPAsyncTCP` behavior
- vendored lwIP override artifacts under `src/tools/stream_bridge/lwip/`
- build flags in `src/targets/*.ini`

## UDP Peer Registration

The UDP bridge can learn its peer in two ways:

1. HTTP helper:
   - `POST /5764peer` with `port=<client-port>`
2. UDP hello:
   - send `ELRS-UDP-HELLO` to `5764`
   - RX responds with `ELRS-UDP-READY`

The host test script supports both, and falls back to UDP hello when HTTP registration is unavailable.

## Test Script

The host-side verification tool is:

- `src/python/test_tools/wifi_uart_stream_compare.py`

It can:

- send deterministic payload over USB-TTL into the RX UART
- receive from the UDP bridge on `5764`
- validate application-frame behavior
- run short tests and soak tests

Typical form:

```bash
cd src
python3 python/test_tools/wifi_uart_stream_compare.py \
  --serial-port /dev/ttyACM0 \
  --baud 921600 \
  --host 192.168.3.103 \
  --transport udp \
  --port 5764 \
  --mode app \
  --frame-payload-size 64 \
  --send-rate-bytes 30720 \
  --total-bytes 1843200
```

## Measured Status

These are branch-specific measurements from the tested ESP8285 setup, not a general ExpressLRS guarantee.

Observed behavior so far:

- `921600` is the best tested UART setting on the current USB-TTL path
- the UDP bridge is now the primary path under investigation
- RX-side short tests at `30KB/s` can maintain `dropped_bytes = 0`
- RX-side short-test limit with the current UDP path reached `50KB/s` before local `dropped_bytes` became non-zero
- PC-side application-frame loss can still appear even when RX-side `dropped_bytes = 0`

So the current branch status is:

- UDP path is working
- RX-side local loss can be held at `0` at and above the original `30KB/s` target
- end-to-end loss at the PC side still needs further tuning depending on packetization and receiver-side buffering

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
