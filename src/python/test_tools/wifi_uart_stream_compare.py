#!/usr/bin/env python3

import argparse
import binascii
import random
import socket
import sys
import threading
import time
from dataclasses import dataclass
from urllib.parse import urlencode
from urllib.request import urlopen, Request

import serial


DEFAULT_TOTAL_BYTES = 256 * 1024
DEFAULT_SERIAL_CHUNK_SIZE = 1024
DEFAULT_SOCKET_CHUNK_SIZE = 4096
DEFAULT_UDP_PORT = 5764
DEFAULT_UDP_HELLO_COUNT = 5
UDP_READY_REPLY = b"ELRS-UDP-READY"
APP_FRAME_MAGIC = b"ELRS"
APP_FRAME_HEADER_SIZE = 12
APP_FRAME_CRC_SIZE = 4
DEFAULT_APP_PAYLOAD_SIZE = 256


@dataclass
class CompareResult:
    match: bool
    mismatch_offset: int | None = None
    expected_byte: int | None = None
    actual_byte: int | None = None


@dataclass
class AppFrame:
    seq: int
    payload: bytes


@dataclass
class AppParseStats:
    total_frames: int = 0
    valid_frames: int = 0
    crc_failures: int = 0
    truncated_frames: int = 0
    sync_losses: int = 0
    valid_payload_bytes: int = 0
    duplicates: int = 0
    out_of_order: int = 0
    missing_frames: int = 0
    missing_runs: list[int] | None = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Send a deterministic high-volume raw byte stream into an ExpressLRS RX UART and "
            "evaluate how the RX WiFi TCP server forwards the stream."
        )
    )
    parser.add_argument("--host", default="10.0.0.1", help="ExpressLRS RX WiFi IP. Default: %(default)s")
    parser.add_argument("--port", type=int, default=0, help="ExpressLRS RX port. Default depends on transport.")
    parser.add_argument(
        "--transport",
        choices=("tcp", "udp"),
        default="tcp",
        help="WiFi transport to receive from. Default: %(default)s",
    )
    parser.add_argument(
        "--udp-hello-count",
        type=int,
        default=DEFAULT_UDP_HELLO_COUNT,
        help="Number of UDP registration datagrams to send before streaming. Default: %(default)s",
    )
    parser.add_argument(
        "--udp-local-port",
        type=int,
        default=40064,
        help="Local UDP port to bind in UDP mode. Default: %(default)s",
    )
    parser.add_argument(
        "--serial-port",
        required=True,
        help="USB-TTL serial device connected to the RX, e.g. /dev/ttyUSB0 or COM5",
    )
    parser.add_argument("--baud", type=int, default=420000, help="Serial baud rate. Default: %(default)s")
    parser.add_argument(
        "--mode",
        choices=("raw", "app"),
        default="app",
        help="Validation mode. 'raw' requires byte-for-byte equality; 'app' uses framed payload stats. Default: %(default)s",
    )
    parser.add_argument(
        "--total-bytes",
        type=int,
        default=DEFAULT_TOTAL_BYTES,
        help="Total payload bytes to generate. Default: %(default)s",
    )
    parser.add_argument(
        "--serial-chunk-size",
        type=int,
        default=DEFAULT_SERIAL_CHUNK_SIZE,
        help="Chunk size used for each serial write. Default: %(default)s",
    )
    parser.add_argument(
        "--socket-chunk-size",
        type=int,
        default=DEFAULT_SOCKET_CHUNK_SIZE,
        help="Chunk size used for each socket recv. Default: %(default)s",
    )
    parser.add_argument(
        "--seed",
        type=lambda value: int(value, 0),
        default=0xE1E5CAFE,
        help="PRNG seed used to generate deterministic payload bytes. Default: %(default)#x",
    )
    parser.add_argument(
        "--send-rate-bytes",
        type=int,
        default=0,
        help="Optional sender rate limit in bytes/s. 0 disables pacing. Default: %(default)s",
    )
    parser.add_argument(
        "--pre-delay",
        type=float,
        default=0.5,
        help="Seconds to wait after connecting before draining stale data. Default: %(default)s",
    )
    parser.add_argument(
        "--post-send-timeout",
        type=float,
        default=5.0,
        help="Seconds to keep receiving after the last serial byte is sent. Default: %(default)s",
    )
    parser.add_argument(
        "--connect-timeout",
        type=float,
        default=3.0,
        help="TCP connect timeout in seconds. Default: %(default)s",
    )
    parser.add_argument(
        "--serial-timeout",
        type=float,
        default=0.1,
        help="Serial read timeout in seconds. Default: %(default)s",
    )
    parser.add_argument(
        "--require-full-match",
        action="store_true",
        help="Fail if the TCP side receives extra trailing bytes after the expected stream.",
    )
    parser.add_argument(
        "--frame-payload-size",
        type=int,
        default=DEFAULT_APP_PAYLOAD_SIZE,
        help="Payload bytes per application frame in app mode. Default: %(default)s",
    )
    return parser.parse_args()


def log(message: str) -> None:
    print(message, flush=True)


def drain_serial(ser: serial.Serial) -> int:
    total = 0
    while True:
        waiting = ser.in_waiting
        if waiting <= 0:
            return total
        total += len(ser.read(waiting))


def drain_socket(sock: socket.socket) -> int:
    total = 0
    sock.setblocking(False)
    try:
        while True:
            chunk = sock.recv(4096)
            if not chunk:
                break
            total += len(chunk)
    except BlockingIOError:
        pass
    finally:
        sock.setblocking(True)
    return total


def drain_udp_socket(sock: socket.socket) -> int:
    total = 0
    sock.setblocking(False)
    try:
        while True:
            chunk, _ = sock.recvfrom(4096)
            if not chunk:
                break
            total += len(chunk)
    except BlockingIOError:
        pass
    finally:
        sock.setblocking(True)
    return total


def wait_for_udp_ready(sock: socket.socket, host: str, port: int, hello_count: int, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for _ in range(hello_count):
            sock.sendto(b"ELRS-UDP-HELLO", (host, port))
            time.sleep(0.02)
        try:
            while True:
                chunk, _ = sock.recvfrom(256)
                if chunk == UDP_READY_REPLY:
                    return
        except socket.timeout:
            pass
    raise TimeoutError("UDP peer did not acknowledge readiness")


def build_test_stream(total_bytes: int, seed: int) -> bytes:
    if total_bytes <= 0:
        raise ValueError("total_bytes must be > 0")

    rng = random.Random(seed)
    return bytes(rng.getrandbits(8) for _ in range(total_bytes))


def split_payload_into_frames(payload: bytes, frame_payload_size: int) -> list[AppFrame]:
    if frame_payload_size <= 0:
        raise ValueError("frame_payload_size must be > 0")

    frames = []
    seq = 0
    offset = 0
    while offset < len(payload):
        chunk = payload[offset : offset + frame_payload_size]
        frames.append(AppFrame(seq=seq, payload=chunk))
        seq += 1
        offset += len(chunk)
    return frames


def encode_app_frames(frames: list[AppFrame]) -> bytes:
    encoded = bytearray()
    for frame in frames:
        payload_len = len(frame.payload)
        header = (
            APP_FRAME_MAGIC
            + frame.seq.to_bytes(4, "big")
            + payload_len.to_bytes(2, "big")
            + b"\x00\x00"
        )
        crc = binascii.crc32(header + frame.payload) & 0xFFFFFFFF
        encoded.extend(header)
        encoded.extend(frame.payload)
        encoded.extend(crc.to_bytes(4, "big"))
    return bytes(encoded)


def compare_bytes(expected: bytes, actual: bytes) -> CompareResult:
    common = min(len(expected), len(actual))
    for idx in range(common):
        if expected[idx] != actual[idx]:
            return CompareResult(
                match=False,
                mismatch_offset=idx,
                expected_byte=expected[idx],
                actual_byte=actual[idx],
            )

    if len(expected) != len(actual):
        return CompareResult(
            match=False,
            mismatch_offset=common,
            expected_byte=expected[common] if common < len(expected) else None,
            actual_byte=actual[common] if common < len(actual) else None,
        )

    return CompareResult(match=True)


def parse_app_frames(data: bytes) -> AppParseStats:
    stats = AppParseStats()
    stats.missing_runs = []
    offset = 0
    valid_seqs: list[int] = []

    while offset < len(data):
        magic_pos = data.find(APP_FRAME_MAGIC, offset)
        if magic_pos < 0:
            break
        if magic_pos > offset:
            stats.sync_losses += 1

        if magic_pos + APP_FRAME_HEADER_SIZE > len(data):
            stats.truncated_frames += 1
            break

        seq = int.from_bytes(data[magic_pos + 4 : magic_pos + 8], "big")
        payload_len = int.from_bytes(data[magic_pos + 8 : magic_pos + 10], "big")
        frame_end = magic_pos + APP_FRAME_HEADER_SIZE + payload_len + APP_FRAME_CRC_SIZE
        if frame_end > len(data):
            stats.truncated_frames += 1
            break

        header = data[magic_pos : magic_pos + APP_FRAME_HEADER_SIZE]
        payload = data[magic_pos + APP_FRAME_HEADER_SIZE : magic_pos + APP_FRAME_HEADER_SIZE + payload_len]
        actual_crc = int.from_bytes(data[frame_end - APP_FRAME_CRC_SIZE : frame_end], "big")
        expected_crc = binascii.crc32(header + payload) & 0xFFFFFFFF

        stats.total_frames += 1
        if actual_crc != expected_crc:
            stats.crc_failures += 1
            offset = frame_end
            continue

        stats.valid_frames += 1
        stats.valid_payload_bytes += payload_len
        valid_seqs.append(seq)
        offset = frame_end

    if valid_seqs:
        prev = valid_seqs[0]
        for seq in valid_seqs[1:]:
            if seq == prev:
                stats.duplicates += 1
                continue
            if seq < prev:
                stats.out_of_order += 1
                continue
            gap = seq - prev - 1
            if gap > 0:
                stats.missing_frames += gap
                stats.missing_runs.append(gap)
            prev = seq

    return stats


def receiver_thread(
    sock: socket.socket,
    sink: bytearray,
    expected_len: int,
    chunk_size: int,
    stop_event: threading.Event,
    error_holder: list[BaseException],
) -> None:
    try:
        while not stop_event.is_set() and len(sink) < expected_len:
            chunk = sock.recv(chunk_size)
            if not chunk:
                break
            sink.extend(chunk)
    except BaseException as exc:  # noqa: BLE001
        error_holder.append(exc)


def udp_receiver_thread(
    sock: socket.socket,
    sink: bytearray,
    expected_len: int,
    chunk_size: int,
    stop_event: threading.Event,
    error_holder: list[BaseException],
) -> None:
    try:
        while not stop_event.is_set() and len(sink) < expected_len:
            chunk, _ = sock.recvfrom(chunk_size)
            if not chunk:
                break
            sink.extend(chunk)
    except BaseException as exc:  # noqa: BLE001
        error_holder.append(exc)


def paced_send(ser: serial.Serial, payload: bytes, chunk_size: int, send_rate_bytes: int) -> float:
    start = time.monotonic()
    offset = 0

    while offset < len(payload):
        chunk = payload[offset : offset + chunk_size]
        ser.write(chunk)
        offset += len(chunk)

        if send_rate_bytes > 0:
            target_elapsed = offset / send_rate_bytes
            while True:
                elapsed = time.monotonic() - start
                sleep_for = target_elapsed - elapsed
                if sleep_for <= 0:
                    break
                time.sleep(min(sleep_for, 0.01))

    ser.flush()
    return time.monotonic() - start


def format_rate(byte_count: int, seconds: float) -> str:
    if seconds <= 0:
        return "n/a"
    rate = byte_count / seconds
    return f"{rate:.1f} B/s ({rate / 1024.0:.2f} KiB/s)"


def log_app_summary(
    stats: AppParseStats,
    expected_frame_count: int,
    expected_payload_bytes: int,
    receive_seconds: float,
) -> int:
    received_frame_loss = max(0, expected_frame_count - stats.total_frames)
    valid_frame_loss = max(0, expected_frame_count - stats.valid_frames)
    payload_rate = format_rate(stats.valid_payload_bytes, max(receive_seconds, 1e-6))
    log(f"App frames expected: {expected_frame_count}")
    log(f"App frames received: {stats.total_frames}")
    log(f"App frames valid:    {stats.valid_frames}")
    log(f"CRC failures:        {stats.crc_failures}")
    log(f"Sync losses:         {stats.sync_losses}")
    log(f"Truncated frames:    {stats.truncated_frames}")
    log(f"Duplicates:          {stats.duplicates}")
    log(f"Out of order:        {stats.out_of_order}")
    log(f"Valid payload bytes: {stats.valid_payload_bytes}/{expected_payload_bytes}")
    log(f"Valid payload rate:  {payload_rate}")
    log(f"Frame count loss:    {received_frame_loss}")
    log(f"Valid frame loss:    {valid_frame_loss}")
    log(f"Seq missing frames:  {stats.missing_frames}")
    runs = stats.missing_runs or []
    log(f"Missing runs:        {len(runs)}")
    log(f"Max burst:           {max(runs) if runs else 0}")
    if runs:
        log(f"First 20 runs:       {runs[:20]}")

    if stats.valid_frames == 0:
        log("[FAIL] No valid application frames received")
        return 1

    log("[PASS] Application-layer statistics collected")
    return 0


def main() -> int:
    args = parse_args()
    expected_payload = build_test_stream(args.total_bytes, args.seed)
    app_frames: list[AppFrame] = []
    if args.mode == "app":
        app_frames = split_payload_into_frames(expected_payload, args.frame_payload_size)
        serial_payload = encode_app_frames(app_frames)
    else:
        serial_payload = expected_payload

    log(f"Prepared deterministic payload: {len(expected_payload)} bytes")
    if args.mode == "app":
        log(f"Application frames: {len(app_frames)} x <= {args.frame_payload_size} bytes")
    log(f"Serial: {args.serial_port} @ {args.baud}")
    port = args.port or (DEFAULT_UDP_PORT if args.transport == "udp" else 5763)
    log(f"{args.transport.upper()}: {args.host}:{port}")
    log(f"Mode: {args.mode}")

    ser = serial.Serial(port=args.serial_port, baudrate=args.baud, timeout=args.serial_timeout)
    if args.transport == "tcp":
        sock = socket.create_connection((args.host, port), timeout=args.connect_timeout)
        sock.settimeout(0.5)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    else:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
        sock.bind(("", args.udp_local_port))
        sock.settimeout(0.5)
        peer_port = sock.getsockname()[1]
        body = urlencode({"port": str(peer_port)}).encode()
        req = Request(f"http://{args.host}/5764peer", data=body, method="POST")
        try:
            with urlopen(req, timeout=args.connect_timeout) as resp:
                if resp.status != 200:
                    raise RuntimeError(f"UDP peer register failed: HTTP {resp.status}")
        except Exception:
            pass
        wait_for_udp_ready(sock, args.host, port, args.udp_hello_count, args.connect_timeout)

    received = bytearray()
    stop_event = threading.Event()
    receiver_errors: list[BaseException] = []

    try:
        time.sleep(args.pre_delay)
        drained_serial = drain_serial(ser)
        drained_socket = drain_socket(sock) if args.transport == "tcp" else drain_udp_socket(sock)
        if drained_serial or drained_socket:
            log(f"Drained stale data: serial={drained_serial} bytes net={drained_socket} bytes")

        receiver_target = receiver_thread if args.transport == "tcp" else udp_receiver_thread
        thread = threading.Thread(
            target=receiver_target,
            args=(sock, received, len(serial_payload), args.socket_chunk_size, stop_event, receiver_errors),
            daemon=True,
        )
        thread.start()

        log("Sending test stream over UART ...")
        send_seconds = paced_send(ser, serial_payload, args.serial_chunk_size, args.send_rate_bytes)
        log(f"UART send done in {send_seconds:.3f}s, rate={format_rate(len(serial_payload), send_seconds)}")

        deadline = time.monotonic() + args.post_send_timeout
        while len(received) < len(serial_payload) and time.monotonic() < deadline:
            if receiver_errors:
                raise receiver_errors[0]
            time.sleep(0.02)

        stop_event.set()
        thread.join(timeout=1.0)
        if receiver_errors:
            raise receiver_errors[0]

        receive_seconds = send_seconds + max(0.0, args.post_send_timeout - max(0.0, deadline - time.monotonic()))
        log(f"{args.transport.upper()} received {len(received)} bytes")
        if received:
            log(f"Observed receive rate={format_rate(len(received), max(receive_seconds, 1e-6))}")

        if args.require_full_match and args.transport == "tcp":
            sock.settimeout(0.2)
            try:
                extra = sock.recv(args.socket_chunk_size)
            except socket.timeout:
                extra = b""
            if extra:
                received.extend(extra)
                log(f"TCP delivered extra trailing bytes: {len(extra)}")

        if args.mode == "app":
            stats = parse_app_frames(bytes(received))
            return log_app_summary(stats, len(app_frames), len(expected_payload), receive_seconds)

        result = compare_bytes(expected_payload, bytes(received))
        if result.match:
            log(f"[PASS] UART -> {args.transport.upper()} byte stream matched exactly")
            return 0

        log(f"[FAIL] UART -> {args.transport.upper()} mismatch detected")
        log(f"Expected bytes: {len(expected_payload)}")
        log(f"Actual bytes:   {len(received)}")
        if result.mismatch_offset is not None:
            log(f"First mismatch offset: {result.mismatch_offset}")
            if result.expected_byte is not None:
                log(f"Expected byte: 0x{result.expected_byte:02x}")
            else:
                log("Expected byte: <end-of-stream>")
            if result.actual_byte is not None:
                log(f"Actual byte:   0x{result.actual_byte:02x}")
            else:
                log("Actual byte:   <end-of-stream>")

            start = max(0, result.mismatch_offset - 16)
            end = min(len(expected_payload), result.mismatch_offset + 16)
            log(f"Expected slice[{start}:{end}]: {expected_payload[start:end].hex()}")
            log(f"Actual   slice[{start}:{min(len(received), result.mismatch_offset + 16)}]: "
                f"{bytes(received[start:min(len(received), result.mismatch_offset + 16)]).hex()}")
        return 1
    finally:
        stop_event.set()
        try:
            sock.close()
        finally:
            ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
