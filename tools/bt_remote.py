#!/usr/bin/env python3
"""
===============================================================================
SHIVANSH MECHA OS • Autonomous Robotics Platform
Embedded Systems Architecture CS140 / CS107: Distributed Systems & Embedded Interfacing Real-Time Control
===============================================================================
File: bt_remote.py
Author: Pilot Shivansh & Antigravity AI Pair-Programmer
Target Architecture: macOS CoreBluetooth / Bleak Asyncio Client Engine
Platform: Kidsbits Multi-Purpose Coding Robot (Model KD0003)

Description:
  High-throughput, asynchronous Bluetooth Low Energy (BLE) interactive terminal
  remote controller. Establishes a GATT client connection to the onboard HM-10
  transceiver, negotiates notifications for real-time telemetry streaming, and
  dispatches single-byte control frames for steering, gear shifts, and emergency
  braking.

GATT Protocol Specifications:
  - Peripheral Advertising Name: "HMSoft"
  - Primary Service UUID: 0000ffe0-0000-1000-8000-00805f9b34fb
  - Read/Write/Notify Characteristic UUID: 0000ffe1-0000-1000-8000-00805f9b34fb
===============================================================================
"""

import asyncio
import sys
from typing import Optional
from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice
from bleak.backends.characteristic import BleakGATTCharacteristic

# GATT Protocol Descriptors
ROBOT_ADVERTISING_NAME: str = "HMSoft"
GATT_UART_SERVICE_UUID: str = "0000ffe0-0000-1000-8000-00805f9b34fb"
GATT_UART_CHAR_UUID: str = "0000ffe1-0000-1000-8000-00805f9b34fb"


def telemetry_rx_callback(characteristic: BleakGATTCharacteristic, data: bytearray) -> None:
    """
    Handles asynchronous telemetry packets relayed from the robot's ATmega328P.
    
    Parameters:
        characteristic: The GATT characteristic emitting the notification.
        data: Raw byte payload streamed from the microcontroller UART.
    """
    decoded_text = data.decode("utf-8", errors="ignore").strip()
    if decoded_text:
        print(f"\n[Telemetry RX]: {decoded_text}")


async def discover_robot_device(timeout_seconds: float = 6.0) -> Optional[BLEDevice]:
    """
    Scans the local 2.4 GHz radio band for HM-10 BLE advertising packets.
    
    Parameters:
        timeout_seconds: Scan discovery window in seconds.
    Returns:
        BLEDevice object if discovered, None otherwise.
    """
    print(f"[*] Scanning local RF environment for '{ROBOT_ADVERTISING_NAME}' ({timeout_seconds}s)...")
    device = await BleakScanner.find_device_by_filter(
        lambda d, adv: adv.local_name == ROBOT_ADVERTISING_NAME or (d.name and ROBOT_ADVERTISING_NAME in d.name),
        timeout=timeout_seconds
    )
    return device


def normalize_and_validate_command(raw: str) -> Optional[bytes]:
    """
    Validates user console input and frames multi-byte payloads with newline.
    Rejects arbitrary raw strings (e.g. 'hello') to protect robot state machine.
    """
    cmd = raw.strip()
    if not cmd:
        return None

    # Single-byte commands
    if len(cmd) == 1:
        c = cmd.upper()
        # Motion & Emergency Brake
        if c in {"S", "F", "B", "L", "R"}:
            return c.encode("ascii")
        # Gears (lowercase q, w, e)
        if cmd in {"q", "w", "e"}:
            return cmd.encode("ascii")
        # Audio Mute / Unmute
        if cmd in {"x", "X"}:
            return cmd.encode("ascii")
        # Autonomous Modes 0-9
        if cmd in "0123456789":
            return cmd.encode("ascii")
        # Joy Greeting, Dances, Expressions & Diagnostics
        if c in {"G", "K", "U", "H", "Z", "A", "Y", "D", "C", "?", "!", "*"}:
            return c.encode("ascii")
        return None

    # Multi-byte commands (must be properly framed with newline \n)
    prefix = cmd[0].upper()
    rest = cmd[1:].strip()

    # Throttle: P <pwm> or P<pwm> (0-255)
    if prefix == "P":
        if rest.isdigit() and 0 <= int(rest) <= 255:
            return f"P{rest}\n".encode("ascii")
        return None

    # Jukebox: J <1-4> or J<1-4>
    if prefix == "J":
        if rest in {"1", "2", "3", "4"}:
            return f"J{rest}\n".encode("ascii")
        return None

    # Text Banner: W <text>
    if prefix == "W":
        clean_text = "".join(ch for ch in rest if 32 <= ord(ch) <= 126)
        if clean_text:
            return f"W{clean_text}\n".encode("ascii")
        return None

    # Clock / Focus Banner: @ <text>
    if cmd[0] == "@":
        clean_text = "".join(ch for ch in rest if 32 <= ord(ch) <= 126)
        if clean_text:
            return f"@{clean_text}\n".encode("ascii")
        return None

    # Quiz Symbol: T <symbol>
    if prefix == "T":
        if len(rest) >= 1:
            clean_sym = rest[0].upper()
            return f"T{clean_sym}\n".encode("ascii")
        return None

    return None


async def create_async_stdin_reader() -> asyncio.StreamReader:
    """
    Creates a non-blocking asynchronous StreamReader connected to sys.stdin.
    Avoids thread executor blocking and allows instant cancellation.
    """
    reader = asyncio.StreamReader()
    protocol = asyncio.StreamReaderProtocol(reader)
    loop = asyncio.get_running_loop()
    await loop.connect_read_pipe(lambda: protocol, sys.stdin)
    return reader


async def run_controller_session() -> None:
    """
    Main asynchronous event loop managing the BLE GATT lifecycle and keyboard dispatch.
    """
    print("========================================================")
    print(" SHIVANSH MECHA OS: Kidsbits Robot Mac Terminal Cockpit       ")
    print(" Pilot: Shivansh | Protocol: BLE GATT 9600 Baud         ")
    print("========================================================")

    device = await discover_robot_device()
    if not device:
        print(f"[-] Fatal: Could not locate '{ROBOT_ADVERTISING_NAME}'. Ensure battery & BT switches are ON.")
        return

    print(f"[+] Device Located: {device.name} [{device.address}]")
    print("[*] Initiating GATT handshake...")

    async with BleakClient(device) as client:
        print(f"[✔] GATT Link Established! (Connected: {client.is_connected})")
        print("-" * 58)
        print(" COMMAND DISPATCH MATRIX:")
        print("   q / w / e     : Shift Gear (1: Precision | 2: Cruise | 3: Turbo)")
        print("   F / B / L / R : Drive (Forward | Reverse | Left | Right)")
        print("   S             : EMERGENCY BRAKE (0 PWM)")
        print("   P <0-255>     : Set Throttle PWM (e.g. 'P 150')")
        print("   J <1-4>       : Play Melody (1: Star Wars | 2: R2 | 3: Mario | 4: Spy)")
        print("   W <text>      : Scroll Text Banner on Matrix Face")
        print("   0 - 9         : Autonomous Modes (1: Obstacle | 2: Line | 5: Pet)")
        print("   exit / quit   : Disconnect & Safe Halt")
        print("-" * 58)

        # Subscribe to UART notifications
        await client.start_notify(GATT_UART_CHAR_UUID, telemetry_rx_callback)
        stdin_reader = await create_async_stdin_reader()

        try:
            while True:
                sys.stdout.write("\nCockpit Command > ")
                sys.stdout.flush()

                line_bytes = await stdin_reader.readline()
                if not line_bytes:
                    break

                line = line_bytes.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                if line.lower() in ["exit", "quit"]:
                    print("[*] Terminating wireless link...")
                    break

                payload = normalize_and_validate_command(line)
                if payload is None:
                    print(f"[!] Rejected command '{line}'. Type valid command (e.g. F, B, L, R, S, q, w, e).")
                    continue

                await client.write_gatt_char(GATT_UART_CHAR_UUID, payload, response=False)
                await asyncio.sleep(0.05)
        finally:
            try:
                print("\n[*] Safety Interlock: Engaging EMERGENCY BRAKE 'S' before disconnect...")
                await client.write_gatt_char(GATT_UART_CHAR_UUID, b"S", response=False)
                await asyncio.sleep(0.15)
            except Exception as exc:
                print(f"[-] Notice on disconnect brake: {exc}")


def main() -> None:
    """Program entrypoint with clean signal handling."""
    try:
        asyncio.run(run_controller_session())
    except (KeyboardInterrupt, asyncio.CancelledError):
        print("\n[!] Program interrupted by user. Safety halt executed. Exited cleanly.")


if __name__ == "__main__":
    main()
