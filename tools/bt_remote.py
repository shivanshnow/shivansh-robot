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
        print("   q / w / e : Shift Gear (1: Precision | 2: Cruise | 3: Turbo)")
        print("   F / B / L / R : Drive (Forward | Reverse | Left | Right)")
        print("   S         : EMERGENCY BRAKE (0 PWM)")
        print("   1 / 2 / 4 : Autonomous (1: Obstacle | 2: Line | 4: Music)")
        print("   0         : Safe Standby Mode")
        print("   exit      : Disconnect & Quit")
        print("-" * 58)

        # Subscribe to UART notifications
        await client.start_notify(GATT_UART_CHAR_UUID, telemetry_rx_callback)

        try:
            event_loop = asyncio.get_running_loop()
            while True:
                try:
                    raw_command = await event_loop.run_in_executor(None, input, "\nCockpit Command > ")
                except EOFError:
                    break

                command = raw_command.strip()
                if not command:
                    continue
                if command.lower() in ["exit", "quit"]:
                    print("[*] Terminating wireless link...")
                    break

                payload = command.encode("utf-8")
                await client.write_gatt_char(GATT_UART_CHAR_UUID, payload, response=False)
                await asyncio.sleep(0.1)
        finally:
            try:
                print("[*] Engaging safety halt 'S' before disconnect...")
                await client.write_gatt_char(GATT_UART_CHAR_UUID, b"S", response=False)
                await asyncio.sleep(0.15)
            except Exception:
                pass


def main() -> None:
    """Program entrypoint with clean signal handling."""
    try:
        asyncio.run(run_controller_session())
    except KeyboardInterrupt:
        print("\n[!] Program interrupted by user. Safety halt executed. Exited cleanly.")


if __name__ == "__main__":
    main()
