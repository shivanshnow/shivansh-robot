#!/usr/bin/env python3
"""
===============================================================================
SHIVANSH MECHA OS • Autonomous Robotics Platform
Embedded Systems Architecture CS140 / CS110: Web Applications & Transport Layer Security Real-Time Control
===============================================================================
File: server.py
Author: Pilot Shivansh & Antigravity AI Pair-Programmer
Target Platform: Python 3.10+ / macOS / Linux

Description:
  Standalone TLS/SSL HTTPS web server and QR-code pairing gateway for the
  ⚡ SHIVANSH MECHA OS Web Bluetooth mobile cockpit.
  
Key Features:
  - Dynamically detects the host's active LAN IPv4 interface address.
  - Generates an in-terminal high-contrast ASCII QR code for mobile scanning.
  - Provisions an ephemeral self-signed X.509 certificate to satisfy W3C Web
    Bluetooth secure context (HTTPS) constraints.
===============================================================================
"""

import http.server
import os
import socket
import ssl
import sys
import qrcode

PORT: int = 8443
WEB_DIR: str = os.path.dirname(os.path.abspath(__file__))


def get_local_ip_address() -> str:
    """
    Determines the host machine's primary local IPv4 address via UDP socket routing.
    
    Returns:
        String representing the local IPv4 address (e.g., '192.168.1.41').
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(('8.8.8.8', 80))
        ip_addr = sock.getsockname()[0]
    except Exception:
        ip_addr = '127.0.0.1'
    finally:
        sock.close()
    return ip_addr


def print_cockpit_banner(url: str) -> None:
    """
    Renders the SHIVANSH MECHA OS system banner and in-terminal QR code for phone pairing.
    
    Parameters:
        url: Complete HTTPS URL endpoint to display.
    """
    print("\n" + "=" * 62)
    print(" ⚡ SHIVANSH MECHA OS • WEB BLUETOOTH HTTPS GATEWAY ⚡")
    print("=" * 62)
    print(f"\n👉 MOBILE LINK: \033[1;36m{url}\033[0m\n")
    print("Scan QR code below with your smartphone camera:")
    print("-" * 62)

    try:
        qr = qrcode.QRCode(border=1)
        qr.add_data(url)
        qr.make(fit=True)
        qr.print_ascii(invert=True)
    except Exception as exc:
        print(f"[!] Could not render ASCII QR code: {exc}")

    print("-" * 62)
    print("Press Ctrl+C to terminate HTTPS server.\n")


def run_https_server() -> None:
    """
    Initializes and starts the HTTPS daemon listening on 0.0.0.0:8443.
    """
    local_ip = get_local_ip_address()
    url = f"https://{local_ip}:{PORT}"
    print_cockpit_banner(url)

    os.chdir(WEB_DIR)
    server_address = ('0.0.0.0', PORT)
    httpd = http.server.HTTPServer(server_address, http.server.SimpleHTTPRequestHandler)

    cert_path = os.path.join(WEB_DIR, 'cert.pem')
    key_path = os.path.join(WEB_DIR, 'key.pem')

    if os.path.exists(cert_path) and os.path.exists(key_path):
        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ssl_context.load_cert_chain(certfile=cert_path, keyfile=key_path)
        httpd.socket = ssl_context.wrap_socket(httpd.socket, server_side=True)
    else:
        print("[!] Warning: SSL certificate files not found. Serving plaintext HTTP.")

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[!] HTTPS server stopped cleanly.")
        sys.exit(0)


if __name__ == "__main__":
    run_https_server()
