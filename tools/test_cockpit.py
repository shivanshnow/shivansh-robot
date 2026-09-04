#!/usr/bin/env python3
"""
===============================================================================
SHIVANSH MECHA OS • Automated Cockpit Regression & Headless Verification
===============================================================================
Serves the web cockpit locally, executes in a headless browser, and asserts:
  1. Zero uncaught JavaScript console errors or runtime exceptions.
  2. Zero 404 / broken HTTP asset requests.
  3. Critical DOM interaction nodes (#flowPad, #speedSlider, #btnEmergencyBrake, #hudLog) are rendered.
"""
import http.server
import os
import socketserver
import sys
import threading
import time

PORT = 8999
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class QuietHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=REPO_ROOT, **kwargs)

    def log_message(self, format, *args):
        pass  # Suppress HTTP access logging


def run_server():
    with socketserver.TCPServer(("", PORT), QuietHandler) as httpd:
        httpd.serve_forever()


def main():
    server_thread = threading.Thread(target=run_server, daemon=True)
    server_thread.start()
    time.sleep(0.5)

    errors = []
    failed_requests = []

    try:
        from playwright.sync_api import sync_playwright
        with sync_playwright() as p:
            browser = p.chromium.launch(headless=True)
            page = browser.new_page()

            def on_console(msg):
                if msg.type == "error":
                    errors.append(msg.text)

            page.on("console", on_console)
            page.on("pageerror", lambda exc: errors.append(str(exc)))

            def on_response(resp):
                if resp.status >= 400:
                    failed_requests.append(f"{resp.status} {resp.url}")

            page.on("response", on_response)

            page.goto(f"http://localhost:{PORT}/index.html", wait_until="networkidle")

            # Check critical DOM nodes
            required_ids = [
                "flowPad", "flowThumb", "speedSlider", "btnEmergencyBrake",
                "btnConnect", "hudLog", "pixelGrid", "tabBtn1", "tabBtn2", "tabBtn3", "tabBtn4"
            ]
            missing = []
            for el_id in required_ids:
                if not page.query_selector(f"#{el_id}"):
                    missing.append(el_id)

            browser.close()

            if missing:
                print(f"[-] FAILED: Missing required DOM elements: {missing}")
                sys.exit(1)
            if failed_requests:
                print(f"[-] FAILED: 404 or broken HTTP assets: {failed_requests}")
                sys.exit(1)
            if errors:
                print(f"[-] FAILED: Uncaught browser console errors: {errors}")
                sys.exit(1)

            print("[✔] Headless Browser Test PASSED! 0 console errors, 0 broken assets, all DOM nodes intact.")
            sys.exit(0)
    except ImportError:
        print("[!] Playwright is not installed in local environment. Running static HTML/DOM audit...")
        with open(os.path.join(REPO_ROOT, "index.html"), "r") as f:
            html = f.read()
        required_ids = [
            "flowPad", "flowThumb", "speedSlider", "btnEmergencyBrake",
            "btnConnect", "hudLog", "pixelGrid", "tabBtn1", "tabBtn2", "tabBtn3", "tabBtn4"
        ]
        for el_id in required_ids:
            assert f'id="{el_id}"' in html, f"Missing {el_id} in index.html"
        print("[✔] Static DOM inspection PASSED! All critical elements present.")
        sys.exit(0)


if __name__ == "__main__":
    main()
