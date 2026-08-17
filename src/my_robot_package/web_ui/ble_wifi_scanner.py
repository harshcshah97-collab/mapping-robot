import subprocess
import json

import logging
import threading
import time
import asyncio
import urllib.request
from typing import Any

from bless import (
    BlessServer,
    BlessGATTCharacteristic,
    GATTCharacteristicProperties,
    GATTAttributePermissions
)

logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')

logger = logging.getLogger("robot_ble")

SERVICE_UUID = "12345678-1234-5678-1234-56789abcdefa"
WIFI_CONFIG_UUID = "12345678-1234-5678-1234-56789abcdef2"
NODE_CMD_UUID = "12345678-1234-5678-1234-56789abcdef3"
IP_UUID = "12345678-1234-5678-1234-56789abcdef4"
WIFI_SCAN_UUID = "12345678-1234-5678-1234-56789abcdef5"

_cached_ip = "127.0.0.1"


def update_ip_loop():
    global _cached_ip
    while True:
        try:
            output = subprocess.check_output(
                ['ip', '-4', '-o', 'addr', 'show', 'dev', 'wlan0'],
                timeout=1,
                text=True,
            )
            address = output.split('inet ', 1)[1].split('/', 1)[0]
            _cached_ip = address.strip()
        except Exception:
            pass
        time.sleep(5)


threading.Thread(target=update_ip_loop, daemon=True).start()


def scan_wifi_networks():
    try:
        subprocess.run(['sudo', 'nmcli', 'dev', 'wifi', 'rescan'], timeout=5, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    except Exception:
        pass

    try:
        result = subprocess.run(
            ['sudo', 'nmcli', '-t', '-f', 'SSID', 'dev', 'wifi'],
            capture_output=True, text=True
        )
        if result.returncode == 0:
            ssids = [line.strip() for line in result.stdout.split('\n') if line.strip()]
            selected = []
            # Keep one JSON notification below a typical negotiated BLE payload.
            for ssid in sorted(set(ssids)):
                candidate = json.dumps(selected + [ssid]).encode('utf-8')
                if len(candidate) > 160:
                    break
                selected.append(ssid)
            return json.dumps(selected).encode('utf-8')
        else:
            return json.dumps([f"Error: nmcli {result.returncode}"]).encode('utf-8')
    except Exception as e:
        return json.dumps([f"Error: {str(e)}"]).encode('utf-8')


server = None


def read_request(characteristic: BlessGATTCharacteristic, **kwargs) -> bytearray:
    if characteristic.uuid.lower() == IP_UUID.lower():
        return bytearray(_cached_ip.encode('utf-8'))
    return characteristic.value


def write_request(characteristic: BlessGATTCharacteristic, value: Any, **kwargs):
    uuid = characteristic.uuid.lower()

    # Convert value to string safely and strip null bytes (crucial for iOS BLE)
    if isinstance(value, (bytes, bytearray)):
        data = value.decode('utf-8', errors='ignore').replace('\x00', '').strip()
    else:
        data = str(value).replace('\x00', '').strip()

    if uuid == NODE_CMD_UUID.lower():
        logger.info("Received node command: %s", data)
        try:
            # Centralize node management by calling the local web server API
            routes = {
                "START_NODES": "start_teleop",
                "START_TELEOP": "start_teleop",
                "START_MAPPING": "start_mapping",
                "START_NAVIGATION": "start_navigation",
                "START_TRACKING": "start_tracking",
                "STOP_NODES": "stop",
            }
            route = routes.get(data)
            if route is None:
                logger.warning("Ignoring unknown node command: %s", data)
                return
            request_url = f"http://127.0.0.1:8080/api/{route}"
            req = urllib.request.Request(request_url, method="POST")
            with urllib.request.urlopen(req, timeout=5):
                pass
            logger.info("Successfully routed '%s' to web server.", data)
        except Exception as e:
            logger.error("Failed to route command to web server: %s", e)

    elif uuid == WIFI_CONFIG_UUID.lower():
        try:
            creds = json.loads(data)
            ssid = creds.get('ssid', '').strip()
            password = creds.get('password', '').strip()
            if not ssid:
                raise ValueError("SSID is required")
            logger.info("Connecting to SSID: %s", ssid)
            cmd = ['sudo', 'nmcli', 'dev', 'wifi', 'connect', ssid]
            if password:
                cmd.extend(['password', password])

            result = subprocess.run(
                cmd, capture_output=True, text=True, timeout=30
            )
            if result.returncode == 0:
                logger.info("Successfully connected to %s", ssid)
            else:
                logger.error(
                    "Wi-Fi connection failed: %s",
                    result.stderr.strip() or result.stdout.strip(),
                )
        except Exception as e:
            logger.error("Error processing Wi-Fi credentials: %s", e)

    elif uuid == WIFI_SCAN_UUID.lower():
        if data == "SCAN":
            logger.info("Received SCAN command from iOS app.")

            def perform_scan_and_notify():
                json_bytes = scan_wifi_networks()
                char = server.get_characteristic(WIFI_SCAN_UUID)
                if char:
                    char.value = bytearray(json_bytes)
                    server.update_value(SERVICE_UUID, WIFI_SCAN_UUID)
                    logger.info("Sending Wi-Fi list to iOS app: %s", json_bytes)
            threading.Thread(
                target=perform_scan_and_notify, daemon=True
            ).start()


async def main():
    global server

    server = BlessServer(name="RobotBLE")
    server.read_request_func = read_request
    server.write_request_func = write_request

    await server.add_new_service(SERVICE_UUID)

    await server.add_new_characteristic(
        SERVICE_UUID, WIFI_SCAN_UUID,
        GATTCharacteristicProperties.write | GATTCharacteristicProperties.notify,
        value=bytearray(b'[]'),
        permissions=GATTAttributePermissions.writeable | GATTAttributePermissions.readable
    )

    await server.add_new_characteristic(
        SERVICE_UUID, WIFI_CONFIG_UUID,
        GATTCharacteristicProperties.write,
        value=bytearray(b''),
        permissions=GATTAttributePermissions.writeable
    )

    await server.add_new_characteristic(
        SERVICE_UUID, IP_UUID,
        GATTCharacteristicProperties.read | GATTCharacteristicProperties.notify,
        value=bytearray(_cached_ip.encode('utf-8')),
        permissions=GATTAttributePermissions.readable
    )

    await server.add_new_characteristic(
        SERVICE_UUID, NODE_CMD_UUID,
        GATTCharacteristicProperties.write,
        value=bytearray(b''),
        permissions=GATTAttributePermissions.writeable
    )

    await server.start()
    logger.info("BLE server started via Bless. Ready for iOS.")

    try:
        while True:
            await asyncio.sleep(5)
            # Periodically update IP address notification
            char = server.get_characteristic(IP_UUID)
            if char:
                char.value = bytearray(_cached_ip.encode('utf-8'))
                server.update_value(SERVICE_UUID, IP_UUID)
    except asyncio.CancelledError:
        pass
    except Exception as e:
        logger.error("Event loop error: %s", e)
    finally:
        logger.info("Stopping BLE server...")
        await server.stop()

if __name__ == '__main__':
    # Disable Wi-Fi power management to prevent dropouts when Bluetooth is active
    subprocess.run(['sudo', 'iw', 'dev', 'wlan0', 'set', 'power_save', 'off'], stderr=subprocess.DEVNULL)

    asyncio.run(main())
