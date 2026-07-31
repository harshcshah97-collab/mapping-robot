import os
import sys
import subprocess
import json

import logging
import threading
import time
import asyncio
import os
import urllib.request
from typing import Any
import builtins

from bless import (
    BlessServer,
    BlessGATTCharacteristic,
    GATTCharacteristicProperties,
    GATTAttributePermissions
)

logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')

# Force all print statements to go through logging so they instantly appear in systemd
def _systemd_print(*args, **kwargs):
    logging.info(" ".join(map(str, args)))
builtins.print = _systemd_print

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
            _cached_ip = subprocess.check_output(['hostname', '-I'], timeout=1).decode('utf-8').split()[0]
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
            unique_ssids = sorted(list(set(ssids)))
            unique_ssids = unique_ssids[:15] # Increased limit, but kept capped to respect BLE MTU sizes
            return json.dumps(unique_ssids).encode('utf-8')
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
        print(f"Received Node Command: {data}")
        try:
            # Centralize node management by calling the local web server API
            if data == "START_NODES" or data == "START_TELEOP":
                req = urllib.request.Request("http://127.0.0.1:8080/api/start_teleop", method="POST")
                urllib.request.urlopen(req, timeout=2)
            elif data == "START_MAPPING":
                req = urllib.request.Request("http://127.0.0.1:8080/api/start_mapping", method="POST")
                urllib.request.urlopen(req, timeout=2)
            elif data == "START_NAVIGATION":
                req = urllib.request.Request("http://127.0.0.1:8080/api/start_navigation", method="POST")
                urllib.request.urlopen(req, timeout=2)
            elif data == "STOP_NODES":
                req = urllib.request.Request("http://127.0.0.1:8080/api/stop", method="POST")
                urllib.request.urlopen(req, timeout=2)
            print(f"Successfully routed '{data}' to web server.")
        except Exception as e:
            print(f"Failed to route command to web server (Web Server might be offline): {e}")

    elif uuid == WIFI_CONFIG_UUID.lower():
        try:
            creds = json.loads(data)
            ssid = creds.get('ssid', '').strip()
            password = creds.get('password', '').strip()
            print(f"Connecting to SSID: {ssid}")
            cmd = ['sudo', 'nmcli', 'dev', 'wifi', 'connect', ssid]
            if password:
                cmd.extend(['password', password])
                
            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode == 0:
                print(f"Successfully connected to {ssid}")
            else:
                print(f"Wi-Fi connection failed: {result.stderr.strip() or result.stdout.strip()}")
        except Exception as e:
            print(f"Error parsing Wi-Fi credentials: {e}, data: '{data}'")

    elif uuid == WIFI_SCAN_UUID.lower():
        if data == "SCAN":
            print("Received SCAN command from iOS app. Scanning...")
            def perform_scan_and_notify():
                json_bytes = scan_wifi_networks()
                char = server.get_characteristic(WIFI_SCAN_UUID)
                if char:
                    char.value = bytearray(json_bytes)
                    server.update_value(SERVICE_UUID, WIFI_SCAN_UUID)
                    print(f"Sending Wi-Fi list to iOS app: {json_bytes}")
            threading.Thread(target=perform_scan_and_notify).start()

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
    print("BLE Server Started via Bless (Native D-Bus)! Ready for iOS.")
    
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
        print(f"Event loop error: {e}")
    finally:
        print("Stopping BLE server...")
        await server.stop()

if __name__ == '__main__':
    # Disable Wi-Fi power management to prevent dropouts when Bluetooth is active
    subprocess.run(['sudo', 'iw', 'dev', 'wlan0', 'set', 'power_save', 'off'], stderr=subprocess.DEVNULL)
    
    asyncio.run(main())