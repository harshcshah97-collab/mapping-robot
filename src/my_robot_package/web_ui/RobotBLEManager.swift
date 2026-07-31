import Foundation
import CoreBluetooth
import Combine

class RobotBLEManager: NSObject, ObservableObject, CBCentralManagerDelegate, CBPeripheralDelegate {
    
    // These UUIDs will match the Python server we build on the Pi
    let serviceUUID = CBUUID(string: "12345678-1234-5678-1234-56789abcdefa")
    let wifiConfigUUID = CBUUID(string: "12345678-1234-5678-1234-56789abcdef2")
    let nodeCommandUUID = CBUUID(string: "12345678-1234-5678-1234-56789abcdef3")
    let ipUUID = CBUUID(string: "12345678-1234-5678-1234-56789abcdef4")
    let wifiScanUUID = CBUUID(string: "12345678-1234-5678-1234-56789abcdef5")
    
    @Published var isConnected = false
    @Published var robotIP: String = ""
    @Published var isScanning = false
    @Published var discoveredPeripherals: [CBPeripheral] = []
    @Published var robotPeripheral: CBPeripheral?
    @Published var availableNetworks: [String] = []
    @Published var isScanningWiFi = false
    
    private var centralManager: CBCentralManager!
    private var wifiCharacteristic: CBCharacteristic?
    private var nodeCharacteristic: CBCharacteristic?
    private var wifiScanCharacteristic: CBCharacteristic?
    
    override init() {
        super.init()
        centralManager = CBCentralManager(delegate: self, queue: nil)
    }
    
    func startScanning() {
        if centralManager.state == .poweredOn {
            isScanning = true
            discoveredPeripherals.removeAll()
            centralManager.scanForPeripherals(withServices: [serviceUUID], options: nil)
        }
    }
    
    func stopScanning() {
        isScanning = false
        centralManager.stopScan()
    }
    
    func connect(to peripheral: CBPeripheral) {
        stopScanning()
        robotPeripheral = peripheral
        robotPeripheral?.delegate = self
        centralManager.connect(peripheral, options: nil)
    }
    
    func disconnect() {
        if let peripheral = robotPeripheral {
            centralManager.cancelPeripheralConnection(peripheral)
        }
    }
    
    // MARK: - Central Manager Delegate
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        if central.state == .poweredOn {
            print("Bluetooth is On. Ready to scan.")
        } else {
            print("Bluetooth is not available.")
        }
    }
    
    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String : Any], rssi RSSI: NSNumber) {
        print("Discovered: \(peripheral.name ?? "Unknown Robot")")
        DispatchQueue.main.async {
            if !self.discoveredPeripherals.contains(where: { $0.identifier == peripheral.identifier }) {
                self.discoveredPeripherals.append(peripheral)
            }
        }
    }
    
    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        print("Connected to Robot!")
        DispatchQueue.main.async {
            self.isConnected = true
        }
        peripheral.discoverServices([serviceUUID])
    }
    
    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        print("Disconnected. Re-scanning...")
        DispatchQueue.main.async {
            self.isConnected = false
            self.robotIP = ""
            self.robotPeripheral = nil
        }
    }
    
    // MARK: - Peripheral Delegate
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        guard let services = peripheral.services else { return }
        for service in services {
            peripheral.discoverCharacteristics(nil, for: service)
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        print("Discovering characteristics for service...")
        guard let characteristics = service.characteristics else { return }
        for characteristic in characteristics {
            print("Found characteristic: \(characteristic.uuid)")
            if characteristic.uuid == ipUUID {
                // Subscribe to IP updates
                peripheral.setNotifyValue(true, for: characteristic)
                peripheral.readValue(for: characteristic)
            } else if characteristic.uuid == wifiConfigUUID {
                self.wifiCharacteristic = characteristic
            } else if characteristic.uuid == nodeCommandUUID {
                self.nodeCharacteristic = characteristic
            } else if characteristic.uuid == wifiScanUUID {
                self.wifiScanCharacteristic = characteristic
                peripheral.setNotifyValue(true, for: characteristic)
            }
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        if characteristic.uuid == ipUUID, let data = characteristic.value {
            if let ipString = String(data: data, encoding: .utf8) {
                DispatchQueue.main.async {
                    self.robotIP = ipString
                    print("Received Robot IP over BLE: \(self.robotIP)")
                }
            }
        } else if characteristic.uuid == wifiScanUUID, let data = characteristic.value {
            // Expecting a JSON array of strings: ["Network1", "Network2"]
            if let networks = try? JSONDecoder().decode([String].self, from: data) {
                DispatchQueue.main.async {
                    self.availableNetworks = Array(Set(networks)).filter { !$0.isEmpty }.sorted()
                    self.isScanningWiFi = false
                }
            }
        }
    }
    
    // MARK: - Actions
    
    func requestWiFiScan() {
        guard let peripheral = robotPeripheral, let characteristic = wifiScanCharacteristic else {
            print("⚠️ Cannot request Wi-Fi scan: Characteristic is nil!")
            return
        }
        DispatchQueue.main.async { self.isScanningWiFi = true }
        
        let commandString = "SCAN"
        if let data = commandString.data(using: .utf8) {
            peripheral.writeValue(data, for: characteristic, type: .withResponse)
        }
    }
    
    func sendWiFiCredentials(ssid: String, password: String) {
        guard let peripheral = robotPeripheral, let characteristic = wifiCharacteristic else {
            print("⚠️ Cannot send Wi-Fi creds: Characteristic is nil!")
            return
        }
        
        let json: [String: String] = ["ssid": ssid, "password": password]
        if let jsonData = try? JSONEncoder().encode(json) {
            peripheral.writeValue(jsonData, for: characteristic, type: .withResponse)
        }
    }
    
    func sendNodeCommand(command: String) {
        guard let peripheral = robotPeripheral, let characteristic = nodeCharacteristic else {
            print("⚠️ Cannot send Node command: Characteristic is nil!")
            return
        }
        
        print("Sending node command: \(command)")
        if let data = command.data(using: .utf8) {
            peripheral.writeValue(data, for: characteristic, type: .withResponse)
        }
    }
}