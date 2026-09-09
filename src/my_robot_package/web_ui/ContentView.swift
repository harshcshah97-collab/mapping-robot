import SwiftUI
import CoreBluetooth

struct ContentView: View {
    @StateObject private var bleManager = RobotBLEManager()
    
    var body: some View {
        TabView {
            ControlView(bleManager: bleManager)
                .tabItem {
                    Label("Control", systemImage: "gamecontroller")
                }
            
            ConnectionView(bleManager: bleManager)
                .tabItem {
                    Label("Bluetooth", systemImage: "bluetooth")
                }
            
            SettingsView(bleManager: bleManager)
                .tabItem {
                    Label("Settings", systemImage: "gearshape")
                }
        }
    }
}

struct ControlView: View {
    @ObservedObject var bleManager: RobotBLEManager
    
    var body: some View {
        VStack(spacing: 0) {
            // Header Status Bar
            HStack {
                Image(systemName: bleManager.isConnected ? "bluetooth.connected" : "bluetooth")
                    .foregroundColor(bleManager.isConnected ? .blue : .gray)
                Text(bleManager.isConnected ? "Robot Connected" : "Not Connected")
                    .fontWeight(.bold)
                
                Spacer()
                
                if !bleManager.robotIP.isEmpty {
                    Image(systemName: "wifi")
                        .foregroundColor(.green)
                    Text(bleManager.robotIP)
                        .font(.caption)
                }
            }
            .padding()
            .background(Color(UIColor.secondarySystemBackground))
            
            // Hybrid Map Area
            ZStack {
                Color.black.edgesIgnoringSafeArea(.all)
                
                if !bleManager.robotIP.isEmpty {
                    // Stream your awesome existing Web UI map via Wi-Fi!
                    MapWebView(ipAddress: bleManager.robotIP)
                } else {
                    VStack(spacing: 15) {
                        Image(systemName: "map.triangle")
                            .font(.system(size: 50))
                            .foregroundColor(.gray)
                        Text("Map Unavailable")
                            .font(.title2)
                            .foregroundColor(.white)
                        Text("Connect robot to Wi-Fi to stream map.")
                            .foregroundColor(.gray)
                    }
                }
            }
        }
    }
}

struct ConnectionView: View {
    @ObservedObject var bleManager: RobotBLEManager
    
    var body: some View {
        NavigationView {
            List {
                Section(header: Text("Status")) {
                    HStack {
                        Text("Bluetooth")
                        Spacer()
                        Text(bleManager.isConnected ? "Connected" : "Disconnected")
                            .foregroundColor(bleManager.isConnected ? .green : .gray)
                    }
                    if bleManager.isConnected {
                        Button("Disconnect") {
                            bleManager.disconnect()
                        }
                        .foregroundColor(.red)
                    }
                }
                
                Section(header: Text("Discovered Robots")) {
                    if bleManager.isScanning {
                        HStack {
                            ProgressView()
                            Text("Scanning...")
                                .padding(.leading, 8)
                        }
                    } else if !bleManager.isConnected {
                        Button("Scan for Robots") {
                            bleManager.startScanning()
                        }
                    }
                    
                    ForEach(bleManager.discoveredPeripherals, id: \.identifier) { peripheral in
                        Button(action: {
                            bleManager.connect(to: peripheral)
                        }) {
                            HStack {
                                Text(peripheral.name ?? "Unknown Robot")
                                Spacer()
                                if bleManager.robotPeripheral?.identifier == peripheral.identifier && bleManager.isConnected {
                                    Image(systemName: "checkmark").foregroundColor(.blue)
                                }
                            }
                        }
                        .foregroundColor(.primary)
                    }
                }
            }
            .navigationTitle("Bluetooth")
        }
    }
}

struct SettingsView: View {
    @ObservedObject var bleManager: RobotBLEManager
    
    @State private var ssid = ""
    @State private var password = ""
    
    var body: some View {
        NavigationView {
            Form {
                Section(header: Text("Wi-Fi Setup"), footer: Text("Scan for open networks and send Wi-Fi credentials to the robot over Bluetooth.")) {
                    if bleManager.isScanningWiFi {
                        HStack {
                            Text("Scanning networks...")
                            Spacer()
                            ProgressView()
                        }
                    } else {
                        Button("Scan for Wi-Fi Networks") {
                            bleManager.requestWiFiScan()
                        }
                        .disabled(!bleManager.isConnected)
                    }
                    
                    if !bleManager.availableNetworks.isEmpty {
                        Picker("Available Networks", selection: $ssid) {
                            Text("Select a network...").tag("")
                            ForEach(bleManager.availableNetworks, id: \.self) { network in
                                Text(network).tag(network)
                            }
                        }
                    }
                    
                    TextField("SSID (Manual Entry)", text: $ssid)
                    SecureField("Password", text: $password)
                    Button("Connect to Wi-Fi") {
                        bleManager.sendWiFiCredentials(ssid: ssid, password: password)
                    }
                    .disabled(!bleManager.isConnected || ssid.isEmpty)
                }
                
                Section(header: Text("Node Management")) {
                    Button("Start Teleop") {
                        bleManager.sendNodeCommand(command: "START_TELEOP")
                    }
                    .disabled(!bleManager.isConnected)

                    Button("Start Mapping") {
                        bleManager.sendNodeCommand(command: "START_MAPPING")
                    }
                    .disabled(!bleManager.isConnected)

                    Button("Start Navigation") {
                        bleManager.sendNodeCommand(command: "START_NAVIGATION")
                    }
                    .disabled(!bleManager.isConnected)

                    Button("Start Person Tracking") {
                        bleManager.sendNodeCommand(command: "START_TRACKING")
                    }
                    .disabled(!bleManager.isConnected)
                    
                    Button("Stop ROS Nodes") {
                        bleManager.sendNodeCommand(command: "STOP_NODES")
                    }
                    .foregroundColor(.red)
                    .disabled(!bleManager.isConnected)
                }
            }
            .navigationTitle("Robot Settings")
        }
    }
}
