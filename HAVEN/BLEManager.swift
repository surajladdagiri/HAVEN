// BLEManager.swift
// HAVEN — Bluetooth Low Energy central manager
// Manages connection to the Arduino Haptic_R4_Controller peripheral.

import Foundation
import CoreBluetooth
import SwiftUI
import Combine

enum BluetoothError: Error {
    case unknown, off
}

/// Service UUID must match the Arduino firmware constant SERVICE_UUID
let serviceID = CBUUID(string: "54df84fc-7f55-4867-bb29-617f9d2a7925")

class BLEManager: NSObject, ObservableObject, CBCentralManagerDelegate, CBPeripheralDelegate {

    private var manager: CBCentralManager!
    private var ESP32: CBPeripheral?
    private var MainService: CBService?
    private var MainCharacteristic: CBCharacteristic?

    @Published var peripherals = [CBPeripheral]()
    private var peripheral_IDs: Set<UUID> = []
    @Published var connected = false
    @Published var FinishedAuto = false

    var appState: AppState?

    init(appState: AppState) {
        super.init()
        manager = CBCentralManager(delegate: self, queue: nil)
        self.appState = appState
    }

    // ── CBCentralManager State ───────────────────────────────────────────────

    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        switch central.state {
        case .poweredOn:
            print("BLE powered on – auto-scanning")
            try? autoScanning()
        case .poweredOff:
            print("BLE powered off")
        default:
            print("BLE state: \(central.state.rawValue)")
        }
    }

    // ── Scanning ─────────────────────────────────────────────────────────────

    func autoScanning() throws {
        guard manager.state != .poweredOff else { throw BluetoothError.off }
        guard manager.state == .poweredOn  else { throw BluetoothError.unknown }

        print("Auto-scan started")
        manager.scanForPeripherals(withServices: [serviceID])

        DispatchQueue.main.asyncAfter(deadline: .now() + 2) {
            self.manager.stopScan()
            print("Auto-scan stopped – found \(self.peripherals.count) device(s)")
            if self.peripherals.count == 1 {
                self.connect(to: self.peripherals[0])
            } else {
                self.peripherals.removeAll()
                self.peripheral_IDs.removeAll()
                withAnimation { self.FinishedAuto = true }
            }
        }
    }

    func startScanning() throws {
        guard manager.state != .poweredOff else { throw BluetoothError.off }
        guard manager.state == .poweredOn  else { throw BluetoothError.unknown }
        manager.scanForPeripherals(withServices: [serviceID])
    }

    func stopScanning() {
        manager.stopScan()
        peripheral_IDs.removeAll()
    }

    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral,
                        advertisementData: [String: Any], rssi: NSNumber) {
        guard !peripheral_IDs.contains(peripheral.identifier) else { return }
        peripheral_IDs.insert(peripheral.identifier)
        let name = peripheral.name ?? "Unknown"
        guard name != "Unknown" else { return }
        print("Discovered: \(name)")
        withAnimation { peripherals.append(peripheral) }
    }

    // ── Connection ────────────────────────────────────────────────────────────

    func connect(to p: CBPeripheral) {
        ESP32 = p
        manager.connect(p)
    }

    func disconnect() {
        print("Disconnect called (no-op for now)")
    }

    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        peripheral.delegate = self
        peripheral.discoverServices([serviceID])
        withAnimation { connected = true }
    }

    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral,
                        error: Error?) {
        withAnimation { connected = false }
    }

    func centralManager(_ central: CBCentralManager, didFailToConnect peripheral: CBPeripheral,
                        error: Error?) {
        appState?.currPage = .Error
    }

    // ── Service / Characteristic Discovery ───────────────────────────────────

    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        guard let services = peripheral.services else { return }
        for service in services {
            MainService = service
            peripheral.discoverCharacteristics(nil, for: service)
        }
    }

    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService,
                    error: Error?) {
        guard let chars = service.characteristics else { return }
        for char in chars {
            MainCharacteristic = char
            print("Characteristic – read:\(char.properties.contains(.read)) write:\(char.properties.contains(.write)) writeNoResp:\(char.properties.contains(.writeWithoutResponse))")
        }
        withAnimation { appState?.currPage = .Algorithm }
        withAnimation { connected = true }
        withAnimation { FinishedAuto = true }
    }

    // ── Write Helpers ─────────────────────────────────────────────────────────

    /// Sends a UTF-8 string to the peripheral (legacy / debug use).
    func sendCommand(_ command: String) {
        guard let peripheral = ESP32, let char = MainCharacteristic else { return }
        peripheral.writeValue(Data(command.utf8), for: char, type: .withResponse)
    }

    /// Sends exactly 5 raw bytes to the haptic controller.
    /// The firmware reads them as motor intensities 0-100 (bytes are clamped on the Arduino).
    ///
    /// Layout: [strong_left, light_left, straight, light_right, strong_right]
    func sendHapticValues(_ values: [UInt8]) {
        guard let peripheral = ESP32, let char = MainCharacteristic else { return }
        // Always send exactly NUM_HAPTICS (5) bytes; pad or trim if needed
        var payload = [UInt8](repeating: 0, count: 5)
        for i in 0..<min(5, values.count) { payload[i] = values[i] }
        peripheral.writeValue(Data(payload), for: char, type: .withResponse)
    }
}