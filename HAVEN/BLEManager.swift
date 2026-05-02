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
            peripheral.discoverCharacteristics(nil, for: service)
        }
    }

    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService,
                    error: Error?) {
        guard let chars = service.characteristics else { return }

        // Pick the best writable characteristic.
        // Priority: write-with-response > write-without-response > first available.
        // Most Arduino haptic firmware uses writeWithoutResponse for low-latency output.
        var bestChar: CBCharacteristic? = nil
        for char in chars {
            let props = char.properties
            print("Characteristic \(char.uuid) – " +
                  "read:\(props.contains(.read)) " +
                  "write:\(props.contains(.write)) " +
                  "writeNoResp:\(props.contains(.writeWithoutResponse))")
            if props.contains(.write) || props.contains(.writeWithoutResponse) {
                // Prefer a full .write char; accept .writeWithoutResponse
                if bestChar == nil || (!bestChar!.properties.contains(.write) && props.contains(.write)) {
                    bestChar = char
                }
            }
        }
        MainCharacteristic = bestChar ?? chars.first

        if let mc = MainCharacteristic {
            print("→ Using characteristic \(mc.uuid) with properties \(mc.properties.rawValue)")
        } else {
            print("⚠️ No usable characteristic found – haptics will not fire")
        }

        withAnimation { appState?.currPage = .Algorithm }
        withAnimation { connected = true }
        withAnimation { FinishedAuto = true }
    }

    // ── Write — v2 Raw Byte Protocol ─────────────────────────────────────────
    //
    // Sends exactly 5 bytes to the Arduino, one per DRV2605L motor, each 0–100.
    // The firmware maps them directly:
    //   byte[0] = Motor 1 (far left)
    //   byte[1] = Motor 2
    //   byte[2] = Motor 3 (centre)
    //   byte[3] = Motor 4
    //   byte[4] = Motor 5 (far right)
    //
    // Write type is determined from the characteristic's advertised properties:
    //   • .write             → .withResponse  (Arduino confirms receipt)
    //   • .writeWithoutResponse → .withoutResponse  (fire-and-forget, lower latency)
    // Using the wrong type causes silent failure — hence the auto-detect.

    func sendHapticValues(_ values: [UInt8]) {
        guard let peripheral = ESP32,
              let char = MainCharacteristic else {
            print("❌ BLE not ready")
            return
        }

        let payload: [UInt8] = Array(
            (values + [0,0,0,0,0]).prefix(5)
        ).map { min($0, 100) }

        print("📤 Sending:", payload)

        let data = Data(payload)

        // Auto-detect write type from the characteristic's advertised properties.
        // Using the wrong type causes a silent failure — CoreBluetooth will drop the
        // write without an error if the peripheral doesn't support the requested mode.
        //   • .write             → .withResponse  (Arduino confirms receipt)
        //   • .writeWithoutResponse → .withoutResponse  (fire-and-forget, lower latency)
        // Prefer write-with-response when the characteristic supports it so that the
        // Arduino's hapticChar.written() handler fires correctly; fall back to
        // without-response otherwise (e.g. a different firmware build).
        let writeType: CBCharacteristicWriteType = char.properties.contains(.write)
            ? .withResponse
            : .withoutResponse

        peripheral.writeValue(data, for: char, type: writeType)
    }

    // ── Delegate: write confirmation (withResponse only) ─────────────────────

    func peripheral(_ peripheral: CBPeripheral,
                    didWriteValueFor characteristic: CBCharacteristic,
                    error: Error?) {

        if let error = error {
            print("❌ WRITE ERROR:", error)
        } else {
            print("✅ WRITE ACKED")
        }
    }
}