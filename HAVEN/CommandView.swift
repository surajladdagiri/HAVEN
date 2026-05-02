//
//  CommandView.swift
//  HAVEN
//

import SwiftUI

private struct HapticPreset: Identifiable {
    let id = UUID()
    let label: String
    let values: [UInt8]   // exactly 5 bytes, 0–100 per motor
}

struct CommandView: View {
    @ObservedObject var appState: AppState
    @ObservedObject var blemanager: BLEManager

    private let presets: [HapticPreset] = [
        HapticPreset(label: "All Off",             values: [0,  0,  0,  0,  0]),
        HapticPreset(label: "Motor 1 — far left",  values: [20, 0,  0,  0,  0]),
        HapticPreset(label: "Motor 2 — left",      values: [0,  20, 0,  0,  0]),
        HapticPreset(label: "Motor 3 — centre",    values: [0,  0,  20, 0,  0]),
        HapticPreset(label: "Motor 4 — right",     values: [0,  0,  0,  20, 0]),
        HapticPreset(label: "Motor 5 — far right", values: [0,  0,  0,  0,  20]),
        HapticPreset(label: "Motors 1+2",          values: [20, 20, 0,  0,  0]),
        HapticPreset(label: "Motors 3+4+5",        values: [0,  0,  20, 20, 20]),
        HapticPreset(label: "All at 50",           values: [50, 50, 50, 50, 50]),
    ]

    init(appState: AppState, ble: BLEManager) {
        self._appState   = ObservedObject(wrappedValue: appState)
        self._blemanager = ObservedObject(wrappedValue: ble)
    }

    var body: some View {
        VStack(spacing: 12) {
            Image(systemName: "hand.tap.fill")
                .font(.largeTitle)
                .padding(.top, 20)
            Text("Send Haptic Commands")
                .font(.headline)
            Text("Raw bytes · 5 motors · 0–100")
                .font(.caption)
                .foregroundColor(.secondary)

            Spacer()

            ForEach(presets) { preset in
                Button {
                    blemanager.sendHapticValues(preset.values)
                } label: {
                    HStack {
                        Text(preset.label)
                            .font(.body)
                        Spacer()
                        Text(preset.values.map(String.init).joined(separator: ","))
                            .font(.system(.caption, design: .monospaced))
                            .foregroundColor(.secondary)
                    }
                    .padding(.horizontal, 20)
                }
                .frame(width: 320, height: 50)
                .background(Color.gray.opacity(0.15))
                .cornerRadius(12)
            }

            Spacer()
        }
        .navigationTitle("BLE Commands")
    }
}
