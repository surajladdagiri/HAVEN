// AlgorithmView.swift
// HAVEN — Algorithm selection menu (shown after BLE connection)
// BLEManager is accessed via @EnvironmentObject (injected by HAVENApp).

import SwiftUI

struct AlgorithmView: View {
    // BLEManager flows down from HAVENApp via .environmentObject(blemanager)
    // No local instantiation needed (and doing so was a bug — init requires AppState).
    @EnvironmentObject var blemanager: BLEManager

    var body: some View {
        NavigationStack {
            List {
                Section(header: Text("Algorithms")) {

                    NavigationLink(destination: ARKitV1View()) {
                        AlgoRow(icon: "1.circle.fill", title: "ARKit",
                                subtitle: "Depth proximity + phone haptics")
                    }

                    NavigationLink(destination: ARKitV2View()) {
                        AlgoRow(icon: "2.circle.fill", title: "ARKit SLAM",
                                subtitle: "37-ray radar + safety lock")
                    }

                    NavigationLink(destination: MLV1View()) {
                        AlgoRow(icon: "3.circle.fill", title: "Apple ML Model",
                                subtitle: "YOLO object detection + depth fusion")
                    }

                    NavigationLink(destination: ARKitVisualView(bleManager: blemanager)) {
                        AlgoRow(icon: "4.circle.fill", title: "SLAM Navigation",
                                subtitle: "2D map · A* pathfinding · haptic guidance",
                                accent: .teal)
                    }
                }
            }
            .navigationTitle("HAVEN")
            .navigationBarTitleDisplayMode(.large)
        }
    }
}

// ── Helper row component ──────────────────────────────────────────────────────

private struct AlgoRow: View {
    let icon: String
    let title: String
    let subtitle: String
    var accent: Color = .blue

    var body: some View {
        HStack(spacing: 14) {
            Image(systemName: icon)
                .foregroundColor(accent)
                .font(.title2)
                .frame(width: 32)

            VStack(alignment: .leading, spacing: 2) {
                Text(title).font(.headline)
                Text(subtitle).font(.caption).foregroundColor(.secondary)
            }
        }
        .padding(.vertical, 6)
    }
}

#Preview {
    // Previews need a mock AppState + BLEManager
    let state = AppState()
    return AlgorithmView()
        .environmentObject(BLEManager(appState: state))
}
