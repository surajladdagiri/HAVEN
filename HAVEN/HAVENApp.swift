// HAVENApp.swift
// HAVEN — App entry point

import SwiftUI
import Combine

class AppState: ObservableObject {
    enum Page {
        case Bluetooth, Algorithm, Error
    }
    @Published var currPage: Page = .Bluetooth
}

@main
struct HAVENApp: App {
    @StateObject var appState: AppState
    @StateObject var blemanager: BLEManager
    // FIX (Step 5): LiDARStreamManager was previously created inside ARKitVisualView.init()
    // via ObservedObject(wrappedValue:). SwiftUI can call a view's initialiser on every
    // re-render, which recreated LiDARStreamManager (and its ARView) and caused
    // ARViewContainerTwo.makeUIView to run again — firing session.run() on an already-running
    // session and producing "Attempting to enable an already-enabled session. Ignoring...".
    // Hoisting it here as @StateObject guarantees exactly one instance for the app lifetime.
    @StateObject var streamManager: LiDARStreamManager

    init() {
        let appState   = AppState()
        let blemanager = BLEManager(appState: appState)
        _appState      = StateObject(wrappedValue: appState)
        _blemanager    = StateObject(wrappedValue: blemanager)
        _streamManager = StateObject(wrappedValue: LiDARStreamManager(ble: blemanager))
    }

    var body: some Scene {
        WindowGroup {
            switch appState.currPage {
            case .Bluetooth:
                BluetoothView(appState: appState, ble: blemanager)

            case .Algorithm:
                NavigationStack {
                    ARKitVisualView(bleManager: blemanager, streamManager: streamManager)
                        .toolbar {
                            ToolbarItem(placement: .navigationBarTrailing) {
                                NavigationLink(
                                    destination: CommandView(appState: appState, ble: blemanager)
                                ) {
                                    Image(systemName: "desktopcomputer.and.arrow.down")
                                        .foregroundColor(.white)
                                }
                            }
                        }
                }

            case .Error:
                ErrorView()
            }
        }
    }
}
