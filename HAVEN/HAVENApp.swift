// HAVENApp.swift
// HAVEN — App entry point
// Injects BLEManager as an EnvironmentObject so every view in the hierarchy
// (including ARKitVisualView) can access the live BLE connection.

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

    init() {
        let appState = AppState()
        _appState   = StateObject(wrappedValue: appState)
        _blemanager = StateObject(wrappedValue: BLEManager(appState: appState))
    }

    var body: some Scene {
        WindowGroup {
            Group {
                if appState.currPage == .Bluetooth {
                    BluetoothView(appState: appState, ble: blemanager)
                } else if appState.currPage == .Algorithm {
                    AlgorithmView()
                } else {
                    ErrorView()
                }
            }
            // Inject blemanager so any descendant view can use @EnvironmentObject
            .environmentObject(blemanager)
        }
    }
}