// HAVENApp.swift
// HAVEN — App entry point

import SwiftUI
import Combine
import UIKit

class AppState: ObservableObject {
    enum Page {
        case Bluetooth, Algorithm, Error
    }
    @Published var currPage: Page = .Bluetooth
}

@MainActor
final class InactivityBlackoutController: ObservableObject {
    static let timeoutNanoseconds: UInt64 = 10_000_000_000

    @Published private(set) var isBlackScreenVisible = false

    private var blackoutTask: Task<Void, Never>?

    func beginMonitoring() {
        scheduleBlackout()
    }

    func handleInteractionWhileAwake() {
        guard !isBlackScreenVisible else { return }
        scheduleBlackout()
    }

    func wakeScreen() {
        isBlackScreenVisible = false
        scheduleBlackout()
    }

    func stopMonitoring() {
        blackoutTask?.cancel()
        blackoutTask = nil
        isBlackScreenVisible = false
    }

    private func scheduleBlackout() {
        blackoutTask?.cancel()
        blackoutTask = Task { [weak self] in
            try? await Task.sleep(nanoseconds: Self.timeoutNanoseconds)
            guard !Task.isCancelled else { return }
            await self?.showBlackScreen()
        }
    }

    private func showBlackScreen() {
        isBlackScreenVisible = true
    }

}

private struct InactivityBlackoutContainer<Content: View>: View {
    @Environment(\.scenePhase) private var scenePhase
    @StateObject private var blackoutController = InactivityBlackoutController()

    private let content: () -> Content

    init(@ViewBuilder content: @escaping () -> Content) {
        self.content = content
    }

    var body: some View {
        ZStack {
            content()

            WindowTouchMonitor {
                blackoutController.handleInteractionWhileAwake()
            }
            .frame(width: 0, height: 0)
            .allowsHitTesting(false)

            if blackoutController.isBlackScreenVisible {
                Color.black
                    .ignoresSafeArea()
                    .contentShape(Rectangle())
                    .gesture(
                        DragGesture(minimumDistance: 0)
                            .onChanged { _ in
                                blackoutController.wakeScreen()
                            }
                    )
            }
        }
        .onAppear {
            blackoutController.beginMonitoring()
        }
        .onChange(of: scenePhase) {
            if scenePhase == .active {
                blackoutController.beginMonitoring()
            } else {
                blackoutController.stopMonitoring()
            }
        }
    }
}

private struct WindowTouchMonitor: UIViewRepresentable {
    let onInteraction: () -> Void

    func makeUIView(context: Context) -> WindowTouchMonitorView {
        let view = WindowTouchMonitorView()
        view.onInteraction = onInteraction
        return view
    }

    func updateUIView(_ uiView: WindowTouchMonitorView, context: Context) {
        uiView.onInteraction = onInteraction
        uiView.installRecognizerIfNeeded()
    }
}

private final class WindowTouchMonitorView: UIView, UIGestureRecognizerDelegate {
    var onInteraction: (() -> Void)?

    private weak var observedWindow: UIWindow?
    private weak var interactionRecognizer: WindowTouchGestureRecognizer?

    override func didMoveToWindow() {
        super.didMoveToWindow()
        installRecognizerIfNeeded()
    }

    override func point(inside point: CGPoint, with event: UIEvent?) -> Bool {
        false
    }

    func installRecognizerIfNeeded() {
        guard let window else { return }
        guard observedWindow !== window || interactionRecognizer == nil else { return }

        uninstallRecognizer()

        let recognizer = WindowTouchGestureRecognizer()
        recognizer.cancelsTouchesInView = false
        recognizer.delaysTouchesBegan = false
        recognizer.delaysTouchesEnded = false
        recognizer.delegate = self
        recognizer.onTouchBegan = { [weak self] in
            self?.onInteraction?()
        }

        window.addGestureRecognizer(recognizer)
        observedWindow = window
        interactionRecognizer = recognizer
    }

    private func uninstallRecognizer() {
        guard let recognizer = interactionRecognizer else { return }
        observedWindow?.removeGestureRecognizer(recognizer)
        interactionRecognizer = nil
        observedWindow = nil
    }

    func gestureRecognizer(
        _ gestureRecognizer: UIGestureRecognizer,
        shouldRecognizeSimultaneouslyWith otherGestureRecognizer: UIGestureRecognizer
    ) -> Bool {
        true
    }

    deinit {
        uninstallRecognizer()
    }
}

private final class WindowTouchGestureRecognizer: UIGestureRecognizer {
    var onTouchBegan: (() -> Void)?

    private var hasHandledCurrentTouch = false

    override func touchesBegan(_ touches: Set<UITouch>, with event: UIEvent) {
        super.touchesBegan(touches, with: event)

        if !hasHandledCurrentTouch {
            hasHandledCurrentTouch = true
            onTouchBegan?()
        }

        state = .began
    }

    override func touchesMoved(_ touches: Set<UITouch>, with event: UIEvent) {
        super.touchesMoved(touches, with: event)
        state = state == .possible ? .began : .changed
    }

    override func touchesEnded(_ touches: Set<UITouch>, with event: UIEvent) {
        super.touchesEnded(touches, with: event)
        state = .ended
    }

    override func touchesCancelled(_ touches: Set<UITouch>, with event: UIEvent) {
        super.touchesCancelled(touches, with: event)
        state = .cancelled
    }

    override func reset() {
        super.reset()
        hasHandledCurrentTouch = false
    }
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

    @ViewBuilder
    private var rootView: some View {
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
                                    .onAppear   { streamManager.isCommandViewActive = true  }
                                    .onDisappear { streamManager.isCommandViewActive = false }
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

    var body: some Scene {
        WindowGroup {
            InactivityBlackoutContainer {
                rootView
            }
        }
    }
}
