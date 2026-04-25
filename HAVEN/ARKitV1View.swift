import SwiftUI
import Combine
import ARKit
import CoreHaptics

// MARK: - Navigation Direction
enum NavigationDirection {
    case straight
    case left
    case right
}

// MARK: - View Model (The Brains)
class ProximityManager: NSObject, ObservableObject, ARSessionDelegate {
    @Published var distance: Float = 0.0
    @Published var intensity: Double = 0.0
    @Published var suggestedDirection: NavigationDirection = .straight
    
    let arSession = ARSession()
    private var hapticEngine: CHHapticEngine?
    
    // --- NEW: Timer variables for the pulsing effect ---
    private var lastPulseTime: TimeInterval = 0
    private let maxPulseInterval: TimeInterval = 1.0 // Slowest speed: 1 tap per second (at 2.5m)
    private let minPulseInterval: TimeInterval = 0.1 // Fastest speed: 10 taps per second (at 0.3m)
    
    // Thresholds
    private let maxWarningDistance: Float = 2.5
    private let minCollisionDistance: Float = 0.3
    
    override init() {
        super.init()
        setupHaptics()
        setupARKit()
    }
    
    private func setupARKit() {
        arSession.delegate = self
        let configuration = ARWorldTrackingConfiguration()
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            configuration.frameSemantics = .sceneDepth
        }
        arSession.run(configuration)
    }
    
    private func setupHaptics() {
        guard CHHapticEngine.capabilitiesForHardware().supportsHaptics else { return }
        do {
            hapticEngine = try CHHapticEngine()
            try hapticEngine?.start()
            // Notice we removed the infinite pattern player here.
            // We will trigger individual taps on the fly instead.
        } catch {
            print("Haptics Setup Error: \(error)")
        }
    }
    
    // --- NEW: The single strong tap ---
    private func playPulse() {
        guard let engine = hapticEngine else { return }
        
        // Intensity is consistently locked at 1.0 (Maximum)
        let intensityParam = CHHapticEventParameter(parameterID: .hapticIntensity, value: 1.0)
        let sharpnessParam = CHHapticEventParameter(parameterID: .hapticSharpness, value: 0.8) // High sharpness for a distinct 'click'
        
        let event = CHHapticEvent(eventType: .hapticTransient, parameters: [intensityParam, sharpnessParam], relativeTime: 0)
        
        do {
            let pattern = try CHHapticPattern(events: [event], parameters: [])
            let player = try engine.makePlayer(with: pattern)
            try player.start(atTime: CHHapticTimeImmediate)
        } catch {
            print("Pulse error: \(error)")
        }
    }
    
    // 60 FPS Loop
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        guard let depthMap = frame.sceneDepth?.depthMap else { return }
        analyzePath(depthMap: depthMap)
    }
    
    private func analyzePath(depthMap: CVPixelBuffer) {
        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }
        
        let width = CVPixelBufferGetWidth(depthMap)
        let height = CVPixelBufferGetHeight(depthMap)
        let centerY = height / 2
        
        let leftX = width / 4
        let centerX = width / 2
        let rightX = (width / 4) * 3
        
        let baseAddress = CVPixelBufferGetBaseAddress(depthMap)!
        let bytesPerRow = CVPixelBufferGetBytesPerRow(depthMap)
        let buffer = baseAddress.assumingMemoryBound(to: Float32.self)
        
        let getDepth = { (x: Int, y: Int) -> Float in
            let index = (y * bytesPerRow / MemoryLayout<Float32>.stride) + x
            return buffer[index]
        }
        
        let leftDepth = getDepth(leftX, centerY)
        let centerDepth = getDepth(centerX, centerY)
        let rightDepth = getDepth(rightX, centerY)
        
        var newDirection: NavigationDirection = .straight
        
        if centerDepth < maxWarningDistance {
            if leftDepth > rightDepth {
                newDirection = .left
            } else {
                newDirection = .right
            }
            
            // --- NEW: Pulse Timing Logic ---
            let clampedDistance = max(minCollisionDistance, min(centerDepth, maxWarningDistance))
            
            // Normalize the distance between 0.0 (closest) and 1.0 (farthest)
            let distanceRatio = Double((clampedDistance - minCollisionDistance) / (maxWarningDistance - minCollisionDistance))
            
            // Calculate how long to wait before the next pulse
            let currentPulseInterval = minPulseInterval + (maxPulseInterval - minPulseInterval) * distanceRatio
            
            let now = CACurrentMediaTime()
            if now - lastPulseTime >= currentPulseInterval {
                playPulse()
                lastPulseTime = now
            }
        }
        
        // We still calculate this so the red screen overlay works smoothly
        let clampedDistance = max(minCollisionDistance, min(centerDepth, maxWarningDistance))
        let currentIntensity = 1.0 - Double((clampedDistance - minCollisionDistance) / (maxWarningDistance - minCollisionDistance))
        
        DispatchQueue.main.async {
            self.distance = centerDepth
            self.intensity = currentIntensity
            self.suggestedDirection = newDirection
        }
    }
}

// MARK: - ARKit to SwiftUI Bridge
struct ARViewContainer: UIViewRepresentable {
    let session: ARSession
    func makeUIView(context: Context) -> ARSCNView {
        let arView = ARSCNView(frame: .zero)
        arView.session = session
        return arView
    }
    func updateUIView(_ uiView: ARSCNView, context: Context) {}
}

// MARK: - The UI
struct ARKitV1View: View {
    @StateObject private var manager = ProximityManager()
    
    var body: some View {
        ZStack {
            ARViewContainer(session: manager.arSession)
                .ignoresSafeArea()
            
            Color.red
                .opacity(manager.intensity * 0.6)
                .ignoresSafeArea()
            
            // Directional Arrow UI
            VStack {
                Spacer()
                
                Image(systemName: arrowIcon(for: manager.suggestedDirection))
                    .resizable()
                    .scaledToFit()
                    .frame(width: 150, height: 150)
                    .foregroundColor(arrowColor(for: manager.suggestedDirection))
                    .shadow(radius: 10)
                    .animation(.spring(), value: manager.suggestedDirection)
                
                Spacer()
                
                Text(String(format: "%.2f meters", manager.distance))
                    .font(.system(size: 24, weight: .bold, design: .monospaced))
                    .foregroundColor(.white)
                    .padding()
                    .background(Color.black.opacity(0.6))
                    .cornerRadius(12)
                    .padding(.bottom, 40)
            }
        }
    }
    
    // Helper to pick the right SF Symbol
    private func arrowIcon(for direction: NavigationDirection) -> String {
        switch direction {
        case .straight: return "arrow.up.circle.fill"
        case .left: return "arrow.turn.up.left"
        case .right: return "arrow.turn.up.right"
        }
    }
    
    // Helper to make turns stand out
    private func arrowColor(for direction: NavigationDirection) -> Color {
        switch direction {
        case .straight: return Color.white.opacity(0.5) // Faded when path is clear
        case .left, .right: return Color.yellow // Bright yellow to indicate a turn is needed
        }
    }
}

#Preview {
    ARKitV1View()
}
