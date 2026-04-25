import SwiftUI
import ARKit
import RealityKit
import Combine
import CoreHaptics

// MARK: - V2 View Model
class V2Manager: NSObject, ObservableObject, ARSessionDelegate {
    @Published var isMapping: Bool = false
    @Published var distance: Float = 0.0
    @Published var radarDistances: [Float] = Array(repeating: 10.0, count: 37)
    @Published var compassNeedleAngle: Float = 0.0
    @Published var lockProgress: Float = 0.0
    
    // Logic State
    @Published var isLocked: Bool = false
    private var hasInitialLock = false
    private var lockedWorldYaw: Float = 0.0
    private var isAlignedWithLock = false
    
    // Timers & Tracking
    private var staringYaw: Float = 0.0
    private var stareDuration: TimeInterval = 0.0
    private var lastFrameTime: TimeInterval = 0.0
    
    let rayCount = 37
    let arView = ARView(frame: .zero)
    private var hapticEngine: CHHapticEngine?
    private var proximityPlayer: CHHapticAdvancedPatternPlayer?
    
    private let safetyThreshold: Float = 3.0 // 3 Meter Lock Requirement
    
    override init() {
        super.init()
        setupHaptics()
        setupAR()
    }
    
    private func setupAR() {
        arView.session.delegate = self
        let config = ARWorldTrackingConfiguration()
        if ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) {
            config.sceneReconstruction = .mesh
            arView.environment.sceneUnderstanding.options.insert(.physics)
            arView.debugOptions.insert(.showSceneUnderstanding)
        }
        config.environmentTexturing = .automatic
        arView.session.run(config)
    }
    
    // MARK: - Haptics System
    private func setupHaptics() {
        guard CHHapticEngine.capabilitiesForHardware().supportsHaptics else { return }
        do {
            hapticEngine = try CHHapticEngine()
            try hapticEngine?.start()
            
            // Create the continuous "Danger" vibration player (starts paused)
            let intensity = CHHapticEventParameter(parameterID: .hapticIntensity, value: 1.0)
            let sharpness = CHHapticEventParameter(parameterID: .hapticSharpness, value: 0.5)
            let event = CHHapticEvent(eventType: .hapticContinuous, parameters: [intensity, sharpness], relativeTime: 0, duration: 3600)
            let pattern = try CHHapticPattern(events: [event], parameters: [])
            proximityPlayer = try hapticEngine?.makeAdvancedPlayer(with: pattern)
        } catch {}
    }
    
    private func playAlignmentHaptic() {
        let event = CHHapticEvent(eventType: .hapticTransient, parameters: [
            CHHapticEventParameter(parameterID: .hapticIntensity, value: 0.8),
            CHHapticEventParameter(parameterID: .hapticSharpness, value: 1.0)
        ], relativeTime: 0)
        try? hapticEngine?.makePlayer(with: try CHHapticPattern(events: [event], parameters: [])).start(atTime: 0)
    }
    
    private func playRelockHaptic() {
        let intensity = CHHapticEventParameter(parameterID: .hapticIntensity, value: 1.0)
        let event1 = CHHapticEvent(eventType: .hapticTransient, parameters: [intensity], relativeTime: 0)
        let event2 = CHHapticEvent(eventType: .hapticTransient, parameters: [intensity], relativeTime: 0.1)
        try? hapticEngine?.makePlayer(with: try CHHapticPattern(events: [event1, event2], parameters: [])).start(atTime: 0)
    }
    
    // MARK: - Core Logic Loop
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        let now = CACurrentMediaTime()
        let dt = lastFrameTime == 0 ? 0 : now - lastFrameTime
        lastFrameTime = now
        
        let cameraTransform = frame.camera.transform
        let origin = simd_make_float3(cameraTransform.columns.3.x, cameraTransform.columns.3.y, cameraTransform.columns.3.z)
        let currentYaw = frame.camera.eulerAngles.y
        
        // FOV Scan
        let fov: Float = 120.0 * .pi / 180.0
        let startAngle: Float = fov / 2
        let endAngle: Float = -fov / 2
        let step = (endAngle - startAngle) / Float(rayCount - 1)
        
        var newDistances: [Float] = []
        let cameraQuat = simd_quatf(cameraTransform)
        
        for i in 0..<rayCount {
            let localAngle = startAngle + Float(i) * step
            let rayWorldDir = simd_act(cameraQuat * simd_quatf(angle: localAngle, axis: [0, 1, 0]), simd_make_float3(0, 0, -1))
            newDistances.append(raycastDistance(origin: origin, direction: rayWorldDir))
        }
        
        let centerDist = newDistances[rayCount / 2]
        
        // --- DANGER / LOCK LOGIC ---
        if centerDist < safetyThreshold {
            // DANGER: Vibrate and reset timer
            try? proximityPlayer?.start(atTime: 0)
            stareDuration = 0.0
        } else {
            // SAFE: Stop vibration and count down
            try? proximityPlayer?.stop(atTime: 0)
            
            let isStaringSteadily = abs(normalizeAngle(currentYaw - staringYaw)) < 0.2
            if isStaringSteadily {
                stareDuration += dt
                if stareDuration >= 5.0 {
                    lockedWorldYaw = currentYaw // Lock exactly where you are looking
                    stareDuration = 0.0
                    playRelockHaptic()
                    DispatchQueue.main.async { self.isLocked = true }
                }
            } else {
                staringYaw = currentYaw
                stareDuration = 0.0
            }
        }
        
        // Needle Calculation
        let rawNeedleAngle = normalizeAngle(lockedWorldYaw - currentYaw)
        
        // Alignment Haptic
        if abs(rawNeedleAngle) < 0.12 {
            if !isAlignedWithLock { isAlignedWithLock = true; playAlignmentHaptic() }
        } else if abs(rawNeedleAngle) > 0.2 { isAlignedWithLock = false }
        
        DispatchQueue.main.async {
            self.isMapping = true
            self.radarDistances = newDistances
            self.distance = centerDist
            self.lockProgress = min(Float(self.stareDuration) / 5.0, 1.0)
            let clampedNeedle = max(-fov/2, min(fov/2, -rawNeedleAngle))
            self.compassNeedleAngle = (self.compassNeedleAngle * 0.7) + (clampedNeedle * 0.3)
        }
    }
    
    private func raycastDistance(origin: SIMD3<Float>, direction: SIMD3<Float>) -> Float {
        let results = arView.scene.raycast(origin: origin, direction: direction, length: 10.0, query: .nearest, mask: .sceneUnderstanding)
        return results.first?.distance ?? 10.0
    }
    
    private func normalizeAngle(_ angle: Float) -> Float {
        var a = angle
        while a > .pi { a -= 2 * .pi }
        while a < -.pi { a += 2 * .pi }
        return a
    }
    
    func pauseSession() { arView.session.pause(); try? proximityPlayer?.stop(atTime: 0) }
}

// MARK: - UI
struct RadarSlice: Shape {
    var startAngle: Double; var endAngle: Double
    func path(in rect: CGRect) -> Path {
        var path = Path(); let center = CGPoint(x: rect.midX, y: rect.maxY)
        path.move(to: center); let offset = -Double.pi / 2
        path.addArc(center: center, radius: rect.width / 2, startAngle: .radians(startAngle + offset), endAngle: .radians(endAngle + offset), clockwise: false)
        return path
    }
}

struct ARKitV2View: View {
    @StateObject private var manager = V2Manager()
    var body: some View {
        ZStack {
            RealityKitViewContainer(arView: manager.arView).ignoresSafeArea()
            VStack {
                HStack {
                    Image(systemName: manager.distance < 3.0 ? "exclamationmark.triangle.fill" : "checkmark.circle.fill")
                        .foregroundColor(manager.distance < 3.0 ? .red : .green)
                    Text(manager.distance < 3.0 ? "Path Blocked (<3m)" : (manager.lockProgress > 0 ? "Locking in \(Int(5 - manager.lockProgress * 5))s..." : "Ready to Lock"))
                        .font(.caption).bold().foregroundColor(.white)
                }
                .padding(10).background(Color.black.opacity(0.7)).cornerRadius(10).padding(.top, 20)
                
                Spacer()
                
                Text(String(format: "%.2f m", manager.distance))
                    .font(.system(size: 28, weight: .black, design: .monospaced))
                    .foregroundColor(manager.distance < 3.0 ? .red : .green)
                    .padding().background(Color.black.opacity(0.7)).cornerRadius(12).padding(.bottom, 10)
                
                ZStack {
                    let sweepArc: Double = 120.0 * .pi / 180.0
                    let sliceArc = sweepArc / Double(manager.rayCount)
                    let startSweep = -sweepArc / 2
                    ForEach(0..<manager.rayCount, id: \.self) { i in
                        RadarSlice(startAngle: startSweep + Double(i) * sliceArc, endAngle: startSweep + Double(i+1) * sliceArc)
                            .fill(manager.radarDistances[i] < 3.0 ? Color.red : Color.green).opacity(0.8)
                    }
                    Rectangle().fill(Color.blue).frame(width: 6, height: 160).cornerRadius(3).offset(y: -80)
                        .rotationEffect(Angle(radians: Double(manager.compassNeedleAngle)), anchor: .bottom)
                    Circle().fill(Color.white).frame(width: 16, height: 16)
                }
                .frame(width: 320, height: 160).padding(.bottom, 40)
            }
        }
        .navigationTitle("V2: Safety Lock").onDisappear { manager.pauseSession() }
    }
}

struct RealityKitViewContainer: UIViewRepresentable {
    let arView: ARView
    func makeUIView(context: Context) -> ARView { return arView }
    func updateUIView(_ uiView: ARView, context: Context) {}
}
