// ARKitVisualView.swift
// HAVEN — Mode 4: SLAM + 2D Map with Haptic Navigation Guidance
//
// Architecture:
//   NavigationEngine      — straight-line path (preferred) + A* fallback, proportional haptics
//   LiDARStreamManager    — ARKit delegate; occupancy grid; stable goal locking; pattern haptics
//   ARViewContainerTwo    — RealityKit bridge; shares arView with manager for 3D goal marker
//   OccupancyMapView      — 2D canvas: grid + path + goal + arrow
//   HapticIndicatorView   — 5-bar live motor display
//   ARKitVisualView       — Root SwiftUI view

import Combine
import SwiftUI
import ARKit
import RealityKit
import Network

// ─────────────────────────────────────────────────────────────────────────────
// MARK: - Grid Coordinate
// ─────────────────────────────────────────────────────────────────────────────

struct GridPoint: Hashable {
    let x: Int
    let z: Int
}

// ─────────────────────────────────────────────────────────────────────────────
// MARK: - Navigation Engine  (pure, stateless)
// ─────────────────────────────────────────────────────────────────────────────

enum NavigationEngine {

    static let gridSize: Float           = 0.05   // 5 cm per cell
    static let goalDistanceMeters: Float = 3.5    // target horizon

    // ── Passability ──────────────────────────────────────────────────────────

    static func isPassable(_ p: GridPoint, in grid: [GridPoint: Int]) -> Bool {
        guard let c = grid[p] else { return false }
        return c == 2 || c == 7
    }

    // ── Goal Selection ───────────────────────────────────────────────────────

    static func findGoal(
        grid: [GridPoint: Int],
        from start: GridPoint,
        facingYaw: Float
    ) -> GridPoint? {
        let goalCells = Int(goalDistanceMeters / gridSize)
        let fwdX = -sin(facingYaw)
        let fwdZ = -cos(facingYaw)

        let distTiers = [
            goalCells,
            Int(Float(goalCells) * 1.25),
            Int(Float(goalCells) * 0.75),
            Int(Float(goalCells) * 1.5),
            Int(Float(goalCells) * 0.5),
        ]
        let angles: [Float] = [
            0,
            -0.26, 0.26,
            -0.52, 0.52,
            -0.79, 0.79,
            -1.05, 1.05,
            -1.31, 1.31,
        ]

        for dist in distTiers {
            var best: (point: GridPoint, score: Float)?
            for angle in angles {
                let dx = fwdX * cos(angle) - fwdZ * sin(angle)
                let dz = fwdX * sin(angle) + fwdZ * cos(angle)
                let candidate = GridPoint(
                    x: start.x + Int(round(Float(dist) * dx)),
                    z: start.z + Int(round(Float(dist) * dz))
                )
                if isPassable(candidate, in: grid) {
                    let score = Float(dist) - abs(angle) * Float(goalCells) * 0.55
                    if best == nil || score > best!.score { best = (candidate, score) }
                }
            }
            if let b = best { return b.point }
        }
        return nil
    }

    // ── Straight-Line Path (always preferred over A*) ────────────────────────
    //
    // Walks a Bresenham line and checks a 3-cell-wide corridor.
    // Centre cells must be passable. Side cells that are *unknown* (not yet
    // scanned) are allowed — only confirmed obstacles on the sides fail the check.
    // This prevents A* from finding winding routes when the direct path is open,
    // and eliminates the left/right oscillation caused by A* re-planning.

    static func straightLinePath(
        from start: GridPoint,
        to goal: GridPoint,
        in grid: [GridPoint: Int]
    ) -> [GridPoint]? {
        let dx    = goal.x - start.x
        let dz    = goal.z - start.z
        let steps = max(abs(dx), abs(dz))
        guard steps > 0 else { return [start] }

        let len   = sqrt(Float(dx * dx + dz * dz))
        let perpX = Int(round(-Float(dz) / len))
        let perpZ = Int(round( Float(dx) / len))

        var path: [GridPoint] = []
        for i in 0...steps {
            let t  = Float(i) / Float(steps)
            let px = Int(round(Float(start.x) + t * Float(dx)))
            let pz = Int(round(Float(start.z) + t * Float(dz)))
            let center = GridPoint(x: px, z: pz)

            // Centre cell must be passable
            if !isPassable(center, in: grid) { return nil }

            // Side cells: only fail on *known* obstacles; unknown = ok
            let left  = GridPoint(x: px + perpX, z: pz + perpZ)
            let right = GridPoint(x: px - perpX, z: pz - perpZ)
            if let lc = grid[left],  lc != 2, lc != 7 { return nil }
            if let rc = grid[right], rc != 2, rc != 7 { return nil }

            path.append(center)
        }
        return path
    }

    // ── A* Pathfinding (fallback when no clear straight line exists) ──────────

    private struct ANode: Comparable {
        let pt: GridPoint; let g: Float; let h: Float
        var f: Float { g + h }
        static func < (a: ANode, b: ANode) -> Bool { a.f < b.f }
    }

    static func astar(
        grid: [GridPoint: Int],
        from start: GridPoint,
        to goal: GridPoint
    ) -> [GridPoint]? {
        var open   = [ANode(pt: start, g: 0, h: heuristic(start, goal))]
        var came   = [GridPoint: GridPoint](minimumCapacity: 1024)
        var gScore = [GridPoint: Float](minimumCapacity: 1024)
        gScore[start] = 0
        var closed = Set<GridPoint>(); closed.reserveCapacity(1024)

        let dirs: [(Int,Int)] = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(1,-1),(-1,1),(1,1)]
        var iters = 0
        while !open.isEmpty, iters < 8000 {
            iters += 1; open.sort()
            let cur = open.removeFirst()
            if cur.pt == goal { return reconstruct(came, end: goal) }
            guard !closed.contains(cur.pt) else { continue }
            closed.insert(cur.pt)
            for (ddx, ddz) in dirs {
                let nb = GridPoint(x: cur.pt.x + ddx, z: cur.pt.z + ddz)
                guard isPassable(nb, in: grid), !closed.contains(nb) else { continue }
                let cost: Float = (abs(ddx) + abs(ddz) == 2) ? 1.414 : 1.0
                let ng = cur.g + cost
                if ng < (gScore[nb] ?? .infinity) {
                    came[nb] = cur.pt; gScore[nb] = ng
                    open.append(ANode(pt: nb, g: ng, h: heuristic(nb, goal)))
                }
            }
        }
        return nil
    }

    private static func heuristic(_ a: GridPoint, _ b: GridPoint) -> Float {
        sqrt(Float((a.x-b.x)*(a.x-b.x) + (a.z-b.z)*(a.z-b.z)))
    }

    private static func reconstruct(_ came: [GridPoint: GridPoint], end: GridPoint) -> [GridPoint] {
        var path = [end]; var cur = end
        while let p = came[cur] { path.insert(p, at: 0); cur = p }
        return path
    }

    // ── Path → Desired World Heading ─────────────────────────────────────────

    static func desiredYaw(path: [GridPoint], from start: GridPoint, lookAhead: Int = 10) -> Float? {
        guard path.count > 1 else { return nil }
        let target = path[min(lookAhead, path.count - 1)]
        let dx = Float(target.x - start.x), dz = Float(target.z - start.z)
        guard abs(dx) > 0.001 || abs(dz) > 0.001 else { return nil }
        return atan2(-dx, -dz)
    }

    // ── Proportional Haptic Mapping ──────────────────────────────────────────
    //
    // 5 tactors span ±90° of navigable FOV (evenly spaced at 45° intervals):
    //
    //   idx:  [0]    [1]    [2]    [3]    [4]
    //        -90°   -45°    0°   +45°   +90°
    //        ◀◀      ◀      ▲      ▶     ▶▶
    //
    // The required turn angle maps to a float position [0..4].
    // Total intensity ≤ 100 is split linearly between the two bracketing tactors.
    //
    //   Examples:
    //     22.5° right →  pos=2.5 → m[2]=50, m[3]=50
    //     45°   right →  pos=3.0 → m[3]=100
    //     90°   right →  pos=4.0 → m[4]=100
    //     90°   left  →  pos=0.0 → m[0]=100
    //
    // ARKit sign: diff = desired − current
    //   diff > 0  →  turn LEFT  →  pos < 2
    //   diff < 0  →  turn RIGHT →  pos > 2
    //
    // Deadzone (< 7°): soft centre pulse (25) confirms on-track.

    static func hapticValues(desiredYaw: Float, currentYaw: Float) -> [Int] {
        var diff = desiredYaw - currentYaw
        while diff >  .pi { diff -= 2 * .pi }
        while diff < -.pi { diff += 2 * .pi }

        let deadzone: Float = 0.12   // ≈ 7°
        if abs(diff) < deadzone { return [0, 0, 25, 0, 0] }

        // Map diff ∈ [-π/2, +π/2] → position ∈ [0, 4]
        let fovHalf: Float = .pi / 2
        let pos = max(0.0, min(4.0, (-diff / fovHalf + 1.0) * 2.0))

        let lo   = min(3, Int(pos))
        let hi   = lo + 1
        let frac = pos - Float(lo)

        var m = [0, 0, 0, 0, 0]
        if frac < 0.005 {
            m[lo] = 100
        } else {
            m[hi] = Int(round(frac * 100))
            m[lo] = 100 - m[hi]
        }
        return m
    }

    // ── Door-Ahead Detection ──────────────────────────────────────────────────
    // Scans 0.5–3 m ahead in a ±15° arc. Returns true if any door cell (class 7)
    // is found in that cone.

    static func isDoorAhead(
        startGrid: GridPoint,
        facingYaw: Float,
        grid: [GridPoint: Int]
    ) -> Bool {
        let fwdX = -sin(facingYaw), fwdZ = -cos(facingYaw)
        let distancesM: [Float]  = [0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0]
        let sweepAngles: [Float] = [-0.26, -0.13, 0, 0.13, 0.26]

        for distM in distancesM {
            let cells = Int(distM / gridSize)
            for angle in sweepAngles {
                let dx = fwdX * cos(angle) - fwdZ * sin(angle)
                let dz = fwdX * sin(angle) + fwdZ * cos(angle)
                let pt = GridPoint(
                    x: startGrid.x + Int(round(Float(cells) * dx)),
                    z: startGrid.z + Int(round(Float(cells) * dz))
                )
                if grid[pt] == 7 { return true }
            }
        }
        return false
    }

    // ── Display Helper ────────────────────────────────────────────────────────

    static func smoothedPath(_ path: [GridPoint], step: Int = 4) -> [GridPoint] {
        guard path.count > 2 else { return path }
        var result = [path[0]]; var i = step
        while i < path.count - 1 { result.append(path[i]); i += step }
        result.append(path[path.count - 1])
        return result
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// MARK: - LiDAR Stream Manager
// ─────────────────────────────────────────────────────────────────────────────

class LiDARStreamManager: NSObject, ObservableObject, ARSessionDelegate {

    // ── Published state ───────────────────────────────────────────────────────
    @Published var connectionStatus = "Waiting for Mac..."
    @Published var occupancyGrid: [GridPoint: Int] = [:]
    @Published var currentPosition: CGPoint = .zero
    @Published var currentYaw: Float = 0.0
    @Published var plannedPath: [GridPoint] = []
    @Published var goalPoint: GridPoint? = nil
    @Published var hapticValues: [Int] = [0, 0, 0, 0, 0]

    let gridSize: Float = NavigationEngine.gridSize

    // ── Shared ARView — goal marker entities are added to this scene ──────────
    let arView = ARView(frame: .zero)
    private var goalAnchor: AnchorEntity? = nil
    // Last known camera world position — used to sanity-check mesh vertices
    private var lastCameraX: Float = 0.0
    private var lastCameraY: Float = 0.0
    private var lastCameraZ: Float = 0.0
    /// Mesh vertices further than this from the camera are discarded as tracking glitches.
    private let maxVertexDistance: Float = 20.0

    // ── BLE (injected; ObservedObject to match this project's pattern) ────────
    @ObservedObject var bleManager: BLEManager

    // ── TCP streaming ─────────────────────────────────────────────────────────
    var listener: NWListener?
    var activeConnection: NWConnection?
    let networkQueue = DispatchQueue(label: "com.haven.network")

    // ── Navigation state ──────────────────────────────────────────────────────
    private var lockedGoal: GridPoint? = nil
    private let goalRefreshRadiusCells: Float = 25.0    // ~1.25 m proximity
    private let minGoalRefreshInterval: TimeInterval = 8.0
    private var lastGoalRefreshTime: TimeInterval = -100
    private var storedDesiredYaw: Float? = nil

    // ── Special haptic patterns ───────────────────────────────────────────────
    //
    //  Door alert    : all 5 tactors at 60 → gap → repeat      (4 phases × 0.15 s)
    //  No-path alert : outermost pair [50,0,0,0,50] → gap × 3  (6 phases × 0.15 s)
    //
    private enum SpecialPattern { case idle, door, noPath }
    private var specialPattern: SpecialPattern = .idle
    private var specialPhase: Int = 0
    private var specialPhaseDeadline: TimeInterval = 0

    private var lastDoorAlertTime: TimeInterval = -100
    private let doorAlertCooldown: TimeInterval = 3.5

    private var lastNoPathAlertTime: TimeInterval = -100
    private let noPathAlertCooldown: TimeInterval = 4.0
    /// Timestamp of the first consecutive "no path" result — alert fires only after 2 s.
    private var noPathSince: TimeInterval? = nil
    private let noPathDebounce: TimeInterval = 2.0

    // ── Grid memory limits ────────────────────────────────────────────────────
    private var gridTimestamps: [GridPoint: TimeInterval] = [:]
    private let gridMaxAge: TimeInterval = 45.0
    private let gridMaxCells: Int = 12_000
    private var lastEvictionTime: TimeInterval = 0
    private let evictionInterval: TimeInterval = 5.0

    // ── Timing ────────────────────────────────────────────────────────────────
    private var lastAstarTime: TimeInterval = 0
    private let astarInterval: TimeInterval = 0.25      // 4 Hz path refresh (was 2 Hz)

    private var lastHapticTime: TimeInterval = 0
    private let hapticInterval: TimeInterval = 0.10     // 10 Hz haptic output

    private let navQueue = DispatchQueue(label: "com.haven.nav", qos: .userInitiated)

    // ── Camera blindspot floor-seed radius ────────────────────────────────────
    // iPhone LiDAR has a ~0.5 m near-field blindspot. Seed a disc of floor cells
    // under the user so A* always has a valid start cell on system warm-up.
    private let blindspotCells: Int = 10   // 10 × 5 cm = 0.5 m radius

    // ── Init ──────────────────────────────────────────────────────────────────
    init(ble: BLEManager) {
        self.bleManager = ble
        super.init()
        startNetworkServer()
    }

    // ── TCP Server ────────────────────────────────────────────────────────────
    func startNetworkServer() {
        do {
            listener = try NWListener(using: .tcp, on: 8080)
            listener?.newConnectionHandler = { [weak self] conn in
                DispatchQueue.main.async { self?.connectionStatus = "Mac Connected – Streaming" }
                self?.activeConnection = conn
                conn.start(queue: self!.networkQueue)
            }
            listener?.start(queue: networkQueue)
        } catch { print("TCP listener error: \(error)") }
    }

    // ── ARSession: Mesh Anchors ───────────────────────────────────────────────
    func session(_ session: ARSession, didUpdate anchors: [ARAnchor]) {
        let meshAnchors = anchors.compactMap { $0 as? ARMeshAnchor }
        guard !meshAnchors.isEmpty else { return }

        var localUpdates: [GridPoint: Int] = [:]
        var allStreamData = Data()

        for anchor in meshAnchors {
            guard let uuidData = anchor.identifier.uuidString.data(using: .utf8),
                  uuidData.count == 36 else { continue }

            var streamData = Data()
            streamData.append(uuidData)
            var anchorFloats: [Float32] = []

            let geo      = anchor.geometry
            let vertBuf  = geo.vertices.buffer.contents()
            let faceBuf  = geo.faces.buffer.contents()
            let classBuf = geo.classification?.buffer.contents()

            for faceIdx in 0..<geo.faces.count {
                var classID: Float32 = 0
                if let cb = classBuf, let cd = geo.classification {
                    let off = cd.offset + faceIdx * cd.stride
                    classID = Float32(cb.advanced(by: off)
                        .assumingMemoryBound(to: UInt8.self).pointee)
                }
                let faceBase = faceIdx * geo.faces.indexCountPerPrimitive * geo.faces.bytesPerIndex
                for j in 0..<geo.faces.indexCountPerPrimitive {
                    let idxOff  = faceBase + j * geo.faces.bytesPerIndex
                    let vertIdx = geo.faces.bytesPerIndex == 4
                        ? Int(faceBuf.advanced(by: idxOff).assumingMemoryBound(to: UInt32.self).pointee)
                        : Int(faceBuf.advanced(by: idxOff).assumingMemoryBound(to: UInt16.self).pointee)
                    let vOff = geo.vertices.offset + vertIdx * geo.vertices.stride
                    let v    = vertBuf.advanced(by: vOff)
                        .assumingMemoryBound(to: SIMD3<Float>.self).pointee
                    let w    = simd_mul(anchor.transform, simd_float4(v.x, v.y, v.z, 1.0))

                    // ── Sanity check: skip vertices that are suspiciously far from the
                    // camera. ARKit occasionally produces garbage transforms during
                    // tracking recovery that would plant phantom cells far away on the map.
                    let vdx = w.x - lastCameraX, vdz = w.z - lastCameraZ
                    if vdx*vdx + vdz*vdz > maxVertexDistance * maxVertexDistance { continue }

                    anchorFloats += [w.x, w.y, w.z, classID]

                    let cls = Int(classID)
                    if cls != 3 {
                        localUpdates[GridPoint(x: Int(round(w.x / gridSize)),
                                               z: Int(round(w.z / gridSize)))] = cls
                    }
                }
            }

            var count = Int32(anchorFloats.count).littleEndian
            streamData.append(Data(bytes: &count, count: 4))
            anchorFloats.withUnsafeBufferPointer { buf in
                streamData.append(buf.baseAddress!.withMemoryRebound(
                    to: UInt8.self, capacity: buf.count * 4) {
                        Data(buffer: UnsafeBufferPointer(start: $0, count: buf.count * 4))
                    })
            }
            allStreamData.append(streamData)
        }

        DispatchQueue.main.async { [weak self] in
            guard let self else { return }
            let now = CACurrentMediaTime()
            var g   = self.occupancyGrid
            for (pt, cls) in localUpdates {
                let ex = g[pt] ?? -1
                if      cls == 7                               { g[pt] = 7 }
                else if [0,1,4,5,6].contains(cls) && ex != 7  { g[pt] = 1 }
                else if cls == 2 && ![1,7].contains(ex)        { g[pt] = 2 }
                self.gridTimestamps[pt] = now
            }
            self.occupancyGrid = g
            if now - self.lastEvictionTime >= self.evictionInterval {
                self.lastEvictionTime = now
                self.evictOldCells(now: now)
            }
        }

        if let conn = activeConnection, conn.state == .ready, !allStreamData.isEmpty {
            conn.send(content: allStreamData, completion: .contentProcessed { _ in })
        }
    }

    // ── ARSession: Frame ──────────────────────────────────────────────────────
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        let tf  = frame.camera.transform
        let wx  = Float(tf.columns.3.x)
        let wy  = Float(tf.columns.3.y)
        let wz  = Float(tf.columns.3.z)
        let yaw = frame.camera.eulerAngles.y
        let now = CACurrentMediaTime()

        lastCameraX = wx
        lastCameraY = wy
        lastCameraZ = wz

        DispatchQueue.main.async { [weak self] in
            guard let self else { return }
            self.currentPosition = CGPoint(x: CGFloat(wx), y: CGFloat(wz))
            self.currentYaw = yaw

            // Seed floor cells in the LiDAR blindspot so A* has a valid start immediately
            let sg = GridPoint(x: Int(round(wx / self.gridSize)),
                               z: Int(round(wz / self.gridSize)))
            self.seedFloorIfNeeded(around: sg)
        }

        // Job 1: A* path refresh at 2 Hz
        if now - lastAstarTime >= astarInterval {
            lastAstarTime = now
            let posSnap = CGPoint(x: CGFloat(wx), y: CGFloat(wz))
            let yawSnap = yaw
            let camY    = wy
            DispatchQueue.main.async { [weak self] in
                guard let self else { return }
                let gridSnap   = self.occupancyGrid
                let lockedSnap = self.lockedGoal
                self.navQueue.async {
                    self.runNavigation(grid: gridSnap, worldPos: posSnap,
                                       yaw: yawSnap, cameraY: camY,
                                       lockedGoal: lockedSnap, now: now)
                }
            }
        }

        // Job 2: Haptic BLE output at 10 Hz
        // Must be on main because CBCentralManager was created on the main queue.
        if now - lastHapticTime >= hapticInterval {
            lastHapticTime = now
            let desired = storedDesiredYaw
            DispatchQueue.main.async { [weak self] in
                self?.tickHapticOutput(desiredYaw: desired, currentYaw: yaw, now: now)
            }
        }

        streamPose(x: wx, y: wy, z: wz)
    }

    // ── Floor Blindspot Seeding ───────────────────────────────────────────────
    // Seeds unknown cells within `blindspotCells` radius as floor (class 2),
    // only when the user's own cell is still nil (not yet scanned by LiDAR).
    // Called on main thread.
    private func seedFloorIfNeeded(around center: GridPoint) {
        guard occupancyGrid[center] == nil else { return }
        let r   = blindspotCells
        let now = CACurrentMediaTime()
        for dx in -r...r {
            for dz in -r...r {
                guard dx*dx + dz*dz <= r*r else { continue }
                let pt = GridPoint(x: center.x + dx, z: center.z + dz)
                if occupancyGrid[pt] == nil {
                    occupancyGrid[pt]  = 2
                    gridTimestamps[pt] = now
                }
            }
        }
    }

    // ── Haptic Output Tick (main thread, 10 Hz) ───────────────────────────────
    private func tickHapticOutput(desiredYaw: Float?, currentYaw: Float, now: TimeInterval) {
        // Special patterns (door / no-path) override navigation haptics
        if specialPattern != .idle {
            let vals = advanceSpecialPattern(now: now)
            hapticValues = vals.map { Int($0) }
            //bleManager.sendHapticValues(vals)
            let command = vals.map { String($0) }.joined(separator: ",")
            bleManager.sendCommand(command)
            return
        }

        let haptics: [Int] = {
            if let d = desiredYaw {
                return NavigationEngine.hapticValues(desiredYaw: d, currentYaw: currentYaw)
            }
            return [0, 0, 0, 0, 0]
        }()
        hapticValues = haptics
       // bleManager.sendHapticValues(haptics.map { UInt8(min(100, max(0, $0))) })
        let command = haptics.map { String(min(100, max(0, $0))) }.joined(separator: ",")
        bleManager.sendCommand(command)
    }

    // ── Special Pattern Engine ────────────────────────────────────────────────
    private func advanceSpecialPattern(now: TimeInterval) -> [UInt8] {
        guard now >= specialPhaseDeadline else {
            return patternOutput(specialPattern, phase: specialPhase)
        }
        specialPhase += 1
        specialPhaseDeadline = now + 0.15

        let maxPhase: Int
        switch specialPattern {
        case .door:   maxPhase = 4   // phases 0-3: buzz/gap/buzz/gap
        case .noPath: maxPhase = 6   // phases 0-5: (buzz/gap) × 3
        case .idle:   maxPhase = 0
        }

        if specialPhase >= maxPhase {
            specialPattern = .idle; specialPhase = 0
            return [0, 0, 0, 0, 0]
        }
        return patternOutput(specialPattern, phase: specialPhase)
    }

    private func patternOutput(_ p: SpecialPattern, phase: Int) -> [UInt8] {
        let buzz = (phase % 2 == 0)
        switch p {
        case .door:   return buzz ? [60, 60, 60, 60, 60] : [0, 0, 0, 0, 0]
        case .noPath: return buzz ? [50,  0,  0,  0, 50] : [0, 0, 0, 0, 0]
        case .idle:   return [0, 0, 0, 0, 0]
        }
    }

    private func triggerDoorAlert(now: TimeInterval) {
        guard now - lastDoorAlertTime  >= doorAlertCooldown,
              specialPattern == .idle else { return }
        lastDoorAlertTime    = now
        specialPattern       = .door
        specialPhase         = 0
        specialPhaseDeadline = now + 0.15
    }

    private func triggerNoPathAlert(now: TimeInterval) {
        // Start the debounce clock on first consecutive no-path result
        if noPathSince == nil { noPathSince = now }
        guard let since = noPathSince,
              now - since >= noPathDebounce,              // must be no-path for 2+ s
              now - lastNoPathAlertTime >= noPathAlertCooldown,
              specialPattern == .idle else { return }
        lastNoPathAlertTime  = now
        specialPattern       = .noPath
        specialPhase         = 0
        specialPhaseDeadline = now + 0.15
    }

    /// Call this whenever a valid path IS found to reset the debounce clock.
    private func clearNoPathDebounce() {
        noPathSince = nil
    }

    // ── Navigation Core (runs on navQueue) ────────────────────────────────────
    private func runNavigation(
        grid: [GridPoint: Int],
        worldPos: CGPoint,
        yaw: Float,
        cameraY: Float,
        lockedGoal: GridPoint?,
        now: TimeInterval
    ) {
        let startGrid = GridPoint(
            x: Int(round(Float(worldPos.x) / gridSize)),
            z: Int(round(Float(worldPos.y) / gridSize))
        )

        // Door detection — fires alert on main thread if door found ahead
        if NavigationEngine.isDoorAhead(startGrid: startGrid, facingYaw: yaw, grid: grid) {
            DispatchQueue.main.async { self.triggerDoorAlert(now: CACurrentMediaTime()) }
        }

        // ── Goal selection / retention ────────────────────────────────────────
        let goal: GridPoint

        if let locked = lockedGoal, shouldKeepGoal(locked, start: startGrid, grid: grid) {
            goal = locked
        } else {
            guard let newGoal = NavigationEngine.findGoal(
                grid: grid, from: startGrid, facingYaw: yaw) else {
                storedDesiredYaw = nil
                DispatchQueue.main.async { [weak self] in
                    guard let self else { return }
                    self.lockedGoal  = nil
                    self.goalPoint   = nil
                    self.plannedPath = []
                    self.update3DGoalMarker(to: nil, cameraY: cameraY)
                    self.triggerNoPathAlert(now: CACurrentMediaTime())
                }
                return
            }

            // Suppress goal flicker: if new goal is within 25° of the current
            // goal's direction AND we refreshed recently, keep the old goal.
            if let locked = lockedGoal,
               now - lastGoalRefreshTime < minGoalRefreshInterval {
                let oldAng = atan2(Float(locked.x  - startGrid.x), Float(locked.z  - startGrid.z))
                let newAng = atan2(Float(newGoal.x - startGrid.x), Float(newGoal.z - startGrid.z))
                var diff   = newAng - oldAng
                while diff >  .pi { diff -= 2 * .pi }
                while diff < -.pi { diff += 2 * .pi }
                goal = abs(diff) < 0.44 ? locked : newGoal   // ≈25° tolerance
            } else {
                goal = newGoal
            }

            if goal == newGoal {
                DispatchQueue.main.async { [weak self] in
                    self?.lockedGoal = newGoal
                    self?.lastGoalRefreshTime = now
                }
            }
        }

        // ── Pathfinding: straight line first, A* as fallback ─────────────────
        let rawPath: [GridPoint]

        if let line = NavigationEngine.straightLinePath(from: startGrid, to: goal, in: grid) {
            rawPath = line   // direct path is clear — always prefer this
        } else if let star = NavigationEngine.astar(grid: grid, from: startGrid, to: goal) {
            rawPath = star
        } else {
            // Goal is unreachable — force refresh next cycle and alert user
            storedDesiredYaw = nil
            DispatchQueue.main.async { [weak self] in
                guard let self else { return }
                self.lockedGoal  = nil
                self.plannedPath = []
                self.goalPoint   = goal
                self.update3DGoalMarker(to: nil, cameraY: cameraY)
                self.triggerNoPathAlert(now: CACurrentMediaTime())
            }
            return
        }

        storedDesiredYaw = NavigationEngine.desiredYaw(path: rawPath, from: startGrid)

        let displayPath = NavigationEngine.smoothedPath(rawPath, step: 4)
        DispatchQueue.main.async { [weak self] in
            guard let self else { return }
            self.clearNoPathDebounce()   // valid path found — reset debounce clock
            self.plannedPath = displayPath
            self.goalPoint   = goal
            self.update3DGoalMarker(to: goal, cameraY: cameraY)
        }
    }

    // ── 3D Goal Marker (main thread) ──────────────────────────────────────────
    // Removes the previous anchor and places a new yellow sphere at the goal's
    // world-space (X, Z) position.  Floor height estimated as cameraY − 1.2 m.
    private func update3DGoalMarker(to goal: GridPoint?, cameraY: Float) {
        goalAnchor?.removeFromParent()
        goalAnchor = nil
        guard let g = goal else { return }

        let wx = Float(g.x) * gridSize
        let wz = Float(g.z) * gridSize
        let wy = cameraY - 1.2

        let sphere = ModelEntity(
            mesh: MeshResource.generateSphere(radius: 0.12),
            materials: [SimpleMaterial(
                color: UIColor.yellow.withAlphaComponent(0.88),
                isMetallic: false
            )]
        )
        sphere.position = SIMD3<Float>(0, 0.20, 0)   // float slightly above floor

        let anchor = AnchorEntity(world: SIMD3<Float>(wx, wy, wz))
        anchor.addChild(sphere)
        arView.scene.addAnchor(anchor)
        goalAnchor = anchor
    }

    // ── Goal Retention Check ──────────────────────────────────────────────────
    private func shouldKeepGoal(_ goal: GridPoint, start: GridPoint, grid: [GridPoint: Int]) -> Bool {
        let dx = Float(goal.x - start.x), dz = Float(goal.z - start.z)
        if sqrt(dx*dx + dz*dz) < goalRefreshRadiusCells { return false }
        if !NavigationEngine.isPassable(goal, in: grid) { return false }

        for i in 1...8 {
            let t  = Float(i) / 9.0
            let px = Int(round(Float(start.x) + t * Float(goal.x - start.x)))
            let pz = Int(round(Float(start.z) + t * Float(goal.z - start.z)))
            if let cls = grid[GridPoint(x: px, z: pz)], cls != 2, cls != 7 { return false }
        }
        return true
    }

    // ── Grid Eviction (main thread) ───────────────────────────────────────────
    private func evictOldCells(now: TimeInterval) {
        let cutoff = now - gridMaxAge
        for k in gridTimestamps.filter({ $0.value < cutoff }).keys {
            occupancyGrid.removeValue(forKey: k)
            gridTimestamps.removeValue(forKey: k)
        }
        let overflow = occupancyGrid.count - gridMaxCells
        if overflow > 0 {
            for (k, _) in gridTimestamps.sorted(by: { $0.value < $1.value }).prefix(overflow) {
                occupancyGrid.removeValue(forKey: k)
                gridTimestamps.removeValue(forKey: k)
            }
        }
        if let g = lockedGoal, occupancyGrid[g] == nil { lockedGoal = nil }
    }

    // ── TCP Pose Streaming ────────────────────────────────────────────────────
    private func streamPose(x: Float, y: Float, z: Float) {
        guard let conn = activeConnection, conn.state == .ready else { return }
        var data = Data()
        data.append("POSE--------------------------------".data(using: .utf8)!)
        var cnt = Int32(3).littleEndian
        data.append(Data(bytes: &cnt, count: 4))
        var arr: [Float32] = [x, y, z]
        arr.withUnsafeBufferPointer { buf in
            data.append(buf.baseAddress!.withMemoryRebound(to: UInt8.self, capacity: 12) {
                Data(buffer: UnsafeBufferPointer(start: $0, count: 12))
            })
        }
        conn.send(content: data, completion: .contentProcessed { _ in })
    }

    // ── Cleanup ───────────────────────────────────────────────────────────────
    func pause() {
        storedDesiredYaw = nil
        specialPattern   = .idle
        noPathSince      = nil
        DispatchQueue.main.async { [weak self] in
            guard let self else { return }
            self.gridTimestamps.removeAll()
            self.update3DGoalMarker(to: nil, cameraY: 0)
            //self.bleManager.sendHapticValues([0, 0, 0, 0, 0])
            bleManager.sendCommand("0,0,0,0,0")
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// MARK: - AR View Bridge
// ─────────────────────────────────────────────────────────────────────────────
// Uses the shared arView from LiDARStreamManager so the 3D goal sphere entity
// lives in the same RealityKit scene as the camera feed.

struct ARViewContainerTwo: UIViewRepresentable {
    @ObservedObject var streamManager: LiDARStreamManager
    @Binding var showCameraFeed: Bool

    func makeUIView(context: Context) -> ARView {
        let v   = streamManager.arView   // ← shared view; 3D goal sphere added here
        let cfg = ARWorldTrackingConfiguration()
        if ARWorldTrackingConfiguration.supportsSceneReconstruction(.meshWithClassification) {
            cfg.sceneReconstruction = .meshWithClassification
        } else if ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) {
            cfg.sceneReconstruction = .mesh
        }
        v.debugOptions.insert(.showSceneUnderstanding)
        v.session.delegate = streamManager
        v.session.run(cfg)
        return v
    }

    func updateUIView(_ uiView: ARView, context: Context) {
        uiView.environment.background = showCameraFeed ? .cameraFeed() : .color(.black)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// MARK: - Occupancy + Navigation Map Canvas
// ─────────────────────────────────────────────────────────────────────────────

struct OccupancyMapView: View {
    @ObservedObject var streamManager: LiDARStreamManager

    var body: some View {
        Canvas { context, size in
            let grid   = streamManager.occupancyGrid
            let camPos = streamManager.currentPosition
            let path   = streamManager.plannedPath
            let goal   = streamManager.goalPoint
            let yaw    = streamManager.currentYaw

            var xs: [Float] = [Float(camPos.x)], zs: [Float] = [Float(camPos.y)]
            if !grid.isEmpty {
                xs.append(contentsOf: grid.keys.map { Float($0.x) * streamManager.gridSize })
                zs.append(contentsOf: grid.keys.map { Float($0.z) * streamManager.gridSize })
            }
            let minX  = xs.min() ?? 0, maxX = xs.max() ?? 0
            let minZ  = zs.min() ?? 0, maxZ = zs.max() ?? 0
            let spanX = max(maxX - minX, 5.0), spanZ = max(maxZ - minZ, 5.0)
            let pX = spanX * 0.12, pZ = spanZ * 0.12
            let scale = min(size.width  / CGFloat(spanX + pX*2),
                            size.height / CGFloat(spanZ + pZ*2))
            let offX = CGFloat(minX - pX), offZ = CGFloat(minZ - pZ)

            let toC: (Float, Float) -> CGPoint = { wx, wz in
                CGPoint(x: (CGFloat(wx) - offX) * scale,
                        y: (CGFloat(wz) - offZ) * scale)
            }
            let g2c: (GridPoint) -> CGPoint = { gp in
                toC(Float(gp.x) * streamManager.gridSize,
                    Float(gp.z) * streamManager.gridSize)
            }

            // 1. Occupancy cells
            let cellPx = CGFloat(streamManager.gridSize) * scale * 1.1
            for (pt, cls) in grid {
                let c = g2c(pt)
                let color: Color = cls == 2 ? Color(red:0.12, green:0.72, blue:0.12)
                                 : cls == 7 ? .orange
                                 : Color(red:0.72, green:0.12, blue:0.12)
                context.fill(Path(CGRect(x:c.x, y:c.y, width:cellPx, height:cellPx)),
                             with: .color(color))
            }

            // 2. Planned path
            if path.count > 1 {
                var line = Path(); line.move(to: g2c(path[0]))
                for i in 1..<path.count { line.addLine(to: g2c(path[i])) }
                context.stroke(line, with: .color(.cyan.opacity(0.25)),
                               style: StrokeStyle(lineWidth:8, lineCap:.round, lineJoin:.round))
                context.stroke(line, with: .color(.cyan.opacity(0.90)),
                               style: StrokeStyle(lineWidth:3, lineCap:.round, lineJoin:.round))
                for pt in path {
                    let c = g2c(pt)
                    context.fill(Path(ellipseIn:CGRect(x:c.x-3, y:c.y-3, width:6, height:6)),
                                 with: .color(.cyan))
                }
            }

            // 3. Goal marker
            if let g = goal {
                let gc = g2c(g)
                context.fill(Path(ellipseIn:CGRect(x:gc.x-13, y:gc.y-13, width:26, height:26)),
                             with: .color(.yellow.opacity(0.30)))
                context.stroke(Path(ellipseIn:CGRect(x:gc.x-10, y:gc.y-10, width:20, height:20)),
                               with: .color(.yellow.opacity(0.95)), lineWidth:2.5)
                context.fill(Path(ellipseIn:CGRect(x:gc.x-4, y:gc.y-4, width:8, height:8)),
                             with: .color(.white))
                for (dx, dy): (CGFloat, CGFloat) in [(-16,0),(16,0),(0,-16),(0,16)] {
                    context.stroke(Path { p in
                        p.move(to: CGPoint(x:gc.x, y:gc.y))
                        p.addLine(to: CGPoint(x:gc.x+dx, y:gc.y+dy))
                    }, with: .color(.yellow.opacity(0.8)), lineWidth:1.5)
                }
            }

            // 4. User position + facing arrow
            let userC   = toC(Float(camPos.x), Float(camPos.y))
            let aLen: CGFloat = 22
            let tip = CGPoint(x: userC.x - sin(CGFloat(yaw)) * aLen,
                              y: userC.y - cos(CGFloat(yaw)) * aLen)
            context.stroke(Path { p in p.move(to:userC); p.addLine(to:tip) },
                           with: .color(.white),
                           style: StrokeStyle(lineWidth:3, lineCap:.round))
            let pLen: CGFloat = 6
            let pPX = cos(CGFloat(yaw)) * pLen, pPY = -sin(CGFloat(yaw)) * pLen
            context.fill(Path { p in
                p.move(to: tip)
                p.addLine(to: CGPoint(x: userC.x - sin(CGFloat(yaw))*(aLen-10)+pPX,
                                      y: userC.y - cos(CGFloat(yaw))*(aLen-10)+pPY))
                p.addLine(to: CGPoint(x: userC.x - sin(CGFloat(yaw))*(aLen-10)-pPX,
                                      y: userC.y - cos(CGFloat(yaw))*(aLen-10)-pPY))
                p.closeSubpath()
            }, with: .color(.white))
            let r: CGFloat = 7
            context.fill(Path(ellipseIn:CGRect(x:userC.x-r, y:userC.y-r, width:r*2, height:r*2)),
                         with: .color(.blue))
            context.stroke(Path(ellipseIn:CGRect(x:userC.x-r, y:userC.y-r, width:r*2, height:r*2)),
                           with: .color(.white), lineWidth:2)
        }
        .background(Color(white: 0.11))
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// MARK: - Haptic Motor Indicator
// ─────────────────────────────────────────────────────────────────────────────

struct HapticIndicatorView: View {
    let values: [Int]
    private let labels = ["◀◀", "◀", "▲", "▶", "▶▶"]
    private let colors: [Color] = [.red, .orange, .green, .orange, .red]

    var body: some View {
        HStack(spacing: 7) {
            ForEach(0..<5, id: \.self) { i in
                VStack(spacing: 3) {
                    Text(labels[i])
                        .font(.system(size: 10, weight: .black))
                        .foregroundColor(colors[i])
                    ZStack(alignment: .bottom) {
                        RoundedRectangle(cornerRadius: 4).fill(Color.white.opacity(0.08))
                            .frame(width: 28, height: 48)
                        RoundedRectangle(cornerRadius: 4)
                            .fill(colors[i].opacity(values[i] > 0 ? 0.90 : 0.20))
                            .frame(width: 28, height: max(2, CGFloat(values[i]) / 100 * 48))
                            .animation(.easeOut(duration: 0.12), value: values[i])
                    }
                    Text("\(values[i])")
                        .font(.system(size: 8, weight: .medium, design: .monospaced))
                        .foregroundColor(.white.opacity(0.55))
                }
            }
        }
        .padding(.horizontal, 14)
        .padding(.vertical, 10)
        .background(.ultraThinMaterial)
        .cornerRadius(14)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// MARK: - Root View
// ─────────────────────────────────────────────────────────────────────────────

struct ARKitVisualView: View {
    @ObservedObject var streamManager: LiDARStreamManager
    @ObservedObject var bleManager: BLEManager

    @State private var showCameraFeed = true
    @State private var show2DMap = false

    init(bleManager: BLEManager) {
        self.streamManager  = LiDARStreamManager(ble: bleManager)
        self.bleManager     = bleManager
        self.showCameraFeed = true
        self.show2DMap      = false
    }

    var body: some View {
        ZStack(alignment: .bottom) {

            // Layer 1: AR camera feed (shares arView with manager for 3D goal sphere)
            ARViewContainerTwo(streamManager: streamManager, showCameraFeed: $showCameraFeed)
                .edgesIgnoringSafeArea(.all)

            // Layer 2: 2D navigation map
            if show2DMap {
                OccupancyMapView(streamManager: streamManager)
                    .edgesIgnoringSafeArea(.all)
                    .transition(.move(edge: .trailing))
                    .zIndex(1)
            }

            // Layer 3: Controls + status
            VStack(spacing: 10) {
                HapticIndicatorView(values: streamManager.hapticValues)

                HStack(spacing: 10) {
                    Button {
                        withAnimation(.easeInOut(duration: 0.28)) {
                            show2DMap.toggle()
                            showCameraFeed = !show2DMap
                        }
                    } label: {
                        Label(show2DMap ? "3D Camera" : "2D Map",
                              systemImage: show2DMap ? "camera.fill" : "map.fill")
                            .font(.subheadline.bold())
                            .padding(.horizontal, 14).padding(.vertical, 9)
                            .background(show2DMap ? Color.blue.opacity(0.8) : Color.teal.opacity(0.8))
                            .foregroundColor(.white)
                            .cornerRadius(10)
                    }

                    HStack(spacing: 5) {
                        Circle()
                            .fill(streamManager.goalPoint != nil ? Color.green : Color.yellow)
                            .frame(width: 8, height: 8)
                        Text(streamManager.goalPoint != nil ? "Navigating" : "Mapping…")
                            .font(.caption.bold())
                    }
                    .padding(.horizontal, 12).padding(.vertical, 9)
                    .background(Color.black.opacity(0.65))
                    .foregroundColor(.white)
                    .cornerRadius(10)

                    Text(bleManager.connected ? "BLE ✓" : "BLE –")
                        .font(.caption.bold())
                        .padding(.horizontal, 10).padding(.vertical, 9)
                        .background(Color.black.opacity(0.65))
                        .foregroundColor(bleManager.connected ? .green : .gray)
                        .cornerRadius(10)
                }
            }
            .padding(.bottom, 44)
            .zIndex(2)
        }
        .onAppear {
            streamManager.bleManager = bleManager
        }
        .onDisappear {
            streamManager.pause()
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// MARK: - Preview
// ─────────────────────────────────────────────────────────────────────────────

// Note: Preview requires an AppState mock because BLEManager's init needs AppState.
// Run on device for full functionality.
