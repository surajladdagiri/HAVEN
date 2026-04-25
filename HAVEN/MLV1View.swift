import SwiftUI
import UIKit
import ARKit
import SceneKit
import Vision
import CoreML
import ImageIO
import Combine


// MARK: - MLV1View

struct MLV1View: View {
    @StateObject private var store = DetectionStore()

    var body: some View {
        ZStack {
            ARCameraView(store: store)
                .ignoresSafeArea()

            OverlayView(objects: store.objects)
                .ignoresSafeArea()
        }
        .navigationBarTitleDisplayMode(.inline)
    }
}

// MARK: - Overlay

struct OverlayView: View {
    let objects: [DetectedObject]

    var body: some View {
        GeometryReader { _ in
            ZStack(alignment: .topLeading) {
                ForEach(objects) { obj in
                    Path { p in p.addRect(obj.viewRect) }
                        .stroke(.green, lineWidth: 2)

                    VStack(alignment: .leading, spacing: 2) {
                        Text(obj.titleLine)
                            .font(.system(size: 14, weight: .semibold))

                        if let d = obj.distanceM {
                            Text(String(format: "%.2f m", d))
                                .font(.system(size: 13, weight: .medium))
                        } else {
                            Text("— m")
                                .font(.system(size: 13, weight: .medium))
                        }
                    }
                    .padding(6)
                    .background(.ultraThinMaterial)
                    .cornerRadius(8)
                    .position(
                        x: max(86, obj.viewRect.minX + 86),
                        y: max(26, obj.viewRect.minY + 18)
                    )
                }
            }
        }
        .allowsHitTesting(false)
    }
}

// MARK: - Models

final class DetectionStore: ObservableObject {
    @Published var objects: [DetectedObject] = []
}

struct DetectedObject: Identifiable {
    let id = UUID()
    let viewRect: CGRect
    let label: String
    let confidence: Float
    let distanceM: Float?

    var titleLine: String {
        let c = max(0, min(1, confidence))
        return String(format: "%@ (%.0f%%)", label, c * 100)
    }
}

// MARK: - AR View

struct ARCameraView: UIViewRepresentable {
    let store: DetectionStore

    func makeCoordinator() -> Coordinator { Coordinator(store: store) }

    func makeUIView(context: Context) -> ARSCNView {
        let view = ARSCNView(frame: .zero)
        view.scene = SCNScene()
        view.automaticallyUpdatesLighting = true
        view.session.delegate = context.coordinator
        context.coordinator.attach(view: view)
        context.coordinator.startARSession()
        return view
    }

    func updateUIView(_ uiView: ARSCNView, context: Context) {}

    static func dismantleUIView(_ uiView: ARSCNView, coordinator: Coordinator) {
        uiView.session.pause()
    }
}

// MARK: - Coordinator

final class Coordinator: NSObject, ARSessionDelegate {
    private weak var view: ARSCNView?
    private let store: DetectionStore

    // If you trained your own 1-class model, replace with: ["myClass"]
    private let classNames: [String] = COCO80.names

    private let visionQueue = DispatchQueue(label: "vision.queue", qos: .userInitiated)
    private var visionRequest: VNCoreMLRequest?

    private var modelInputSize: CGSize = CGSize(width: 640, height: 640)

    private var isProcessing = false
    private var frameCounter = 0
    private let runEveryNFrames = 2

    private var didLogOnce = false
    private var outputIsNMS6: Bool = false // detected at runtime

    init(store: DetectionStore) {
        self.store = store
        super.init()
        self.visionRequest = makeVisionRequest()
    }

    func attach(view: ARSCNView) { self.view = view }

    func startARSession() {
        guard let view else { return }
        let config = ARWorldTrackingConfiguration()
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            config.frameSemantics.insert(.sceneDepth)
        } else {
            print("⚠️ sceneDepth not supported.")
        }
        view.session.run(config, options: [.resetTracking, .removeExistingAnchors])
    }

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        frameCounter += 1
        guard frameCounter % runEveryNFrames == 0 else { return }
        guard !isProcessing else { return }
        guard let request = visionRequest else { return }
        guard let view else { return }

        isProcessing = true

        let pixelBuffer = frame.capturedImage
        let depthMap = frame.sceneDepth?.depthMap

        let interfaceOrientation = Self.currentInterfaceOrientation()
        let exifOrientation = Self.exifOrientation(for: interfaceOrientation)

        let viewportSize = view.bounds.size
        let displayTransform = frame.displayTransform(for: interfaceOrientation, viewportSize: viewportSize)

        visionQueue.async { [weak self] in
            guard let self else { return }

            let handler = VNImageRequestHandler(cvPixelBuffer: pixelBuffer,
                                                orientation: exifOrientation,
                                                options: [:])
            do {
                try handler.perform([request])
            } catch {
                print("Vision error:", error)
                self.finish(objects: [])
                return
            }

            let results = request.results ?? []

            // Path A: Vision-native object observations (only if model supports it)
            if let obs = results as? [VNRecognizedObjectObservation], !obs.isEmpty {
                let out = self.mapVisionObservations(obs,
                                                    exifOrientation: exifOrientation,
                                                    depthMap: depthMap,
                                                    viewportSize: viewportSize,
                                                    displayTransform: displayTransform)
                self.finish(objects: out)
                return
            }

            // Path B: raw feature observation (MLMultiArray)
            let featureObs = results.compactMap { $0 as? VNCoreMLFeatureValueObservation }
            let arrays = featureObs.compactMap { $0.featureValue.multiArrayValue }
            guard let first = arrays.first else {
                self.finish(objects: [])
                return
            }

            if !self.didLogOnce {
                self.didLogOnce = true
                print("✅ YOLO output shape:", first.shape.map { $0.intValue }, "dtype:", first.dataType.rawValue)
                print("✅ modelInputSize:", self.modelInputSize, "classes:", self.classNames.count)

                // Detect "NMS6" output (like [1,300,6] / [300,6] / [1,6,300])
                let shape = first.shape.map { $0.intValue }
                self.outputIsNMS6 = shape.contains(6)
                print("✅ outputIsNMS6:", self.outputIsNMS6)
            }

            // Oriented image size (Vision ran on oriented image)
            let nativeW = CVPixelBufferGetWidth(pixelBuffer)
            let nativeH = CVPixelBufferGetHeight(pixelBuffer)
            let orientedSize: CGSize = {
                switch exifOrientation {
                case .left, .right: return CGSize(width: nativeH, height: nativeW)
                default: return CGSize(width: nativeW, height: nativeH)
                }
            }()

            // --- Decode detections ---
            let detections: [YOLODetection]
            if self.outputIsNMS6 {
                // Your case: [1,300,6]
                detections = YOLOPostProcessor.decodeNMS6(
                    arrays: arrays,
                    modelInputSize: self.modelInputSize,
                    orientedImageSize: orientedSize,
                    confThreshold: 0.30, // tweak up/down
                    applyLetterboxUndo: true // if boxes are shifted, set false
                )
            } else {
                // fallback (raw-head models like [1,84,8400]) – not your current shape
                detections = []
            }

            // Map to overlay objects + depth
            var out: [DetectedObject] = []
            out.reserveCapacity(detections.count)

            for det in detections {
                let rectNativeTL = Self.orientedRectToNativeTopLeft(det.rectOrientedTL, exif: exifOrientation)

                let distanceM: Float? = depthMap.flatMap {
                    DepthSampler.medianDepthMeters(in: $0, bboxNativeTopLeft: rectNativeTL)
                }

                let viewRect = Self.nativeNormalizedRectToViewRect(rectNativeTL,
                                                                  viewportSize: viewportSize,
                                                                  displayTransform: displayTransform)

                // Filter nonsense
                if viewRect.width < 10 || viewRect.height < 10 { continue }
                if viewRect.width > viewportSize.width * 0.98 || viewRect.height > viewportSize.height * 0.98 { continue }

                let label = self.classNames[safe: det.classIndex] ?? "class\(det.classIndex)"
                out.append(DetectedObject(viewRect: viewRect,
                                         label: label,
                                         confidence: det.score,
                                         distanceM: distanceM))
            }

            self.finish(objects: out)
        }
    }

    private func mapVisionObservations(_ observations: [VNRecognizedObjectObservation],
                                       exifOrientation: CGImagePropertyOrientation,
                                       depthMap: CVPixelBuffer?,
                                       viewportSize: CGSize,
                                       displayTransform: CGAffineTransform) -> [DetectedObject] {
        var out: [DetectedObject] = []
        out.reserveCapacity(observations.count)

        for obs in observations {
            guard let top = obs.labels.first else { continue }
            let label = top.identifier
            let conf = max(0, min(1, Float(top.confidence)))

            let rBL = obs.boundingBox
            let rectOrientedTL = CGRect(x: rBL.minX, y: 1.0 - rBL.maxY, width: rBL.width, height: rBL.height)

            let rectNativeTL = Self.orientedRectToNativeTopLeft(rectOrientedTL, exif: exifOrientation)
            let distanceM: Float? = depthMap.flatMap { DepthSampler.medianDepthMeters(in: $0, bboxNativeTopLeft: rectNativeTL) }

            let viewRect = Self.nativeNormalizedRectToViewRect(rectNativeTL,
                                                              viewportSize: viewportSize,
                                                              displayTransform: displayTransform)
            if viewRect.width > 10, viewRect.height > 10 {
                out.append(DetectedObject(viewRect: viewRect, label: label, confidence: conf, distanceM: distanceM))
            }
        }
        return out
    }

    private func finish(objects: [DetectedObject]) {
        DispatchQueue.main.async { [weak self] in
            self?.store.objects = objects
            self?.isProcessing = false
        }
    }

    private func makeVisionRequest() -> VNCoreMLRequest? {
        guard let url = Bundle.main.url(forResource: "yolo26n", withExtension: "mlmodelc") else {
            print("❌ Missing yolo26n.mlmodelc in bundle.")
            return nil
        }
        do {
            let mlModel = try MLModel(contentsOf: url)

            if let input = mlModel.modelDescription.inputDescriptionsByName.values.first,
               let ic = input.imageConstraint {
                self.modelInputSize = CGSize(width: ic.pixelsWide, height: ic.pixelsHigh)
            }

            let vnModel = try VNCoreMLModel(for: mlModel)
            let req = VNCoreMLRequest(model: vnModel)
            req.imageCropAndScaleOption = .scaleFit // matches YOLO letterbox style
            return req
        } catch {
            print("❌ CoreML load error:", error)
            return nil
        }
    }

    // MARK: Orientation + Mapping

    static func currentInterfaceOrientation() -> UIInterfaceOrientation {
        (UIApplication.shared.connectedScenes.first as? UIWindowScene)?.interfaceOrientation ?? .portrait
    }

    static func exifOrientation(for orientation: UIInterfaceOrientation) -> CGImagePropertyOrientation {
        switch orientation {
        case .portrait: return .right
        case .portraitUpsideDown: return .left
        case .landscapeLeft: return .up
        case .landscapeRight: return .down
        default: return .right
        }
    }

    // ORIENTED (top-left) -> NATIVE (top-left), normalized
    static func orientedRectToNativeTopLeft(_ rectO: CGRect, exif: CGImagePropertyOrientation) -> CGRect {
        let corners = [
            CGPoint(x: rectO.minX, y: rectO.minY),
            CGPoint(x: rectO.maxX, y: rectO.minY),
            CGPoint(x: rectO.minX, y: rectO.maxY),
            CGPoint(x: rectO.maxX, y: rectO.maxY)
        ].map { orientedPointToNativeTopLeft($0, exif: exif) }

        let xs = corners.map { $0.x }
        let ys = corners.map { $0.y }
        let minX = xs.min() ?? 0
        let maxX = xs.max() ?? 0
        let minY = ys.min() ?? 0
        let maxY = ys.max() ?? 0

        return CGRect(x: minX, y: minY, width: maxX - minX, height: maxY - minY)
            .intersection(CGRect(x: 0, y: 0, width: 1, height: 1))
    }

    static func orientedPointToNativeTopLeft(_ p: CGPoint, exif: CGImagePropertyOrientation) -> CGPoint {
        switch exif {
        case .right: return CGPoint(x: p.y, y: 1 - p.x)
        case .left:  return CGPoint(x: 1 - p.y, y: p.x)
        case .down:  return CGPoint(x: 1 - p.x, y: 1 - p.y)
        default:     return p
        }
    }

    static func nativeNormalizedRectToViewRect(_ rectN: CGRect,
                                              viewportSize: CGSize,
                                              displayTransform: CGAffineTransform) -> CGRect {
        let tl = CGPoint(x: rectN.minX, y: rectN.minY).applying(displayTransform)
        let br = CGPoint(x: rectN.maxX, y: rectN.maxY).applying(displayTransform)

        let x1 = tl.x * viewportSize.width
        let y1 = tl.y * viewportSize.height
        let x2 = br.x * viewportSize.width
        let y2 = br.y * viewportSize.height

        return CGRect(x: x1, y: y1, width: x2 - x1, height: y2 - y1).standardized
    }
}

// MARK: - Depth Sampling

enum DepthSampler {
    static func medianDepthMeters(in depthMap: CVPixelBuffer, bboxNativeTopLeft: CGRect) -> Float? {
        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }

        guard CVPixelBufferGetPixelFormatType(depthMap) == kCVPixelFormatType_DepthFloat32,
              let base = CVPixelBufferGetBaseAddress(depthMap) else { return nil }

        let w = CVPixelBufferGetWidth(depthMap)
        let h = CVPixelBufferGetHeight(depthMap)
        let rowBytes = CVPixelBufferGetBytesPerRow(depthMap)
        let strideFloats = rowBytes / MemoryLayout<Float>.size
        let ptr = base.assumingMemoryBound(to: Float.self)

        let rect = bboxNativeTopLeft.intersection(CGRect(x: 0, y: 0, width: 1, height: 1))

        let x0 = Int((rect.minX * CGFloat(w)).rounded(.down))
        let x1 = Int((rect.maxX * CGFloat(w)).rounded(.up))
        let y0 = Int((rect.minY * CGFloat(h)).rounded(.down))
        let y1 = Int((rect.maxY * CGFloat(h)).rounded(.up))
        guard x1 > x0, y1 > y0 else { return nil }

        let stepX = max(1, (x1 - x0) / 14)
        let stepY = max(1, (y1 - y0) / 14)

        var samples: [Float] = []
        samples.reserveCapacity(300)

        for y in stride(from: max(0, y0), to: min(h, y1), by: stepY) {
            let row = y * strideFloats
            for x in stride(from: max(0, x0), to: min(w, x1), by: stepX) {
                let d = ptr[row + x]
                if d.isFinite, d > 0.05, d < 20.0 { samples.append(d) }
            }
        }

        guard !samples.isEmpty else { return nil }
        samples.sort()
        return samples[samples.count / 2]
    }
}

// MARK: - YOLO Decode for NMS output (N,6)

struct YOLODetection {
    let rectOrientedTL: CGRect   // normalized 0..1, top-left origin in ORIENTED space
    let score: Float             // 0..1
    let classIndex: Int
}

enum YOLOPostProcessor {

    /// Decodes NMS-style output: typically [x1,y1,x2,y2,score,class] with shape like [1,300,6]
    static func decodeNMS6(
        arrays: [MLMultiArray],
        modelInputSize: CGSize,
        orientedImageSize: CGSize,
        confThreshold: Float,
        applyLetterboxUndo: Bool
    ) -> [YOLODetection] {

        var dets: [YOLODetection] = []
        dets.reserveCapacity(64)

        for a in arrays {
            let r = MultiArrayReader(a)
            let layout = NMS6Layout(r.shape)

            guard layout.isValid else { continue }

            for i in 0..<layout.N {
                let v0 = layout.get(r, i, 0)
                let v1 = layout.get(r, i, 1)
                let v2 = layout.get(r, i, 2)
                let v3 = layout.get(r, i, 3)
                var scoreRaw = layout.get(r, i, 4)
                let clsRaw = layout.get(r, i, 5)

                let score = normalizeProb(scoreRaw)
                if score < confThreshold { continue }

                let cls = Int(round(clsRaw))

                // Figure out whether coords are normalized or pixels
                let maxCoord = max(abs(v0), abs(v1), abs(v2), abs(v3))
                let coordsAreNormalized = maxCoord <= 2.0

                // Determine if it's xyxy or cxcywh
                let looksLikeXYXY = (v2 > v0) && (v3 > v1)

                var rectInputPx: CGRect
                if looksLikeXYXY {
                    var x1 = v0, y1 = v1, x2 = v2, y2 = v3
                    if coordsAreNormalized {
                        x1 *= Float(modelInputSize.width)
                        y1 *= Float(modelInputSize.height)
                        x2 *= Float(modelInputSize.width)
                        y2 *= Float(modelInputSize.height)
                    }
                    rectInputPx = CGRect(x: CGFloat(x1), y: CGFloat(y1),
                                         width: CGFloat(x2 - x1), height: CGFloat(y2 - y1)).standardized
                } else {
                    // assume cxcywh
                    var cx = v0, cy = v1, w = v2, h = v3
                    if coordsAreNormalized {
                        cx *= Float(modelInputSize.width)
                        cy *= Float(modelInputSize.height)
                        w  *= Float(modelInputSize.width)
                        h  *= Float(modelInputSize.height)
                    }
                    let x1 = cx - w / 2
                    let y1 = cy - h / 2
                    rectInputPx = CGRect(x: CGFloat(x1), y: CGFloat(y1),
                                         width: CGFloat(w), height: CGFloat(h)).standardized
                }

                // Basic sanity in input space
                if rectInputPx.width < 2 || rectInputPx.height < 2 { continue }
                if rectInputPx.width > modelInputSize.width * 0.99 || rectInputPx.height > modelInputSize.height * 0.99 { continue }

                let rectOrientedPx: CGRect
                if applyLetterboxUndo {
                    rectOrientedPx = undoLetterboxScaleFit(rectInputPx: rectInputPx,
                                                          inputSize: modelInputSize,
                                                          orientedSize: orientedImageSize)
                } else {
                    // If your model already outputs in original image space, use this:
                    rectOrientedPx = rectInputPx
                }

                let rectOrientedTL = CGRect(
                    x: rectOrientedPx.minX / orientedImageSize.width,
                    y: rectOrientedPx.minY / orientedImageSize.height,
                    width: rectOrientedPx.width / orientedImageSize.width,
                    height: rectOrientedPx.height / orientedImageSize.height
                ).intersection(CGRect(x: 0, y: 0, width: 1, height: 1))

                if rectOrientedTL.width < 0.005 || rectOrientedTL.height < 0.005 { continue }

                dets.append(YOLODetection(rectOrientedTL: rectOrientedTL, score: score, classIndex: cls))
            }
        }

        return dets
    }

    private static func normalizeProb(_ x: Float) -> Float {
        if x >= 0, x <= 1 { return x }
        return sigmoid(x)
    }

    private static func sigmoid(_ x: Float) -> Float {
        if x >= 10 { return 0.99995 }
        if x <= -10 { return 0.00005 }
        return 1 / (1 + exp(-x))
    }

    private static func undoLetterboxScaleFit(rectInputPx: CGRect, inputSize: CGSize, orientedSize: CGSize) -> CGRect {
        let inW = inputSize.width, inH = inputSize.height
        let imgW = orientedSize.width, imgH = orientedSize.height

        let scale = min(inW / imgW, inH / imgH)
        let newW = imgW * scale
        let newH = imgH * scale
        let padX = (inW - newW) / 2
        let padY = (inH - newH) / 2

        let x = (rectInputPx.minX - padX) / scale
        let y = (rectInputPx.minY - padY) / scale
        let w = rectInputPx.width / scale
        let h = rectInputPx.height / scale

        return CGRect(x: x, y: y, width: w, height: h).standardized
            .intersection(CGRect(x: 0, y: 0, width: imgW, height: imgH))
    }
}

// MARK: - NMS6 Layout Helper

struct NMS6Layout {
    let shape: [Int]
    let rank: Int
    let nAxis: Int
    let fAxis: Int
    let batchAxis: Int?
    let N: Int
    let isValid: Bool

    init(_ shape: [Int]) {
        self.shape = shape
        self.rank = shape.count

        var nAxisTmp = 0
        var fAxisTmp = 0
        var batchAxisTmp: Int? = nil
        var NTmp = 0
        var validTmp = false

        // Supported: [N,6], [6,N], [1,N,6], [1,6,N], etc.
        guard rank == 2 || rank == 3 else {
            self.nAxis = 0; self.fAxis = 0; self.batchAxis = nil; self.N = 0; self.isValid = false
            return
        }

        guard let f = shape.firstIndex(of: 6) else {
            self.nAxis = 0; self.fAxis = 0; self.batchAxis = nil; self.N = 0; self.isValid = false
            return
        }
        fAxisTmp = f

        if rank == 2 {
            nAxisTmp = (f == 0) ? 1 : 0
            NTmp = shape[nAxisTmp]
            validTmp = NTmp > 0
        } else {
            // Pick a batch axis if any dimension is 1
            batchAxisTmp = shape.firstIndex(of: 1)

            // Choose nAxis as the axis that's neither fAxis nor batchAxis
            if let b = batchAxisTmp {
                let others = [0, 1, 2].filter { $0 != f && $0 != b }
                nAxisTmp = others.first ?? ((f == 2) ? 1 : 2)
            } else {
                let others = [0, 1, 2].filter { $0 != f }
                nAxisTmp = others.first ?? ((f == 2) ? 1 : 2)
            }

            NTmp = shape[nAxisTmp]
            validTmp = NTmp > 0
        }

        self.nAxis = nAxisTmp
        self.fAxis = fAxisTmp
        self.batchAxis = batchAxisTmp
        self.N = NTmp
        self.isValid = validTmp
    }

    func get(_ r: MultiArrayReader, _ i: Int, _ f: Int) -> Float {
        if rank == 2 {
            var idx = [0, 0]
            idx[nAxis] = i
            idx[fAxis] = f
            return r.value(idx)
        } else {
            var idx = [0, 0, 0]
            if let b = batchAxis { idx[b] = 0 }
            idx[nAxis] = i
            idx[fAxis] = f
            return r.value(idx)
        }
    }
}

// MARK: - MultiArray Reader

struct MultiArrayReader {
    let shape: [Int]
    let strides: [Int]
    let dataType: MLMultiArrayDataType
    let ptr: UnsafeRawPointer

    init(_ a: MLMultiArray) {
        self.shape = a.shape.map { $0.intValue }
        self.strides = a.strides.map { $0.intValue }
        self.dataType = a.dataType
        self.ptr = UnsafeRawPointer(a.dataPointer)
    }

    func value(_ indices: [Int]) -> Float {
        var offset = 0
        for i in 0..<indices.count { offset += indices[i] * strides[i] }

        switch dataType {
        case .float32:
            return ptr.assumingMemoryBound(to: Float.self)[offset]
        case .double:
            return Float(ptr.assumingMemoryBound(to: Double.self)[offset])
        case .float16:
            let u = ptr.assumingMemoryBound(to: UInt16.self)[offset]
            return Float(Float16(bitPattern: u))
        default:
            return 0
        }
    }
}

// MARK: - COCO 80 labels

enum COCO80 {
    static let names: [String] = [
        "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat","traffic light",
        "fire hydrant","stop sign","parking meter","bench","bird","cat","dog","horse","sheep","cow",
        "elephant","bear","zebra","giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee",
        "skis","snowboard","sports ball","kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket","bottle",
        "wine glass","cup","fork","knife","spoon","bowl","banana","apple","sandwich","orange",
        "broccoli","carrot","hot dog","pizza","donut","cake","chair","couch","potted plant","bed",
        "dining table","toilet","tv","laptop","mouse","remote","keyboard","cell phone","microwave","oven",
        "toaster","sink","refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush"
    ]
}

extension Array {
    subscript(safe i: Int) -> Element? {
        (i >= 0 && i < count) ? self[i] : nil
    }
}
