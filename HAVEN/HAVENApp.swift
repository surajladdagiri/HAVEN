//
//  HAVENApp.swift
//  HAVEN
//
//  Created by Suraj Laddagiri  on 4/4/26.
//

import SwiftUI
import ARKit

@main
struct HAVENApp: App {
    let isLiDARSupported = ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh)
    var body: some Scene {
        WindowGroup {
            //ContentView()
            if isLiDARSupported {
                LiDARView()
            } else {
                AIView()
            }
        }
    }
}
