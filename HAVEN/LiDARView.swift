//
//  LiDARView.swift
//  HAVEN
//
//  Created by Suraj Laddagiri  on 4/4/26.
//


import SwiftUI

struct LiDARView: View {
    var body: some View {
        VStack {
            Image(systemName: "checkmark.seal.fill")
                .font(.system(size: 100))
                .foregroundStyle(.tint)
                .padding(50)
            Text("LiDAR Supported")
        }
        .padding()
    }
}

#Preview {
    LiDARView()
}
