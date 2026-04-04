//
//  AIView.swift
//  HAVEN
//
//  Created by Suraj Laddagiri  on 4/4/26.
//

import SwiftUI

struct AIView: View {
    var body: some View {
        VStack {
            Image(systemName: "exclamationmark.triangle.fill")
                .font(.system(size: 100))
                .foregroundStyle(.tint)
                .padding(50)
            Text("LiDAR Not Supported - Work in Progress")
        }
        .padding()
    }
}

#Preview {
    AIView()
}
