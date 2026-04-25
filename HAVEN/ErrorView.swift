//
//  ErrorView.swift
//  Music Strip
//
//  Created by Suraj Laddagiri  on 8/6/25.
//

import SwiftUI

struct ErrorView: View {
    var body: some View {
        VStack {
            Image(systemName: "exclamationmark.triangle.fill")
                .resizable()
                .frame(width:150, height:150)
                .scaledToFit()
                .padding()
            Text("Some Error Occured!")
                .font(.system(size: 34, weight: .bold, design: .default))
                .padding()
            Button("Quit"){
                exit(0)
            }.frame(width: 100, height:40)
            .background(Color.gray.opacity(0.3))
            .cornerRadius(20)
        }
        
    }
}


#Preview {
    ErrorView()
}
