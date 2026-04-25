//
//  CommandView.swift
//  HAVEN
//
//  Created by Suraj Laddagiri  on 3/2/26.
//

import SwiftUI

// MARK: - App Home Screen
struct CommandView: View {
    @ObservedObject var appState: AppState
    @ObservedObject var blemanager: BLEManager
    
    init(appState: AppState, ble: BLEManager){
        self.appState = appState
        self.blemanager = ble
    }
    
    
    
    var body: some View {
            VStack{
                Image(systemName: "desktopcomputer.and.arrow.down")
                                .font(.largeTitle)
                                .padding(.top, 20)
                Text("Send Commands")
                Spacer()
                Button {
                    blemanager.sendCommand("0,0,0,0,0")
                } label:{
                    Text("(0,0,0,0,0)")
                }
                .frame(width: 300, height: 50)
                .background(Color.gray.opacity(0.2))
                .cornerRadius(30)
                
                Button {
                    blemanager.sendCommand("20,0,0,0,0")
                } label:{
                    Text("(20,0,0,0,0)")
                }
                .frame(width: 300, height: 50)
                .background(Color.gray.opacity(0.2))
                .cornerRadius(30)
                
                Button {
                    blemanager.sendCommand("0,20,0,0,0")
                } label:{
                    Text("(0,20,0,0,0)")
                }
                .frame(width: 300, height: 50)
                .background(Color.gray.opacity(0.2))
                .cornerRadius(30)
                
                Button {
                    blemanager.sendCommand("0,0,20,0,0")
                } label:{
                    Text("(0,0,20,0,0)")
                }
                .frame(width: 300, height: 50)
                .background(Color.gray.opacity(0.2))
                .cornerRadius(30)
                
                Button {
                    blemanager.sendCommand("0,0,0,20,0")
                } label:{
                    Text("(0,0,0,20,0)")
                }
                .frame(width: 300, height: 50)
                .background(Color.gray.opacity(0.2))
                .cornerRadius(30)
                
                Button {
                    blemanager.sendCommand("0,0,0,0,20")
                } label:{
                    Text("(0,0,0,0,20)")
                }
                .frame(width: 300, height: 50)
                .background(Color.gray.opacity(0.2))
                .cornerRadius(30)
                
                
                Button {
                    blemanager.sendCommand("20,20,0,0,0")
                } label:{
                    Text("(20,20,0,0,0)")
                }
                .frame(width: 300, height: 50)
                .background(Color.gray.opacity(0.2))
                .cornerRadius(30)
                
                Button {
                    blemanager.sendCommand("0,0,20,20,20")
                } label:{
                    Text("(0,0,20,20,20)")
                }
                .frame(width: 300, height: 50)
                .background(Color.gray.opacity(0.2))
                .cornerRadius(30)
                
                Button {
                    blemanager.sendCommand("50,50,50,50,50")
                } label:{
                    Text("(50,50,50,50,50)")
                }
                .frame(width: 300, height: 50)
                .background(Color.gray.opacity(0.2))
                .cornerRadius(30)
            }
        
        
    }
}


//#Preview {
//    CommandView()
//}

