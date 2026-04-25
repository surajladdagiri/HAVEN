//
//  BluetoothView.swift
//  Music Strip
//
//  Created by Suraj Laddagiri  on 8/6/25.
//

import SwiftUI
import CoreBluetooth


struct BluetoothView: View {
    @ObservedObject var blemanager: BLEManager
    @State var Showerror = false
    @State var ErrorText = ""
    @State var scanning = false
    @ObservedObject var appState: AppState
    @State var connecting = false
    init(appState: AppState, ble: BLEManager){
        self.appState = appState
        self.blemanager = ble
    }
    var body: some View {
        
        
        
        if blemanager.FinishedAuto{
            VStack {
                if !connecting{
                    Image(systemName: "wave.3.up.circle.fill")
                        .scaleEffect(3)
                        .foregroundStyle(.tint)
                        .padding(.bottom)
                }
                
                if !scanning{
                    Button("Start Scan"){
                        do {
                            try blemanager.startScanning()
                        } catch BluetoothError.off{
                            ErrorText = "Bluetooth is off"
                            Showerror = true
                        } catch {
                            ErrorText = "Unknown error occured"
                            Showerror = true
                        }
                        
                        
                        withAnimation(.default){
                            scanning.toggle()
                        }
                        
                    }
                    .buttonStyle(.borderedProminent)
                    
                    Button("Skip"){
                        appState.currPage = .Algorithm
                    }
                    .buttonStyle(.borderedProminent)
                }else{
                    
                    
                    if !connecting{
                        Button("Stop Scan"){
                            blemanager.stopScanning()
                            blemanager.peripherals.removeAll()
                            withAnimation(.default){
                                scanning.toggle()
                            }
                        }
                        .buttonStyle(.borderedProminent)
                        List(blemanager.peripherals, id: \.identifier){ peripheral in
                            Button(peripheral.name ?? "Unnamed Device"){
                                blemanager.stopScanning()
                                withAnimation{
                                    connecting = true
                                }
                                blemanager.connect(to: peripheral)
                            }
                        }
                    }
                    
                    
                    
                }
                
                
            }
            .padding()
            .alert("Bluetooth Error", isPresented: $Showerror,actions: {
                Button("Quit"){
                    exit(0)
                }
                Button("OK"){
                    Showerror = false
                }
            }, message: {
                Text("\(ErrorText)")
            })
            if connecting{
                if !blemanager.connected{
                    ZStack{
                        Rectangle()
                            .fill(.gray)
                            .opacity(0.2)
                            .frame(width: 1000, height: 1000)
                        VStack{
                            ZStack{
                                Rectangle()
                                    .fill(.gray)
                                    .frame(width: 200, height: 200)
                                    .cornerRadius(20)
                                    .opacity(0.2)
                                ProgressView()
                                    .progressViewStyle(CircularProgressViewStyle())
                                    .scaleEffect(2.0)
                                    .tint(.blue)
                            }
                            Text("Connecting...")
                        }
                    }
                }else{
                    ZStack{
                        Rectangle()
                            .fill(.gray)
                            .opacity(0.2)
                            .frame(width: 1000, height: 1000)
                            .offset(x:-2000)
                        VStack{
                            ZStack{
                                Rectangle()
                                    .fill(.gray)
                                    .frame(width: 200, height: 200)
                                    .cornerRadius(20)
                                    .opacity(0.2)
                                    .offset(x:-2000)
                                ProgressView()
                                    .progressViewStyle(CircularProgressViewStyle())
                                    .scaleEffect(2.0)
                                    .tint(.blue)
                                    .offset(x:-2000)
                            }
                            Text("Connecting...")
                                .offset(x:-2000)
                        }
                    }
                }
            }

            
            
            
            
        }
        else{
            if !blemanager.connected{
                VStack {
                    ProgressView()
                        .progressViewStyle(CircularProgressViewStyle())
                        .scaleEffect(2.0)
                        .tint(.blue)
                    Text("Attempting to Auto Connect...")
                        .font(.system(size: 20, weight: .bold, design: .default))
                        .padding()
                    
                }
            }else{
                VStack {
                    ProgressView()
                        .progressViewStyle(CircularProgressViewStyle())
                        .scaleEffect(2.0)
                        .tint(.blue)
                        .offset(x:-1000)
                    Text("Attempting to Auto Connect...")
                        .font(.system(size: 20, weight: .bold, design: .default))
                        .padding()
                        .offset(x:-1000)
                }
            }
            
            
            
            
            
        }
        
    }
}


//#Preview {
//    var test: AppState = AppState()
//    var test2: BluetoothState = BluetoothState()
//    BluetoothView(appState: test)
//}
