#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <Adafruit_DRV2605.h>
#include <ArduinoBLE.h>

#define MUX_ADDR 0x70
#define NUM_HAPTICS 5
const unsigned long UPDATE_INTERVAL = 25;

const char* SERVICE_UUID        = "54df84fc-7f55-4867-bb29-617f9d2a7925";
const char* CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
const int PORTS[NUM_HAPTICS] = {7, 6, 5, 4, 3};
QWIICMUX mux;
Adafruit_DRV2605 drv;

BLEService hapticService(SERVICE_UUID);
BLECharacteristic hapticChar(CHARACTERISTIC_UUID, BLERead | BLEWrite, NUM_HAPTICS);

int signals[NUM_HAPTICS] = {0};
unsigned long lastUpdate = 0;
bool portOK[NUM_HAPTICS] = {false};

const uint8_t AMP = 100; // DEBUG: force strong vibration

// ─────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("=== DEBUG HAVEN START ===");

    Wire1.begin();

    if (!mux.begin(MUX_ADDR, Wire1)) {
        Serial.println("MUX FAIL");
        while (1);
    }

    Serial.println("MUX OK");

    // Init DRV per port
    for (int i = 0; i < NUM_HAPTICS; i++) {

        int port = PORTS[i];   // map motor → mux port
        mux.setPort(port);

        if (drv.begin(&Wire1)) {
            drv.selectLibrary(1);
            drv.useERM(); // 🔥 IMPORTANT FIX
            drv.setMode(DRV2605_MODE_REALTIME);

            portOK[i] = true;
            Serial.print("Port "); Serial.print(i); Serial.println(" OK");
        } else {
            Serial.print("Port "); Serial.print(i); Serial.println(" FAIL");
        }
    }

    // startup test
    Serial.println("Startup vibration test...");
    for (int i = 0; i < NUM_HAPTICS; i++) signals[i] = 100;
    updateHaptics();
    delay(500);
    for (int i = 0; i < NUM_HAPTICS; i++) signals[i] = 0;
    updateHaptics();

    // BLE
    if (!BLE.begin()) {
        Serial.println("BLE FAIL");
        while (1);
    }

    BLE.setLocalName("Haptic_R4_Controller");
    BLE.setAdvertisedService(hapticService);
    hapticService.addCharacteristic(hapticChar);
    BLE.addService(hapticService);

    uint8_t zero[5] = {0,0,0,0,0};
    hapticChar.writeValue(zero, 5);

    BLE.advertise();
    Serial.println("BLE READY");
}

// ─────────────────────────────────────────────

void loop() {
    BLE.poll();
    BLEDevice central = BLE.central();

    if (central && central.connected()) {

        if (hapticChar.written()) {

            Serial.println("📩 BLE WRITE RECEIVED");

            const uint8_t* data = hapticChar.value();

            for (int i = 0; i < NUM_HAPTICS; i++) {
                signals[i] = data[i];
                Serial.print(signals[i]);
                Serial.print(" ");
            }
            Serial.println();
        }
    } else {
        for (int i = 0; i < NUM_HAPTICS; i++) signals[i] = 0;
    }

    if (millis() - lastUpdate > UPDATE_INTERVAL) {
        lastUpdate = millis();
        updateHaptics();
    }
}

// ─────────────────────────────────────────────

void updateHaptics() {
    for (int i = 0; i < NUM_HAPTICS; i++) {

        if (!portOK[i]) continue;

        int port = PORTS[i];
        mux.setPort(port);

        drv.begin(&Wire1);
        drv.useERM();
        drv.setMode(DRV2605_MODE_REALTIME);

        int val = constrain(signals[i], 0, 100);
        uint8_t out = (val > 0) ? AMP : 0;

        drv.setRealtimeValue(out);

        Serial.print("P"); Serial.print(port);
        Serial.print(":");
        Serial.print(out);
        Serial.print(" ");
    }
    Serial.println();
}