#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <Adafruit_DRV2605.h>
#include <ArduinoBLE.h>

#define MUX_ADDR 0x70
#define NUM_HAPTICS 5
const unsigned long UPDATE_INTERVAL = 25;

// Maximum intensity sent to any haptic tactor (0–127 RTP scale).
// Hard-capped at 25 to prevent overdrive on the DRV2605L ERM motors.
const uint8_t MAX_HAPTIC_INTENSITY = 25;

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
            drv.useERM();
            drv.setMode(DRV2605_MODE_REALTIME);

            portOK[i] = true;
            Serial.print("Port "); Serial.print(i); Serial.println(" OK");
        } else {
            Serial.print("Port "); Serial.print(i); Serial.println(" FAIL");
        }
    }

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

        // NOTE: drv.begin() / useERM() / setMode() are one-time init calls —
        // they belong only in setup(). Re-calling them here every 25 ms was
        // resetting the driver state on each tick, causing latency and jitter.
        // Only switch the mux port and write the RTP value each update.

        // Clamp incoming signal to [0, MAX_HAPTIC_INTENSITY].
        // This ensures motors never exceed 25 regardless of what iOS sends.
        uint8_t out = (uint8_t)constrain(signals[i], 0, MAX_HAPTIC_INTENSITY);

        drv.setRealtimeValue(out);

        Serial.print("P"); Serial.print(port);
        Serial.print(":");
        Serial.print(out);
        Serial.print(" ");
    }
    Serial.println();
}