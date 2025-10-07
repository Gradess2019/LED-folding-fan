#include "WaveApp.h"

WaveApp::WaveApp() 
    : sensor(SDA_PIN, SCL_PIN, I2C_FREQ), 
      ledBlinker(LED_PIN), 
      lastPrintMs(0) {
}

bool WaveApp::initializeSensor() {
    Serial.println("Initializing BMI160 sensor...");
    while (!sensor.begin()) {
        Serial.println("ERROR: BMI160 not found or failed to initialize. Retrying...");
        delay(1000);
    }
    
    Serial.printf("BMI160 @0x%02X, CHIP_ID=0x%02X\n", sensor.getI2CAddress(), sensor.getChipID());

    uint8_t pmu = sensor.getPowerModeStatus();
    uint8_t err = sensor.getErrorRegister();
    Serial.printf("PMU_STATUS=0x%02X, ERR_REG=0x%02X\n", pmu, err);

    Serial.println("BMI160 configured: ACC=±4g @100Hz, GYR=±500dps @100Hz");
    return true;
}

void WaveApp::handleSensorError() {
    Serial.println("ERROR: Failed to read gyroscope data. Resetting sensor...");
    sensor.reset();
    delay(1000);
    
    // Re-initialize sensor after reset
    Serial.println("Re-initializing sensor after reset...");
    while (!sensor.begin()) {
        Serial.println("ERROR: BMI160 re-initialization failed. Retrying...");
        delay(1000);
    }
    Serial.println("Sensor re-initialized successfully");
}

void WaveApp::printDebugInfo(float magnitude, uint32_t now) {
    if (now - lastPrintMs > 1000) {
        Serial.printf("No wave detected. GYR |dps|: %.2f (TH: %.2f, HY: %.2f)\n", 
            magnitude, detector.getThreshold(), detector.getHysteresis());
        lastPrintMs = now;
    }
}

void WaveApp::setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("Wave trigger (BMI160 + ESP32-C3)");
    
    // Initialize sensor
    if (!initializeSensor()) {
        Serial.println("FATAL: Failed to initialize sensor");
        return;
    }
    
    // Start continuous calibration
    calibrator.initialize();
}

void WaveApp::loop() {
    // Read gyroscope data and convert to DPS
    int16_t gx, gy, gz;
    if (!sensor.readGyroscope(gx, gy, gz)) {
        handleSensorError();
        return;
    }

    // Calculate magnitude in DPS
    float magnitude_dps = sensor.getMagnitudeDPS(gx, gy, gz);

    // Collect sample and analyze continuously
    sampleBuffer.push(magnitude_dps);
    calibrator.update(sampleBuffer);

    // Update detector with current thresholds
    detector.setThresholds(calibrator.getThreshold(), calibrator.getHysteresis());

    // Wave detection logic
    uint32_t now = millis();
    
    if (detector.detectWave(magnitude_dps, now)) {
        Serial.println("WAVE!");
        
        // Start LED blink
        ledBlinker.trigger(LED_BLINK_DURATION_MS);
    } else {
        // Print debug info when no wave detected
        printDebugInfo(magnitude_dps, now);
    }

    // Handle LED blink (asynchronous)
    ledBlinker.tick(now);

    delay(10); // ~100 Hz update rate; sensors run at 100 Hz
}
