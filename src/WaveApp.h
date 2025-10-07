#ifndef WAVEAPP_H
#define WAVEAPP_H

#include <Arduino.h>
#include "BMI160.h"
#include "SampleBuffer.h"
#include "AutoCalibrator.h"
#include "WaveDetector.h"
#include "LedBlinker.h"

class WaveApp {
private:
    // Hardware configuration
    static const int SDA_PIN = 8;
    static const int SCL_PIN = 9;
    static const int LED_PIN = 4;
    static const uint32_t I2C_FREQ = 400000;
    static const uint32_t LED_BLINK_DURATION_MS = 100;
    
    // Components
    BMI160 sensor;
    SampleBuffer sampleBuffer;
    AutoCalibrator calibrator;
    WaveDetector detector;
    LedBlinker ledBlinker;
    
    // Debug timing
    uint32_t lastPrintMs;
    
    bool initializeSensor();
    void handleSensorError();
    void printDebugInfo(float magnitude, uint32_t now);

public:
    WaveApp();
    
    void setup();
    void loop();
};

#endif
