#include "WaveDetector.h"

WaveDetector::WaveDetector() 
    : armed(true), lastEventMs(0), threshold(300.0f), hysteresis(200.0f) {
}

void WaveDetector::setThresholds(float newThreshold, float newHysteresis) {
    threshold = newThreshold;
    hysteresis = newHysteresis;
}

bool WaveDetector::detectWave(float magnitude, uint32_t currentTime) {
    if (armed && magnitude > threshold) {
        if (currentTime - lastEventMs > REFRACT_MS) {
            lastEventMs = currentTime;
            armed = false;
            return true;  // Wave detected!
        }
    } else if (!armed && magnitude < hysteresis) {
        armed = true; // re-arm below hysteresis
    }
    
    return false;  // No wave detected
}

void WaveDetector::reset() {
    armed = true;
    lastEventMs = 0;
}
