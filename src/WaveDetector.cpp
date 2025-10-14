#if !defined(G_LOG_WAVE_DETECTOR)
    #define LOG_DISABLE
#endif
#include <Utils.h>
#include "WaveDetector.h"

WaveDetector::WaveDetector() 
    : armed(true), lastEventMs(0), threshold(300.0f), hysteresis(200.0f) {
}

void WaveDetector::setThresholds(float newThreshold, float newHysteresis) {
    threshold = newThreshold;
    hysteresis = newHysteresis;
    LOGF_DEBUG("Thresholds updated - TH: %.2f, HY: %.2f", threshold, hysteresis);
}

bool WaveDetector::detectWave(float magnitude, uint32_t currentTime) {
    if (armed && magnitude > threshold) {
        if (currentTime - lastEventMs > REFRACT_MS) {
            lastEventMs = currentTime;
            armed = false;
            LOGF_INFO("WAVE DETECTED! Magnitude: %.2f > %.2f", magnitude, threshold);
            return true;  // Wave detected!
        } else {
            LOG_TRACE("Magnitude above threshold but in refractory period");
        }
    } else if (!armed && magnitude < hysteresis) {
        armed = true; // re-arm below hysteresis
        LOGF_DEBUG("Re-armed below hysteresis (%.2f < %.2f)", magnitude, hysteresis);
    }
    
    return false;  // No wave detected
}

void WaveDetector::reset() {
    armed = true;
    lastEventMs = 0;
    LOG_DEBUG("Reset - re-armed and cleared event time");
}
