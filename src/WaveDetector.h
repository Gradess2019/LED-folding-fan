#ifndef WAVEDETECTOR_H
#define WAVEDETECTOR_H

#include <Arduino.h>

class WaveDetector {
private:
    static const uint32_t REFRACT_MS = 200;  // debounce between waves
    
    bool armed;
    uint32_t lastEventMs;
    float threshold;
    float hysteresis;
    
public:
    WaveDetector();
    
    void setThresholds(float newThreshold, float newHysteresis);
    bool detectWave(float magnitude, uint32_t currentTime);
    void reset();
    
    bool isArmed() const { return armed; }
    float getThreshold() const { return threshold; }
    float getHysteresis() const { return hysteresis; }
};

#endif
