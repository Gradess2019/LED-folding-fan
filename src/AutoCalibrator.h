#ifndef AUTOCALIBRATOR_H
#define AUTOCALIBRATOR_H

#include <Arduino.h>
#include "SampleBuffer.h"

class AutoCalibrator {
private:
    static const int MIN_SAMPLES_FOR_ANALYSIS = 20;
    static const uint32_t ANALYSIS_INTERVAL_MS = 250;
    
    // Threshold and hysteresis bounds
    static const float MIN_THRESHOLD;
    static const float MAX_THRESHOLD;
    static const float MIN_HYSTERESIS;
    static const float MAX_HYSTERESIS;
    
    // Analysis parameters
    static const float BASELINE_PERCENTILE;
    static const float ACTIVITY_PERCENTILE;
    static const float ADAPTATION_RATE;
    
    float threshold;
    float hysteresis;
    uint32_t lastAnalysisMs;
    uint32_t lastDebugMs;
    
    void calculateStatistics(const SampleBuffer& buffer, int sampleCount, 
                           float& mean, float& stdDev, float& p25, float& p50, 
                           float& p75, float& p90);
    void updateThresholds(const SampleBuffer& buffer, int sampleCount, 
                        float mean, float stdDev, float p25, float p50, 
                        float p75, float p90);

public:
    AutoCalibrator();
    
    void initialize();
    void update(const SampleBuffer& buffer);
    
    float getThreshold() const { return threshold; }
    float getHysteresis() const { return hysteresis; }
};

#endif
