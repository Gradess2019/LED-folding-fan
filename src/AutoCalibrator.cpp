#if !defined(G_LOG_AUTO_CALIBRATOR)
    #define LOG_DISABLE
#endif
#include <Utils.h>
#include "AutoCalibrator.h"
#include <math.h>

// Static constant definitions
const float AutoCalibrator::MIN_THRESHOLD = 100.0f;
const float AutoCalibrator::MAX_THRESHOLD = 1000.0f;
const float AutoCalibrator::MIN_HYSTERESIS = 50.0f;
const float AutoCalibrator::MAX_HYSTERESIS = 800.0f;
const float AutoCalibrator::BASELINE_PERCENTILE = 50.0f;
const float AutoCalibrator::ACTIVITY_PERCENTILE = 75.0f;
const float AutoCalibrator::ADAPTATION_RATE = 1.0f;

AutoCalibrator::AutoCalibrator() 
    : threshold(300.0f), hysteresis(200.0f), lastAnalysisMs(0), lastDebugMs(0) {
}

void AutoCalibrator::initialize() {
    lastAnalysisMs = millis();
    LOG_INFO("=== CONTINUOUS CALIBRATION STARTED ===");
    LOG_INFO("System will continuously adapt thresholds based on sensor data.");
    LOG_INFO("Move the sensor naturally to establish baseline patterns.");
}

void AutoCalibrator::update(const SampleBuffer& buffer) {
    uint32_t now = millis();
    
    // Check if it's time for analysis
    if (now - lastAnalysisMs < ANALYSIS_INTERVAL_MS) {
        return;
    }
    
    int sampleCount = buffer.size();
    if (sampleCount < MIN_SAMPLES_FOR_ANALYSIS) {
        LOGF_TRACE("Not enough samples for analysis (%d/%d)", sampleCount, MIN_SAMPLES_FOR_ANALYSIS);
        return;
    }
    
    LOGF_DEBUG("Starting analysis with %d samples", sampleCount);
    
    // Calculate statistics
    float mean, stdDev, p25, p50, p75, p90;
    calculateStatistics(buffer, sampleCount, mean, stdDev, p25, p50, p75, p90);
    
    // Update thresholds
    updateThresholds(buffer, sampleCount, mean, stdDev, p25, p50, p75, p90);
    
    lastAnalysisMs = now;
    
    // Debug output (less frequent)
    if (now - lastDebugMs > 5000) { // Every 5 seconds
        LOGF_INFO("Adaptive thresholds - Samples: %d, Mean: %.1f, StdDev: %.1f, TH: %.1f, HY: %.1f", 
            sampleCount, mean, stdDev, threshold, hysteresis);
        lastDebugMs = now;
    }
}

void AutoCalibrator::calculateStatistics(const SampleBuffer& buffer, int sampleCount, 
                                       float& mean, float& stdDev, float& p25, float& p50, 
                                       float& p75, float& p90) {
    // Calculate running statistics without sorting
    float sum = 0;
    float sumSquared = 0;
    float minVal = buffer[0];
    float maxVal = buffer[0];
    
    for (int i = 0; i < sampleCount; i++) {
        float val = buffer[i];
        sum += val;
        sumSquared += val * val;
        if (val < minVal) minVal = val;
        if (val > maxVal) maxVal = val;
    }
    
    // Calculate basic statistics
    mean = sum / sampleCount;
    float variance = (sumSquared / sampleCount) - (mean * mean);
    stdDev = sqrt(variance);
    
    // Estimate percentiles using mean and standard deviation
    p25 = mean - 0.67f * stdDev;  // Approximate 25th percentile
    p50 = mean;                   // 50th percentile (median approximation)
    p75 = mean + 0.67f * stdDev;  // Approximate 75th percentile
    p90 = mean + 1.28f * stdDev;  // Approximate 90th percentile
}

void AutoCalibrator::updateThresholds(const SampleBuffer& buffer, int sampleCount, 
                                    float mean, float stdDev, float p25, float p50, 
                                    float p75, float p90) {
    // Calculate baseline noise using recent low values
    float baselineNoise = 0;
    int baselineCount = 0;
    for (int i = 0; i < sampleCount; i++) {
        if (buffer[i] <= p50) {  // Use values below median as baseline
            baselineNoise += buffer[i];
            baselineCount++;
        }
    }
    if (baselineCount > 0) {
        baselineNoise /= baselineCount;
    } else {
        baselineNoise = p25;  // Fallback to estimated 25th percentile
    }
    
    // Calculate recent activity using recent high values
    float recentActivity = 0;
    int activityCount = 0;
    for (int i = 0; i < sampleCount; i++) {
        if (buffer[i] >= p75) {  // Use values above 75th percentile
            recentActivity += buffer[i];
            activityCount++;
        }
    }
    if (activityCount > 0) {
        recentActivity /= activityCount;
    } else {
        recentActivity = p90;  // Fallback to estimated 90th percentile
    }
    
    // Smart threshold calculation using statistical approach
    float newThreshold = max(p75, baselineNoise * 2.0f);
    newThreshold = min(newThreshold, recentActivity * 0.8f);
    
    // Adaptive hysteresis calculation
    float newHysteresis = max(baselineNoise * 1.5f, newThreshold * 0.4f);
    newHysteresis = min(newHysteresis, newThreshold * 0.8f);
    
    // Smooth threshold changes to avoid sudden jumps
    float thresholdChange = newThreshold - threshold;
    float hysteresisChange = newHysteresis - hysteresis;
    
    // Apply gradual changes using adaptation rate constant
    threshold += thresholdChange * ADAPTATION_RATE;
    hysteresis += hysteresisChange * ADAPTATION_RATE;
    
    // Ensure reasonable bounds
    if (threshold < MIN_THRESHOLD) {
        threshold = MIN_THRESHOLD;
        LOG_DEBUG("Threshold clamped to minimum");
    }
    if (threshold > MAX_THRESHOLD) {
        threshold = MAX_THRESHOLD;
        LOG_DEBUG("Threshold clamped to maximum");
    }
    if (hysteresis < MIN_HYSTERESIS) {
        hysteresis = MIN_HYSTERESIS;
        LOG_DEBUG("Hysteresis clamped to minimum");
    }
    if (hysteresis > MAX_HYSTERESIS) {
        hysteresis = MAX_HYSTERESIS;
        LOG_DEBUG("Hysteresis clamped to maximum");
    }
    if (hysteresis > threshold * 0.9f) {
        hysteresis = threshold * 0.9f;
        LOG_DEBUG("Hysteresis adjusted to maintain ratio");
    }
    
    LOGF_TRACE("Updated thresholds - TH: %.2f, HY: %.2f", threshold, hysteresis);
}
