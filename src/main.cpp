#include <Arduino.h>
#include "BMI160.h"

#define SDA_PIN 8
#define SCL_PIN 9
#define LED_PIN 4

// Create BMI160 sensor instance
BMI160 sensor(SDA_PIN, SCL_PIN, 400000);

// Wave detection thresholds (DPS - degrees per second) - will be calibrated dynamically
float THRESHOLD = 300.0f;     // Trigger threshold (calibrated)
float HYSTERESIS = 200.0f;    // Re-arm threshold (calibrated)
static const uint32_t REFRACT_MS = 300;  // debounce between waves

// Calibration parameters
static const uint32_t CALIBRATION_DURATION_MS = 10000;  // 10 seconds calibration
static const uint32_t CALIBRATION_SAMPLE_INTERVAL_MS = 50;  // Sample every 50ms

// Calibration state
bool calibrationMode = true;
uint32_t calibrationStartMs = 0;

// Wave analysis arrays
#define MAX_SAMPLES 200  // 10 seconds * 20 samples/second
float dpsSamples[MAX_SAMPLES];
int sampleIndex = 0;
bool samplesCollected = false;

bool armed = true;
uint32_t lastEventMs = 0;

// LED blink state
bool ledBlinking = false;
uint32_t ledBlinkStartMs = 0;
static const uint32_t LED_BLINK_DURATION_MS = 100;

void startCalibration() {
    calibrationMode = true;
    calibrationStartMs = millis();
    sampleIndex = 0;
    samplesCollected = false;
    Serial.println("=== CALIBRATION STARTED ===");
    Serial.println("Please move the sensor naturally for 10 seconds...");
    Serial.println("Make some gentle waves and movements to establish baseline.");
}

// Advanced wave analysis function
void analyzeWavePatterns() {
    if (sampleIndex < 10) return; // Need at least 10 samples
    
    // Sort samples for percentile analysis
    float sortedSamples[MAX_SAMPLES];
    for (int i = 0; i < sampleIndex; i++) {
        sortedSamples[i] = dpsSamples[i];
    }
    
    // Simple bubble sort
    for (int i = 0; i < sampleIndex - 1; i++) {
        for (int j = 0; j < sampleIndex - i - 1; j++) {
            if (sortedSamples[j] > sortedSamples[j + 1]) {
                float temp = sortedSamples[j];
                sortedSamples[j] = sortedSamples[j + 1];
                sortedSamples[j + 1] = temp;
            }
        }
    }
    
    // Calculate percentiles
    float p10 = sortedSamples[(int)(sampleIndex * 0.1)];  // 10th percentile (quiet periods)
    float p50 = sortedSamples[(int)(sampleIndex * 0.5)];  // 50th percentile (median)
    float p90 = sortedSamples[(int)(sampleIndex * 0.9)];  // 90th percentile (active periods)
    float p95 = sortedSamples[(int)(sampleIndex * 0.95)]; // 95th percentile (strong movements)
    
    // Calculate baseline noise (average of bottom 30%)
    float baselineNoise = 0;
    int baselineCount = (int)(sampleIndex * 0.3);
    for (int i = 0; i < baselineCount; i++) {
        baselineNoise += sortedSamples[i];
    }
    baselineNoise /= baselineCount;
    
    // Calculate wave activity (average of top 20%)
    float waveActivity = 0;
    int waveCount = (int)(sampleIndex * 0.2);
    for (int i = sampleIndex - waveCount; i < sampleIndex; i++) {
        waveActivity += sortedSamples[i];
    }
    waveActivity /= waveCount;
    
    // Smart threshold calculation
    // Use 75th percentile as base threshold (between median and high activity)
    float baseThreshold = sortedSamples[(int)(sampleIndex * 0.75)];
    
    // Ensure threshold is at least 2x baseline noise and not too high
    THRESHOLD = max(baseThreshold, baselineNoise * 2.0f);
    THRESHOLD = min(THRESHOLD, waveActivity * 0.8f); // Don't go too high
    
    // Hysteresis should be between baseline and threshold
    HYSTERESIS = max(baselineNoise * 1.5f, THRESHOLD * 0.4f);
    HYSTERESIS = min(HYSTERESIS, THRESHOLD * 0.7f);
    
    // Ensure reasonable minimums
    if (THRESHOLD < 20.0f) THRESHOLD = 20.0f;
    if (HYSTERESIS < 10.0f) HYSTERESIS = 10.0f;
    
    Serial.println("=== CALIBRATION COMPLETE ===");
    Serial.printf("Samples analyzed: %d\n", sampleIndex);
    Serial.printf("Baseline noise (bottom 30%%): %.2f DPS\n", baselineNoise);
    Serial.printf("Wave activity (top 20%%): %.2f DPS\n", waveActivity);
    Serial.printf("P10: %.2f, P50: %.2f, P90: %.2f, P95: %.2f\n", p10, p50, p90, p95);
    Serial.printf("Calculated THRESHOLD: %.2f DPS\n", THRESHOLD);
    Serial.printf("Calculated HYSTERESIS: %.2f DPS\n", HYSTERESIS);
    Serial.println("Ready for wave detection!");
}

// Calibration function
void performCalibration(float currentDps) {
    uint32_t now = millis();
    
    if (calibrationMode) {
        // Collect samples during calibration
        if (sampleIndex < MAX_SAMPLES) {
            dpsSamples[sampleIndex] = currentDps;
            sampleIndex++;
        }
        
        // Check if calibration is complete
        if (now - calibrationStartMs >= CALIBRATION_DURATION_MS) {
            analyzeWavePatterns();
            calibrationMode = false;
            // No re-calibration - calibration is complete
        }
    }
}

void setup()
{
	Serial.begin(115200);
	delay(2000);
	Serial.println("Wave trigger (BMI160 + ESP32-C3)");
	
	// Initialize LED pin
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, LOW);

	Serial.println("Initializing BMI160 sensor...");
	while (!sensor.begin())
	{
		Serial.println("ERROR: BMI160 not found or failed to initialize. Retrying...");
		delay(1000);
	}
	
	Serial.printf("BMI160 @0x%02X, CHIP_ID=0x%02X\n", sensor.getI2CAddress(), sensor.getChipID());

	uint8_t pmu = sensor.getPowerModeStatus();
	uint8_t err = sensor.getErrorRegister();
	Serial.printf("PMU_STATUS=0x%02X, ERR_REG=0x%02X\n", pmu, err);

	Serial.println("BMI160 configured: ACC=±4g @100Hz, GYR=±500dps @100Hz");
	
	// Start calibration
	startCalibration();
}

void loop()
{
	// Read gyroscope data and convert to DPS (exactly like your working code)
	int16_t gx, gy, gz;
	if (!sensor.readGyroscope(gx, gy, gz)) {
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
		return;
	}

	// Calculate magnitude in DPS (exactly like your working code)
	float magnitude_dps = sensor.getMagnitudeDPS(gx, gy, gz);

	// Perform calibration
	performCalibration(magnitude_dps);

	// Print in the same format as your working code
	// Serial.print("GYR |dps|: ");
	// Serial.println(magnitude_dps, 3);

	// Wave detection logic (only when not calibrating)
	uint32_t now = millis();

	if (!calibrationMode && armed && magnitude_dps > THRESHOLD)
	{
		if (now - lastEventMs > REFRACT_MS)
		{
			Serial.println("WAVE!");
			lastEventMs = now;
			armed = false;
			
			// Start LED blink (asynchronous) - always restart for new waves
			ledBlinking = true;
			ledBlinkStartMs = now;
			digitalWrite(LED_PIN, HIGH);
		}
	}
	else if (!calibrationMode && !armed && magnitude_dps < HYSTERESIS)
	{
		armed = true; // re-arm below hysteresis
	}
	else if (!calibrationMode) {
		// print every 1000 ms when no wave detected, use separate timer
		static uint32_t lastPrintMs = 0;
		if (now - lastPrintMs > 1000) {
			Serial.printf("No wave detected. GYR |dps|: %.2f (TH: %.2f, HY: %.2f)\n", 
				magnitude_dps, THRESHOLD, HYSTERESIS);
			lastPrintMs = now;
		}
	}
	else if (calibrationMode) {
		// Show calibration progress
		static uint32_t lastCalibPrintMs = 0;
		if (now - lastCalibPrintMs > 1000) {
			uint32_t elapsed = now - calibrationStartMs;
			uint32_t remaining = (CALIBRATION_DURATION_MS - elapsed) / 1000;
			
			// Calculate current average
			float currentAvg = 0;
			if (sampleIndex > 0) {
				for (int i = 0; i < sampleIndex; i++) {
					currentAvg += dpsSamples[i];
				}
				currentAvg /= sampleIndex;
			}
			
			Serial.printf("Calibrating... %lu seconds remaining. Samples: %d, Avg: %.2f DPS\n", 
				remaining, sampleIndex, currentAvg);
			lastCalibPrintMs = now;
		}
	}

	// Handle LED blink (asynchronous)
	if (ledBlinking && (now - ledBlinkStartMs >= LED_BLINK_DURATION_MS)) {
		digitalWrite(LED_PIN, LOW);
		ledBlinking = false;
	}

	delay(50); // ~20 Hz print rate; sensors run at 100 Hz
}