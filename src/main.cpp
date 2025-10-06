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
static const uint32_t REFRACT_MS = 200;  // debounce between waves

// Continuous calibration parameters
#define SAMPLE_BUFFER_SIZE 100  // Rolling buffer of recent samples
#define MIN_SAMPLES_FOR_ANALYSIS 20  // Minimum samples needed for analysis
#define ANALYSIS_INTERVAL_MS 250  // Recalculate thresholds every 0.25 seconds

// Threshold and hysteresis bounds
#define MIN_THRESHOLD 100.0f
#define MAX_THRESHOLD 1000.0f
#define MIN_HYSTERESIS 50.0f
#define MAX_HYSTERESIS 800.0f

// Analysis parameters
#define BASELINE_PERCENTILE 50.0f    // Use values below 50th percentile for baseline noise
#define ACTIVITY_PERCENTILE 75.0f    // Use values above 75th percentile for activity analysis
#define ADAPTATION_RATE 1.0f        // 100% change per analysis cycle for immediate adaptation

// Continuous calibration state
float sampleBuffer[SAMPLE_BUFFER_SIZE];
int bufferIndex = 0;
bool bufferFull = false;
uint32_t lastAnalysisMs = 0;

bool armed = true;
uint32_t lastEventMs = 0;

// LED blink state
bool ledBlinking = false;
uint32_t ledBlinkStartMs = 0;
static const uint32_t LED_BLINK_DURATION_MS = 100;

void initializeContinuousCalibration() {
    bufferIndex = 0;
    bufferFull = false;
    lastAnalysisMs = millis();
    Serial.println("=== CONTINUOUS CALIBRATION STARTED ===");
    Serial.println("System will continuously adapt thresholds based on sensor data.");
    Serial.println("Move the sensor naturally to establish baseline patterns.");
}

// Optimized wave analysis function - no sorting needed!
void analyzeRecentSamples() {
    int sampleCount = bufferFull ? SAMPLE_BUFFER_SIZE : bufferIndex;
    if (sampleCount < MIN_SAMPLES_FOR_ANALYSIS) return;
    
    // Calculate running statistics without sorting
    float sum = 0;
    float sumSquared = 0;
    float minVal = sampleBuffer[0];
    float maxVal = sampleBuffer[0];
    
    for (int i = 0; i < sampleCount; i++) {
        float val = sampleBuffer[i];
        sum += val;
        sumSquared += val * val;
        if (val < minVal) minVal = val;
        if (val > maxVal) maxVal = val;
    }
    
    // Calculate basic statistics
    float mean = sum / sampleCount;
    float variance = (sumSquared / sampleCount) - (mean * mean);
    float stdDev = sqrt(variance);
    
    // Estimate percentiles using mean and standard deviation
    // This is much faster than sorting and gives good approximations
    float p25 = mean - 0.67f * stdDev;  // Approximate 25th percentile
    float p50 = mean;                   // 50th percentile (median approximation)
    float p75 = mean + 0.67f * stdDev;  // Approximate 75th percentile
    float p90 = mean + 1.28f * stdDev;  // Approximate 90th percentile
    
    // Calculate baseline noise using recent low values
    // BASELINE: Sensor readings during quiet periods (no waves, just ambient movement)
    float baselineNoise = 0;
    int baselineCount = 0;
    for (int i = 0; i < sampleCount; i++) {
        if (sampleBuffer[i] <= p50) {  // Use values below median as baseline
            baselineNoise += sampleBuffer[i];
            baselineCount++;
        }
    }
    if (baselineCount > 0) {
        baselineNoise /= baselineCount;
    } else {
        baselineNoise = p25;  // Fallback to estimated 25th percentile
    }
    
    // Calculate recent activity using recent high values
    // RECENT ACTIVITY: Sensor readings during active periods (waves, movements, gestures)
    // activityCount: Number of samples that represent high activity (above 75th percentile)
    float recentActivity = 0;
    int activityCount = 0;
    for (int i = 0; i < sampleCount; i++) {
        if (sampleBuffer[i] >= p75) {  // Use values above 75th percentile
            recentActivity += sampleBuffer[i];
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
    float thresholdChange = newThreshold - THRESHOLD;
    float hysteresisChange = newHysteresis - HYSTERESIS;
    
    // Apply gradual changes using adaptation rate constant
    THRESHOLD += thresholdChange * ADAPTATION_RATE;
    HYSTERESIS += hysteresisChange * ADAPTATION_RATE;
    
    // Ensure reasonable bounds using constants
    if (THRESHOLD < MIN_THRESHOLD) THRESHOLD = MIN_THRESHOLD;
    if (THRESHOLD > MAX_THRESHOLD) THRESHOLD = MAX_THRESHOLD;
    if (HYSTERESIS < MIN_HYSTERESIS) HYSTERESIS = MIN_HYSTERESIS;
    if (HYSTERESIS > MAX_HYSTERESIS) HYSTERESIS = MAX_HYSTERESIS;
    if (HYSTERESIS > THRESHOLD * 0.9f) HYSTERESIS = THRESHOLD * 0.9f;
    
    // Debug output (less frequent)
    static uint32_t lastDebugMs = 0;
    if (millis() - lastDebugMs > 5000) { // Every 5 seconds
        Serial.printf("Adaptive thresholds - Samples: %d, Mean: %.1f, StdDev: %.1f, TH: %.1f, HY: %.1f\n", 
            sampleCount, mean, stdDev, THRESHOLD, HYSTERESIS);
        lastDebugMs = millis();
    }
}

// Continuous sample collection and analysis
void collectAndAnalyzeSample(float currentDps) {
    uint32_t now = millis();
    
    // Add sample to rolling buffer
    sampleBuffer[bufferIndex] = currentDps;
    bufferIndex++;
    if (bufferIndex >= SAMPLE_BUFFER_SIZE) {
        bufferIndex = 0;
        bufferFull = true;
    }
    
    // Analyze samples periodically
    if (now - lastAnalysisMs >= ANALYSIS_INTERVAL_MS) {
        analyzeRecentSamples();
        lastAnalysisMs = now;
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
	
	// Start continuous calibration
	initializeContinuousCalibration();
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

	// Collect sample and analyze continuously
	collectAndAnalyzeSample(magnitude_dps);

	// Wave detection logic
	uint32_t now = millis();

	if (armed && magnitude_dps > THRESHOLD)
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
	else if (!armed && magnitude_dps < HYSTERESIS)
	{
		armed = true; // re-arm below hysteresis
	}
	else {
		// print every 1000 ms when no wave detected, use separate timer
		static uint32_t lastPrintMs = 0;
		if (now - lastPrintMs > 1000) {
			Serial.printf("No wave detected. GYR |dps|: %.2f (TH: %.2f, HY: %.2f)\n", 
				magnitude_dps, THRESHOLD, HYSTERESIS);
			lastPrintMs = now;
		}
	}

	// Handle LED blink (asynchronous)
	if (ledBlinking && (now - ledBlinkStartMs >= LED_BLINK_DURATION_MS)) {
		digitalWrite(LED_PIN, LOW);
		ledBlinking = false;
	}

	delay(10); // ~100 Hz update rate; sensors run at 100 Hz
}