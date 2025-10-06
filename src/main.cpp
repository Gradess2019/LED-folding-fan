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

// Continuous calibration parameters
#define SAMPLE_BUFFER_SIZE 100  // Rolling buffer of recent samples
#define MIN_SAMPLES_FOR_ANALYSIS 20  // Minimum samples needed for analysis
#define ANALYSIS_INTERVAL_MS 2000  // Recalculate thresholds every 2 seconds

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

// Continuous wave analysis function
void analyzeRecentSamples() {
    int sampleCount = bufferFull ? SAMPLE_BUFFER_SIZE : bufferIndex;
    if (sampleCount < MIN_SAMPLES_FOR_ANALYSIS) return;
    
    // Create working copy for sorting
    float workingSamples[SAMPLE_BUFFER_SIZE];
    for (int i = 0; i < sampleCount; i++) {
        workingSamples[i] = sampleBuffer[i];
    }
    
    // Simple bubble sort
    for (int i = 0; i < sampleCount - 1; i++) {
        for (int j = 0; j < sampleCount - i - 1; j++) {
            if (workingSamples[j] > workingSamples[j + 1]) {
                float temp = workingSamples[j];
                workingSamples[j] = workingSamples[j + 1];
                workingSamples[j + 1] = temp;
            }
        }
    }
    
    // Calculate percentiles from recent samples
    float p25 = workingSamples[(int)(sampleCount * 0.25)];  // 25th percentile
    float p50 = workingSamples[(int)(sampleCount * 0.50)];  // 50th percentile (median)
    float p75 = workingSamples[(int)(sampleCount * 0.75)];  // 75th percentile
    float p90 = workingSamples[(int)(sampleCount * 0.90)];  // 90th percentile
    
    // Calculate baseline noise (average of bottom 40%)
    float baselineNoise = 0;
    int baselineCount = (int)(sampleCount * 0.4);
    for (int i = 0; i < baselineCount; i++) {
        baselineNoise += workingSamples[i];
    }
    baselineNoise /= baselineCount;
    
    // Calculate recent activity (average of top 30%)
    float recentActivity = 0;
    int activityCount = (int)(sampleCount * 0.3);
    for (int i = sampleCount - activityCount; i < sampleCount; i++) {
        recentActivity += workingSamples[i];
    }
    recentActivity /= activityCount;
    
    // Adaptive threshold calculation
    // Use 75th percentile as base, but ensure it's above noise
    float newThreshold = max(p75, baselineNoise * 2.5f);
    newThreshold = min(newThreshold, recentActivity * 0.7f); // Don't go too high
    
    // Adaptive hysteresis calculation
    float newHysteresis = max(baselineNoise * 1.8f, newThreshold * 0.5f);
    newHysteresis = min(newHysteresis, newThreshold * 0.8f);
    
    // Smooth threshold changes to avoid sudden jumps
    float thresholdChange = newThreshold - THRESHOLD;
    float hysteresisChange = newHysteresis - HYSTERESIS;
    
    // Apply gradual changes (max 20% change per analysis)
    THRESHOLD += thresholdChange * 0.2f;
    HYSTERESIS += hysteresisChange * 0.2f;
    
    // Ensure reasonable bounds
    if (THRESHOLD < 15.0f) THRESHOLD = 15.0f;
    if (THRESHOLD > 1000.0f) THRESHOLD = 1000.0f;
    if (HYSTERESIS < 8.0f) HYSTERESIS = 8.0f;
    if (HYSTERESIS > THRESHOLD * 0.9f) HYSTERESIS = THRESHOLD * 0.9f;
    
    // Debug output (less frequent)
    static uint32_t lastDebugMs = 0;
    if (millis() - lastDebugMs > 5000) { // Every 5 seconds
        Serial.printf("Adaptive thresholds - Samples: %d, TH: %.1f, HY: %.1f\n", 
            sampleCount, THRESHOLD, HYSTERESIS);
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

	// Print in the same format as your working code
	// Serial.print("GYR |dps|: ");
	// Serial.println(magnitude_dps, 3);

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

	delay(50); // ~20 Hz print rate; sensors run at 100 Hz
}