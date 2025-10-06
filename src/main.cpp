#include <Arduino.h>
#include "BMI160.h"

#define SDA_PIN 8
#define SCL_PIN 9

// Create BMI160 sensor instance
BMI160 sensor(SDA_PIN, SCL_PIN, 400000);

// Wave detection thresholds (DPS - degrees per second)
static const float THRESHOLD = 300.0f;     // Trigger threshold
static const float HYSTERESIS = 200.0f;    // Re-arm threshold
static const uint32_t REFRACT_MS = 300;  // debounce between waves

bool armed = true;
uint32_t lastEventMs = 0;

void setup()
{
	Serial.begin(115200);
	delay(2000);
	Serial.println("Wave trigger (BMI160 + ESP32-C3)");

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
	Serial.println("Ready. Flick the fan to see 'WAVE!'");
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
			Serial.printf("No wave detected. GYR |dps|: %.2f\n", magnitude_dps);
			lastPrintMs = now;
		}
	}

	delay(50); // ~20 Hz print rate; sensors run at 100 Hz
}