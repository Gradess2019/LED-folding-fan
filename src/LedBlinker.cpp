#if !defined(G_LOG_LED_BLINKER)
    #define LOG_DISABLE
#endif
#include <Utils.h>
#include "LedBlinker.h"

LedBlinker::LedBlinker(int pin) 
    : ledPin(pin), blinking(false), blinkStartMs(0), blinkDurationMs(0) {
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);
}

void LedBlinker::trigger(uint32_t durationMs) {
    blinking = true;
    blinkStartMs = millis();
    blinkDurationMs = durationMs;
    digitalWrite(ledPin, HIGH);
    LOGF_DEBUG("Triggered blink for %dms", durationMs);
}

void LedBlinker::tick(uint32_t currentTime) {
    if (blinking && (currentTime - blinkStartMs >= blinkDurationMs)) {
        digitalWrite(ledPin, LOW);
        blinking = false;
        LOG_TRACE("Blink completed");
    }
}

void LedBlinker::stop() {
    if (blinking) {
        digitalWrite(ledPin, LOW);
        blinking = false;
        LOG_DEBUG("Blink stopped manually");
    }
}
