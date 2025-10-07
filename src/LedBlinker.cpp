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
}

void LedBlinker::tick(uint32_t currentTime) {
    if (blinking && (currentTime - blinkStartMs >= blinkDurationMs)) {
        digitalWrite(ledPin, LOW);
        blinking = false;
    }
}

void LedBlinker::stop() {
    if (blinking) {
        digitalWrite(ledPin, LOW);
        blinking = false;
    }
}
