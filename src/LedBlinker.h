#ifndef LEDBLINKER_H
#define LEDBLINKER_H

#include <Arduino.h>

class LedBlinker {
private:
    int ledPin;
    bool blinking;
    uint32_t blinkStartMs;
    uint32_t blinkDurationMs;
    
public:
    LedBlinker(int pin);
    
    void trigger(uint32_t durationMs);
    void tick(uint32_t currentTime);
    void stop();
    
    bool isBlinking() const { return blinking; }
};

#endif
