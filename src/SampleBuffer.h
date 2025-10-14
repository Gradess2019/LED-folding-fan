#ifndef SAMPLEBUFFER_H
#define SAMPLEBUFFER_H

#include <Arduino.h>

class SampleBuffer {
private:
    static const int BUFFER_SIZE = 100;
    float samples[BUFFER_SIZE];
    int head;
    int count;
    bool isFull;

public:
    SampleBuffer();
    
    void push(float sample);
    int size() const;
    bool full() const;
    float getSample(int index) const;
    void clear();
    
    // Iterator-like access for analysis
    float operator[](int index) const;
};

#endif
