#include "SampleBuffer.h"

SampleBuffer::SampleBuffer() : head(0), count(0), isFull(false) {
}

void SampleBuffer::push(float sample) {
    samples[head] = sample;
    head = (head + 1) % BUFFER_SIZE;
    
    if (!isFull) {
        count++;
        if (count >= BUFFER_SIZE) {
            isFull = true;
        }
    }
}

int SampleBuffer::size() const {
    return isFull ? BUFFER_SIZE : count;
}

bool SampleBuffer::full() const {
    return isFull;
}

float SampleBuffer::getSample(int index) const {
    if (index < 0 || index >= size()) {
        return 0.0f;
    }
    
    if (isFull) {
        // For full buffer, calculate actual index from head
        int actualIndex = (head - size() + index + BUFFER_SIZE) % BUFFER_SIZE;
        return samples[actualIndex];
    } else {
        return samples[index];
    }
}

void SampleBuffer::clear() {
    head = 0;
    count = 0;
    isFull = false;
}

float SampleBuffer::operator[](int index) const {
    return getSample(index);
}
