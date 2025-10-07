# Logging Configuration Guide

This project uses a specialized logging system from `Utils.h` that allows fine-grained control over logging output for each class.

## How to Enable Logging

Each class has its own logging flag that can be defined to enable logging for that specific class. To enable logging, define the appropriate flag before including the class files.

### Available Log Flags

- `G_LOG_SAMPLE_BUFFER` - Enable logging for SampleBuffer class
- `G_LOG_AUTO_CALIBRATOR` - Enable logging for AutoCalibrator class  
- `G_LOG_WAVE_DETECTOR` - Enable logging for WaveDetector class
- `G_LOG_LED_BLINKER` - Enable logging for LedBlinker class
- `G_LOG_WAVE_APP` - Enable logging for WaveApp class

## Performance Considerations

- TRACE level logging can be very verbose and may impact performance
- DEBUG level is suitable for development but should be disabled in production
- INFO/WARN/ERROR levels have minimal performance impact
- Each class can be individually controlled to balance debugging needs with performance
