#include <Arduino.h>
#include "WaveApp.h"

// Create the main application instance
WaveApp app;

void setup()
{
	app.setup();
}

void loop()
{
	app.loop();
}