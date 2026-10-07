#pragma once

#include <Arduino.h>
#include <Wire.h>

// Read the line sensor using the single-request procedure tested by E6.
// The caller initializes the bus and chooses when to request a reading.
class LineSensor
{
public:
    // Attach the caller's I2C bus without starting hardware during construction.
    explicit LineSensor(TwoWire &sensorBus) : sensorBus(sensorBus) {}

    // Perform one blocking register write and repeated-START byte read.
    // Return true for a complete byte; false covers every request failure.
    // On failure, the caller's rawReading remains unchanged.
    bool read(uint8_t &rawReading)
    {
        sensorBus.beginTransmission(0x12);
        sensorBus.write(0x30);
        // Preserve E6's repeated START: the write and read share one transaction.
        const uint8_t transmissionError = sensorBus.endTransmission(false);
        if (transmissionError != 0)
            return false;

        const size_t receivedByteCount = sensorBus.requestFrom(0x12, static_cast<size_t>(1), true);
        if (receivedByteCount == 1 && sensorBus.available() == 1)
        {
            rawReading = static_cast<uint8_t>(sensorBus.read());
            return true;
        }

        // Discard incomplete feedback exactly as E6 does before the next request.
        while (sensorBus.available())
            sensorBus.read();
        return false;
    }

private:
    // Reference to the I2C controller owned and configured by the application.
    TwoWire &sensorBus;
};
