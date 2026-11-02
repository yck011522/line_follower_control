#pragma once

#include <Arduino.h>
#include <ble14_protocol.h>

struct RobotTelemetrySnapshot {
    uint8_t nfcUid[4]{};
    uint16_t nfcAgeMs = UINT16_MAX;
    uint8_t lineSensorMask = 0; // 1 = line under sensor, bit 7 = leftmost
    uint8_t controllerState = 0;
    uint8_t lineFollowerState = 0;
    int16_t leftSpeedMmS = 0;
    int16_t rightSpeedMmS = 0;
    uint16_t batteryMv = 0;
};

class BleTelemetry {
public:
    // Starts the Test 14-compatible BLE server and advertises this car's identity.
    bool begin(uint8_t carId);

    // Restarts advertising after a disconnect without blocking the control loop.
    void service(uint32_t nowMs);

    // Publishes one 20 Hz Robot State snapshot when a modem subscribed.
    void publishIfDue(uint32_t nowMs, uint32_t periodMs, const RobotTelemetrySnapshot& snapshot);

    // Copies the newest World State once for main-loop processing.
    bool takeWorldState(Ble14::World& world);

    // Returns whether this BLE connection has received a valid World State.
    bool hasWorldState() const;

    // Returns the age of the last World State, saturating at UINT32_MAX.
    uint32_t worldStateAgeMs(uint32_t nowMs) const;
};
