#pragma once

#include <cstddef>
#include <cstdint>

namespace Ble14 {
constexpr char SERVICE_UUID[] = "12345678-1234-5678-1234-567812345678";
constexpr char WORLD_UUID[] = "12345678-1234-5678-1234-567812345679";
constexpr char TELEMETRY_UUID[] = "12345678-1234-5678-1234-56781234567a";
constexpr uint8_t VERSION = 14;
constexpr size_t CARS = 4;
constexpr uint32_t WORLD_EXPIRY_MS = 1000;
constexpr uint32_t SCAN_PERIOD_MS = 5000;
constexpr uint32_t SCAN_DURATION_MS = 2000;
constexpr uint32_t WATCHDOG_MS = 12000;

#pragma pack(push, 1)
struct World {
    uint16_t sequence;
    uint8_t topology;
    uint8_t movement[4];
};
struct Robot {
    uint8_t car;
    uint16_t sequence;
    uint16_t worldSequence;
    uint8_t worldLostPrevious5s;
    uint8_t nfc[4];
    uint16_t nfcAge;
    uint8_t lineSensorMask; // E14: 1 = line under sensor, bit 7 = leftmost; E13 sends zero
    uint8_t controllerState;
    uint8_t lineFollowerState;
    int16_t leftSpeed;
    int16_t rightSpeed;
    uint16_t battery;
};
#pragma pack(pop)

static_assert(sizeof(World) == 7, "World State characteristic must remain seven bytes");
static_assert(sizeof(Robot) == 21, "Robot State notification must remain 21 bytes");

// Converts one ASCII hexadecimal character to its nibble value, or -1 if invalid.
inline int digit(char c) {
    if (c >= '0' && c <= '9') return c-'0';
    if (c >= 'a' && c <= 'f') return c-'a'+10;
    if (c >= 'A' && c <= 'F') return c-'A'+10;
    return -1;
}

// Validates and decodes the modem's W-prefixed seven-byte World State line.
inline bool decodeWorld(const char* line, size_t length, World& output) {
    if (length != 1+sizeof(World)*2 || line[0] != 'W') return false;
    auto* bytes = reinterpret_cast<uint8_t*>(&output);
    for (size_t i = 0; i < sizeof(World); ++i) {
        const int high = digit(line[1+i*2]), low = digit(line[2+i*2]);
        if (high < 0 || low < 0) return false;
        bytes[i] = uint8_t((high<<4)|low);
    }
    return true;
}

// Encodes one packed Robot State as lowercase hexadecimal followed by LF.
inline void encodeRobot(const Robot& packet, char* output) {
    constexpr char hex[] = "0123456789abcdef";
    const auto* bytes = reinterpret_cast<const uint8_t*>(&packet);
    for (size_t i = 0; i < sizeof(packet); ++i) {
        output[2*i] = hex[bytes[i]>>4];
        output[2*i+1] = hex[bytes[i]&15];
    }
    output[sizeof(packet)*2] = '\n';
}

// Performs wrap-safe uint32 deadline comparison for millis() timers.
inline bool due(uint32_t now, uint32_t deadline) {
    return int32_t(now-deadline) >= 0;
}
}
