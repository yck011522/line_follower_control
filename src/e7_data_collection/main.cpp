// E7: manually pushed car, raw line detection and encoder telemetry at 100 Hz.
// One loop reads both I2C buses and sends one ESP-NOW packet. No motion commands.
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <MotorDriver.h>
#include <MotorSettings.h>
#include <WiFi.h>
#include <Wire.h>
#include <atomic>
#include <cstring>
#include <esp_now.h>
#include <esp_timer.h>
#include <esp_random.h>

// =============================================================================
// CONFIGURATION - network credentials and car identity come from platformio.ini.
// =============================================================================

const uint64_t controlPeriodUs = 10000; // One schedule for acquisition and sending.
const uint32_t wifiConnectTimeoutMs = 20000;
const size_t maximumMessageLength = 250; // Existing master forwards one text line.

// Match the tested radio's static addressing and E6's two independent I2C buses.
const IPAddress staticIpAddress(STATIC_IP_ADDRESS);
const IPAddress gatewayAddress(GATEWAY_ADDRESS);
const IPAddress subnetMask(255, 255, 255, 0);
TwoWire &motorBus = Wire;
TwoWire sensorBus(1);
MotorDriver motorDriver(motorBus);

// =============================================================================
// STATE - only the radio callbacks share state with loop().
// =============================================================================

bool sensorBusReady = false;
bool motorBusReady = false;
bool motorConfigured = false;
bool radioReady = false;
bool otaInitialized = false;
bool otaInProgress = false;

// A single-element mailbox transfers discovery safely out of the Wi-Fi task.
// The first valid master's address is retained until the car reboots.
QueueHandle_t masterDiscoveryQueue = nullptr;
uint8_t masterMacAddress[6] = {};
bool masterKnown = false;
std::atomic<bool> telemetrySendPending{false};
std::atomic<uint32_t> telemetryDeliveryFailures{0};

// Sequence counts acquisition attempts, even before discovery or during TX loss.
// A random boot ID distinguishes counter/timestamp resets in a recording.
uint32_t bootIdentifier = 0;
uint32_t sampleSequence = 0;
uint64_t nextControlAtUs = 0;
uint32_t skippedControlSlots = 0;
uint32_t telemetryBusySkips = 0;
uint32_t telemetrySubmissionErrors = 0;
char telemetryMessage[maximumMessageLength + 1] = {};

// Each wheel keeps its own successful read time; failed reads never replace it.
// Raw counts and speeds retain the driver's electrical sign, without inversion.
const MotorDriver::Wheel encoderWheels[] = {MotorDriver::Wheel::Left, MotorDriver::Wheel::Right};
int32_t previousEncoderCounts[2] = {};
uint64_t previousEncoderReadAtUs[2] = {};
bool previousEncoderValid[2] = {};

// =============================================================================
// RADIO CALLBACKS - validate/copy only; no I2C, serial output or waiting here.
// =============================================================================

// Accept the reference radio's W,<sequence> packet as discovery, not a command.
// Require a complete decimal int32 sequence (including W,-1); ignore other data.
void onRadioReceive(const esp_now_recv_info_t *receiveInfo, const uint8_t *data, int length)
{
    if (length < 3 || length > 13 || data[0] != 'W' || data[1] != ',')
        return;

    // Validate digits without assuming the radio payload contains a terminator.
    int digitIndex = data[2] == '-' ? 3 : 2;
    if (digitIndex == length)
        return;
    uint64_t sequenceMagnitude = 0;
    for (; digitIndex < length; ++digitIndex)
    {
        if (data[digitIndex] < '0' || data[digitIndex] > '9')
            return;
        sequenceMagnitude = sequenceMagnitude * 10 + data[digitIndex] - '0';
    }

    // The mailbox owns a copy of all six bytes after this callback returns.
    const uint64_t maximumMagnitude = data[2] == '-' ? 2147483648ULL : 2147483647ULL;
    if (sequenceMagnitude <= maximumMagnitude)
        xQueueOverwrite(masterDiscoveryQueue, receiveInfo->src_addr);
}

// Mark the radio free after completion. Delivery success is a MAC-layer ACK,
// not proof that the master USB queue or the Python logger saved the sample.
void onRadioSend(const esp_now_send_info_t *sendInfo, esp_now_send_status_t status)
{
    if (status != ESP_NOW_SEND_SUCCESS)
        telemetryDeliveryFailures.fetch_add(1, std::memory_order_relaxed);
    telemetrySendPending.store(false, std::memory_order_release);
}

// =============================================================================
// OTA - initialize once Wi-Fi connects, including after the initial timeout.
// =============================================================================

// Register OTA callbacks once. Acquisition pauses during an upload, and both
// wheels are released again before flashing or recovering from an OTA error.
void initializeOta()
{
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);

    // OTA runs from loop(), so these callbacks never race with I2C acquisition.
    ArduinoOTA.onStart([]()
    {
        otaInProgress = true;
        if (motorBusReady)
            motorDriver.releaseMotorOutputs();
        Serial.println("E7 OTA started; acquisition paused, outputs released.");
    });

    // A successful upload reboots into the new image; do not resume the old run.
    ArduinoOTA.onEnd([]()
    {
        Serial.println("E7 OTA complete; rebooting.");
    });

    // On failure, keep outputs released and invalidate the speed history so
    // the next speed does not span the upload pause. Counts remain cumulative.
    ArduinoOTA.onError([](ota_error_t error)
    {
        if (motorBusReady)
            motorDriver.releaseMotorOutputs();
        previousEncoderValid[0] = previousEncoderValid[1] = false;
        otaInProgress = false;
        Serial.printf("E7 OTA error=%u\n", static_cast<unsigned>(error));
    });

    ArduinoOTA.begin();
    otaInitialized = true;
    Serial.printf("E7 OTA ready: %s, IP=%s\n", OTA_HOSTNAME, WiFi.localIP().toString().c_str());
}

// =============================================================================
// SETUP - release/configure motors first, then connect the radio and enable OTA.
// =============================================================================

// Initialize hardware without waiting for USB or commanding wheel movement.
// MotorDriver reapplies the five baseline settings; PID stays stored/unverified.
void setup()
{
    Serial.begin(115200);
    delay(100);

    // E6 sensor setup: D6/D7, controller 1, 400 kHz, 1 ms Wire timeout.
    sensorBusReady = sensorBus.begin(D6, D7, 400000);
    sensorBus.setTimeOut(1);

    // Release outputs before the radio's connection wait. Zero PWM exits the
    // driver's speed PID; a zero-speed target would resist manual pushing.
    motorBusReady = motorBus.begin(D4, D5, 400000);
    motorBus.setTimeOut(1);
    if (motorBusReady)
        motorConfigured = motorDriver.initialize();
    Serial.printf("E7 car=%u sensor_bus=%u motor_configured=%u PID=stored_unverified (3/0.375/0.5)\n",
                  unsigned(CAR_ID), unsigned(sensorBusReady), unsigned(motorConfigured));

    // Copy the tested STA/static-IP setup, including disabling modem sleep.
    // The AP chooses the radio channel; the existing master is fixed to 6.
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    if (!WiFi.config(staticIpAddress, gatewayAddress, subnetMask))
        Serial.println("E7 static IP setup failed.");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // Bound the initial wait. A later connection enables OTA at the top of loop.
    const uint32_t connectionStartedAtMs = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - connectionStartedAtMs < wifiConnectTimeoutMs)
        delay(100);
    Serial.printf("E7 WiFi=%u channel=%d MAC=%s\n", unsigned(WiFi.status() == WL_CONNECTED),
                  WiFi.channel(), WiFi.macAddress().c_str());

    // Create callback storage before registering callbacks; setup failures are
    // visible locally and acquisition can still run with the motors released.
    masterDiscoveryQueue = xQueueCreate(1, sizeof(masterMacAddress));
    if (masterDiscoveryQueue != nullptr && esp_now_init() == ESP_OK)
    {
        radioReady = esp_now_register_recv_cb(onRadioReceive) == ESP_OK &&
                     esp_now_register_send_cb(onRadioSend) == ESP_OK;
    }
    Serial.printf("E7 radio=%u; send W,0 through master to discover it.\n", unsigned(radioReady));

    // Start the only measurement schedule after all blocking startup work.
    bootIdentifier = esp_random();
    nextControlAtUs = esp_timer_get_time();
}

// =============================================================================
// MAIN LOOP - one 100 Hz cycle: release, line read, encoder reads, telemetry.
// =============================================================================

// Service OTA while waiting for the next cycle. Acquisition and sending share
// one schedule; missed slots are counted instead of issuing catch-up bursts.
void loop()
{
    if (WiFi.status() == WL_CONNECTED && !otaInitialized)
        initializeOta();
    if (otaInitialized)
        ArduinoOTA.handle();
    if (otaInProgress)
        return;

    // Keep the 10 ms schedule anchored even when I2C or Wi-Fi takes too long.
    const uint64_t cycleStartedAtUs = esp_timer_get_time();
    if (cycleStartedAtUs < nextControlAtUs)
        return;
    const uint64_t missedSlots = (cycleStartedAtUs - nextControlAtUs) / controlPeriodUs;
    skippedControlSlots += static_cast<uint32_t>(missedSlots);
    nextControlAtUs += (missedSlots + 1) * controlPeriodUs;
    const uint32_t currentSampleSequence = sampleSequence++;

    // -------------------------------------------------------------------------
    // Keep outputs released, including after a driver reset or earlier I2C fault.
    // 255 means the bus never initialized; 0 means this transaction succeeded.
    // -------------------------------------------------------------------------
    uint8_t outputReleaseError = 255;
    if (motorBusReady)
    {
        motorDriver.releaseMotorOutputs();
        outputReleaseError = motorDriver.lastCommunicationError();
    }

    // -------------------------------------------------------------------------
    // Read the raw eight-channel byte with E6's exact repeated-START sequence.
    // -1 denotes missing data. Never substitute an earlier successful sample.
    // -------------------------------------------------------------------------
    int lineRawMask = -1;
    uint8_t lineReadError = 255;
    const uint32_t lineReadOffsetUs = esp_timer_get_time() - cycleStartedAtUs;
    if (sensorBusReady)
    {
        sensorBus.beginTransmission(0x12);
        sensorBus.write(0x30);
        lineReadError = sensorBus.endTransmission(false);

        // The sensor byte is active-low: bit 7 is X1, bit 0 is X8.
        if (lineReadError == 0)
        {
            const size_t receivedBytes = sensorBus.requestFrom(0x12, static_cast<size_t>(1), true);
            if (receivedBytes == 1 && sensorBus.available() == 1)
                lineRawMask = sensorBus.read();
            else
            {
                lineReadError = 0x80; // Same short-read convention as MotorDriver.
                while (sensorBus.available())
                    sensorBus.read(); // Discard incomplete feedback as in E6.
            }
        }
    }

    // -------------------------------------------------------------------------
    // Read cumulative M2/M4 counts. Derive mm/s from successive successful reads
    // using the existing calibration and each wheel's actual elapsed time.
    // -------------------------------------------------------------------------
    int32_t encoderCounts[2] = {};
    uint32_t encoderReadOffsetsUs[2] = {};
    uint8_t encoderReadErrors[2] = {255, 255};
    double wheelSpeedsMmPerSecond[2] = {NAN, NAN};
    for (size_t wheelIndex = 0; wheelIndex < 2; ++wheelIndex)
    {
        // These are sequential read-start times, not simultaneous hardware latches.
        const uint64_t encoderReadAtUs = esp_timer_get_time();
        encoderReadOffsetsUs[wheelIndex] = encoderReadAtUs - cycleStartedAtUs;
        if (!motorBusReady)
            continue;
        const bool encoderValid = motorDriver.readEncoderPosition(encoderWheels[wheelIndex], encoderCounts[wheelIndex]);
        encoderReadErrors[wheelIndex] = motorDriver.lastCommunicationError();
        if (!encoderValid)
            continue;

        // Unsigned subtraction handles the 32-bit encoder rollover; widen the
        // signed interpretation explicitly rather than overflowing signed C++.
        if (previousEncoderValid[wheelIndex] && encoderReadAtUs > previousEncoderReadAtUs[wheelIndex])
        {
            const uint32_t differenceBits = static_cast<uint32_t>(encoderCounts[wheelIndex]) -
                                            static_cast<uint32_t>(previousEncoderCounts[wheelIndex]);
            const int64_t differenceCounts = differenceBits <= INT32_MAX ? int64_t(differenceBits) :
                                             int64_t(differenceBits) - 0x100000000LL;
            const uint64_t elapsedUs = encoderReadAtUs - previousEncoderReadAtUs[wheelIndex];
            wheelSpeedsMmPerSecond[wheelIndex] = differenceCounts *
                double(MotorSettings::millimetersPerEncoderCount) * 1000000.0 / elapsedUs;
        }

        // Following a read failure, speed averages over the gap since the last
        // valid count. The first successful read has count data but speed=nan.
        previousEncoderCounts[wheelIndex] = encoderCounts[wheelIndex];
        previousEncoderReadAtUs[wheelIndex] = encoderReadAtUs;
        previousEncoderValid[wheelIndex] = true;
    }
    const uint32_t acquisitionDurationUs = esp_timer_get_time() - cycleStartedAtUs;

    // -------------------------------------------------------------------------
    // Learn the first master from W,<sequence>. No heartbeat is required after
    // discovery, and W,-1 does not reset the encoder or telemetry sequence.
    // -------------------------------------------------------------------------
    if (!radioReady)
        return;
    if (!masterKnown && xQueueReceive(masterDiscoveryQueue, masterMacAddress, 0) == pdTRUE)
    {
        esp_now_peer_info_t masterPeer = {};
        memcpy(masterPeer.peer_addr, masterMacAddress, sizeof(masterMacAddress));
        masterPeer.channel = 0; // Follow the STA/AP channel, as in slave_radio.
        masterPeer.encrypt = false;
        masterKnown = esp_now_is_peer_exist(masterMacAddress) || esp_now_add_peer(&masterPeer) == ESP_OK;
    }
    if (!masterKnown)
        return;

    // Never queue multiple transmissions or delay acquisition waiting for ACKs.
    // Sequence gaps and this counter reveal samples dropped while radio is busy.
    if (telemetrySendPending.load(std::memory_order_acquire))
    {
        ++telemetryBusySkips;
        return;
    }

    // -------------------------------------------------------------------------
    // Text protocol E7,version,car,... fits the existing master's newline bridge.
    // See README.md for the exact columns, timing definitions and invalid values.
    // -------------------------------------------------------------------------
    const int messageLength = snprintf(telemetryMessage, sizeof(telemetryMessage),
        "E7,1,%u,%08lX,%lu,%llu,%lu,%d,%u,%ld,%lu,%.3f,%u,%ld,%lu,%.3f,%u,%u,%u,%lu,%lu,%lu,%lu,%lu",
        unsigned(CAR_ID), (unsigned long)bootIdentifier, (unsigned long)currentSampleSequence,
        (unsigned long long)cycleStartedAtUs, (unsigned long)lineReadOffsetUs, lineRawMask, unsigned(lineReadError),
        (long)encoderCounts[0], (unsigned long)encoderReadOffsetsUs[0], wheelSpeedsMmPerSecond[0], unsigned(encoderReadErrors[0]),
        (long)encoderCounts[1], (unsigned long)encoderReadOffsetsUs[1], wheelSpeedsMmPerSecond[1], unsigned(encoderReadErrors[1]),
        unsigned(motorConfigured), unsigned(outputReleaseError), (unsigned long)acquisitionDurationUs,
        (unsigned long)skippedControlSlots, (unsigned long)telemetryBusySkips, (unsigned long)telemetrySubmissionErrors,
        (unsigned long)telemetryDeliveryFailures.load(std::memory_order_relaxed));

    // Reject oversize packets rather than forwarding a truncated CSV record.
    if (messageLength <= 0 || messageLength > int(maximumMessageLength))
    {
        ++telemetrySubmissionErrors;
        return;
    }

    // Set pending before submission because completion may arrive immediately.
    // No callback is expected for a rejected send, so clear pending on that path.
    telemetrySendPending.store(true, std::memory_order_release);
    if (esp_now_send(masterMacAddress, reinterpret_cast<const uint8_t *>(telemetryMessage), messageLength) != ESP_OK)
    {
        telemetrySendPending.store(false, std::memory_order_release);
        ++telemetrySubmissionErrors;
    }
}
