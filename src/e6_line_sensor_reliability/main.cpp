// Include libraries.
#include <Arduino.h>

#include <Wire.h>
#include <esp_timer.h>
#include <esp_arduino_version.h>

// Setting for benchmark test parameters, modifiable from Serial
uint32_t test_DurationS = 10;      // Total duration of the benchmark test in milliseconds
const char command_Duration = 'D'; // Command to set the duration of the benchmark test
const char command_Start = 'S';    // Command to start the benchmark test
const char command_Stop = 'X';     // Command to stop the benchmark test

// Configurable settings for the test
// Sets the i2c_sensor_ReadingRateHz
char command_ReadingRate = 'R'; // Command to set the reading rate of the benchmark test in Hz
// Sets the i2c_sensor_ClockHz
char command_ClockHz = 'C'; // Command to set the clock speed of the benchmark
// Sets the i2c_sensor_TimeoutMs
char command_TimeoutMs = 'T'; // Command to set the timeout of the benchmark test

// Settings for dummy motor I2C bus 1.
// Typical: SDA = D4, SCL = D5, 400 MHz clock, 1 ms timeout, 100 Hz writing rate
uint8_t i2c_motor_SDA = D4;
uint8_t i2c_motor_SCL = D5;
uint8_t i2c_motor_ClockHz = 400000;
uint8_t i2c_motor_TimeoutMs = 1;
uint8_t i2c_motor_WritingRateHz = 100;

// Settings for Line Sensor I2C bus 2.
// Typical: SDA = D6, SCL = D7, 400 MHz clock, 1 ms timeout, 100 Hz reading rate
uint8_t i2c_sensor_SDA = D6;
uint8_t i2c_sensor_SCL = D7;
uint8_t i2c_sensor_ClockHz = 400000;
uint8_t i2c_sensor_TimeoutMs = 1;
uint8_t i2c_sensor_ReadingRateHz = 100;

///////////////////////////
// Classes and global variables

// Rename Wire bus for clarity as MotorBus
TwoWire &Motor_Bus = Wire;
TwoWire Sensor_Bus = TwoWire(1); // Manually instantiate Wire1 for Hardware Controller 1

bool test_Running = false; // Flag to indicate if the benchmark test is running

//////////////////////////////
// Struct for Measurement statistics

struct Statistics
{
    uint32_t attempts = 0, successes = 0, transmitErrors = 0, shortReads = 0;
    uint64_t totalUs = 0, successUs = 0, minimumUs = UINT64_MAX, maximumUs = 0;
    uint64_t elapsedUs = 0, skippedSlots = 0, maximumFailedUs = 0;
    uint8_t lastRaw = 0, lastTransmitError = 0;
    const char *reason = "duration reached";
};
Statistics test_stats; // Global variable to hold the statistics of the benchmark test

// Function to read sensor once
void readOnce(Statistics &stats)
{
    const uint64_t start = esp_timer_get_time();
    bool success = false;
    Sensor_Bus.beginTransmission(0x12);
    Sensor_Bus.write(0x30);
    const uint8_t tx_error = Sensor_Bus.endTransmission(false);
    if (tx_error)
    {
        ++stats.transmitErrors;
        stats.lastTransmitError = tx_error;
    }
    else
    {
        const size_t count = Sensor_Bus.requestFrom(0x12, static_cast<size_t>(1), true);
        if (count == 1 && Sensor_Bus.available() == 1)
        {
            stats.lastRaw = static_cast<uint8_t>(Sensor_Bus.read());
            success = true;
        }
        else
        {
            ++stats.shortReads;
            while (Sensor_Bus.available())
                Sensor_Bus.read(); // Discard partial feedback.
        }
    }
    const uint64_t duration = esp_timer_get_time() - start;
    ++stats.attempts;
    stats.totalUs += duration;
    if (duration < stats.minimumUs)
        stats.minimumUs = duration;
    if (duration > stats.maximumUs)
        stats.maximumUs = duration;
    if (success)
    {
        ++stats.successes;
        stats.successUs += duration;
    }
    else if (duration > stats.maximumFailedUs)
        stats.maximumFailedUs = duration; // Separate failure latency from normal successful reads.
}

// Function to print summary to Serial
// Call only outside the timed I2C request path, to avoid affecting the benchmark.
void printSummary(const Statistics &stats)
{
    Serial.printf("E3 END: %s\n", stats.reason);
    Serial.printf("Success: %lu / %lu (%.2f%%)\n", (unsigned long)stats.successes,
                  (unsigned long)stats.attempts, stats.attempts ? 100.0 * stats.successes / stats.attempts : 0.0);
    Serial.printf("Request time, all attempts: mean=%.2f us min=%llu us max=%llu us\n",
                  stats.attempts ? double(stats.totalUs) / stats.attempts : 0.0,
                  (unsigned long long)(stats.attempts ? stats.minimumUs : 0), (unsigned long long)stats.maximumUs);
    if (stats.successes)
        Serial.printf("Successful requests: mean=%.2f us; last raw=0x%02X\n",
                      double(stats.successUs) / stats.successes, stats.lastRaw);
    Serial.printf("Transmit errors=%lu short reads=%lu last transmit error=%u\n",
                  (unsigned long)stats.transmitErrors, (unsigned long)stats.shortReads, stats.lastTransmitError);
    Serial.printf("Failed requests: count=%lu max=%llu us\n",
                  (unsigned long)(stats.attempts - stats.successes), (unsigned long long)stats.maximumFailedUs);
    Serial.printf("Elapsed=%.3f ms achieved=%.2f requests/s skipped slots=%llu\n\n",
                  stats.elapsedUs / 1000.0, stats.elapsedUs ? 1e6 * stats.attempts / stats.elapsedUs : 0.0,
                  (unsigned long long)stats.skippedSlots);
}

// Initialize the second I2C bus directly, without constructing LineSensor.
void setup()
{
    Serial.begin(115200);
    // Initialize the I2C buses for the sensor and motor
    if (!Sensor_Bus.begin(i2c_sensor_SDA, i2c_sensor_SCL, i2c_sensor_ClockHz))
    {
        Serial.println("Failed to initialize Sensor_Bus");
    }
    else
    {
        Serial.println("Sensor_Bus initialized successfully");
    }
    Sensor_Bus.setTimeout(i2c_sensor_TimeoutMs);

    if (!Motor_Bus.begin(i2c_motor_SDA, i2c_motor_SCL, i2c_motor_ClockHz))
    {
        Serial.println("Failed to initialize Motor_Bus");
    }
    else
    {
        Serial.println("Motor_Bus initialized successfully");
    }
    Motor_Bus.setTimeout(i2c_motor_TimeoutMs);
}

void loop()
{
    // Read Seral Command to set test parameters
    // Parameters can only be set when test are not running, other prints back error
    // Tests can be started and stopped with commands, and the duration can be set before starting the test

    if (Serial.available())
    {
        char command = Serial.read();
        switch (command)
        {
        case command_Duration:
            if (test_Running)
            {
                Serial.println("Cannot set duration while test is running");
            }
            else
            {
                // Read the duration value from Serial
                while (!Serial.available())
                    ; // Wait for input
                test_DurationS = Serial.parseInt();
                Serial.printf("Test duration set to %lu seconds\n", (unsigned long)test_DurationS);
            }
            break;
        case command_Start:
            if (test_Running)
            {
                Serial.println("Test is already running");
            }
            else
            {
                test_Running = true;
                Serial.println("Starting test...");
                // Start the benchmark test here
                // ...
            }
            break;
        case command_Stop:
            if (!test_Running)
            {
                Serial.println("Test is not running");
            }
            else
            {
                test_Running = false;
                Serial.println("Stopping test...");
                // Stop the benchmark test here
                // Print the summary of the test results
                // ...
            }
            break;
        default:
            Serial.println("Unknown command");
            break;
        }
    }

    // Stop the test if the duration has been reached
    if (test_Running)
    {
        static unsigned long startTime = millis();
        if (millis() - startTime >= test_DurationS * 1000)
        {
            test_Running = false;
            Serial.println("Test duration reached, stopping test...");
            // Stop the benchmark test here
            // Print the summary of the test results
            // ...
        }
    }

    // Code to grab one sensor reading when polling time is due
    if (test_Running)
    {
        static uint64_t lastReadTimeUs = 0;
        uint64_t currentTimeUs = esp_timer_get_time();
        if (currentTimeUs - lastReadTimeUs >= 1000000 / i2c_sensor_ReadingRateHz)
        {
            lastReadTimeUs = currentTimeUs;
            // Read the sensor here
            readOnce(test_stats);
        }
    }
}