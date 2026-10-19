#include <Arduino.h>
#include <Wire.h>
#include <esp_timer.h>
#include <stdarg.h>
#include "../e8_nfc_fm17550_i2c/NfcAsync.h"

namespace
{
    NfcAsync reader;
    bool running = false;
    uint8_t gainCode = 1; // Default receiver gain: code 1 selects 23 dB.
    // Indexed by RFCfgReg bits 6:4; codes 0/2 and 1/3 intentionally share gains.
    constexpr uint8_t gainDbByCode[] = {18, 23, 18, 23, 33, 38, 43, 48};
    bool receivingGain = false, gainOverflow = false;
    char gainArgument[12] = {};
    size_t gainLength = 0;
    uint64_t pollStartedUs = 0;
    uint64_t windowStartedUs = 0;
    constexpr uint64_t reportIntervalUs = 5000000;
    struct Statistics
    {
        uint32_t polls = 0, tags = 0, noTags = 0, errors = 0;
        uint64_t totalUs = 0, longestUs = 0;
    } stats;

    uint32_t serialDropped = 0, serialShortWrites = 0;
    uint64_t serialLongestUs = 0;

    // Drop output rather than wait for USB: one fixed-buffer format and one write.
    void serialOutput(const char *format, ...)
    {
        const uint64_t startedUs = esp_timer_get_time();
        if (!Serial) {
            ++serialDropped;
            return;
        }
        char line[256];
        va_list arguments;
        va_start(arguments, format);
        const int length = vsnprintf(line, sizeof(line), format, arguments);
        va_end(arguments);
        // Reserve room for the entire line; never wait or queue a retry backlog.
        if (length <= 0 || length >= static_cast<int>(sizeof(line)) || Serial.availableForWrite() < length) {
            ++serialDropped;
        } else if (Serial.write(reinterpret_cast<const uint8_t *>(line), static_cast<size_t>(length)) != static_cast<size_t>(length)) {
            // A disconnect between the space check and write may lose part of a line.
            ++serialShortWrites;
        }
        const uint64_t elapsedUs = esp_timer_get_time() - startedUs;
        if (elapsedUs > serialLongestUs) serialLongestUs = elapsedUs;
    }

    // Initialize outside the asynchronous poll path; only the NFC address is accessed.
    void startReader()
    {
        NfcAsync::Options options;
        options.gain = gainCode;
        options.logInitialization = false; // Do not bypass E9B's guarded writer.
        running = reader.begin(options);
        if (running)
            serialOutput("GAIN applied=%u dB\n", gainDbByCode[gainCode]);
        if (!running)
        {
            const bool rfOff = reader.end();
            serialOutput("E9B initialization failed; RF off=%s. Send S to retry.\n", rfOff ? "yes" : "unconfirmed");
        }
        stats = {};
        windowStartedUs = esp_timer_get_time();
    }

    // Stop pending RF work explicitly, including cancellation during a host capture.
    void stopReader()
    {
        running = false;
        const bool rfOff = reader.end();
        serialOutput("E9B stopped; RF off=%s\n", rfOff ? "yes" : "unconfirmed");
    }

    // Accept discrete datasheet gains in dB; restart RF safely outside timed reads.
    void finishGainCommand()
    {
        gainArgument[gainLength] = '\0';
        char *end = nullptr;
        const long requestedDb = strtol(gainArgument, &end, 10);
        while (end && (*end == ' ' || *end == '\t'))
            ++end;
        if (!gainOverflow && gainLength == 0)
        {
            serialOutput("GAIN configured=%u dB; allowed: 18,23,33,38,43,48; use G 23 + Enter\n", gainDbByCode[gainCode]);
            return;
        }
        uint8_t requestedCode = 8;
        if (!gainOverflow && end != gainArgument && *end == '\0')
        {
            for (uint8_t code = 0; code < 8; ++code)
            {
                if (gainDbByCode[code] == requestedDb)
                {
                    requestedCode = code;
                    break;
                }
            }
        }
        if (requestedCode == 8)
        {
            serialOutput("GAIN rejected; allowed dB: 18,23,33,38,43,48\n");
            return;
        }
        const bool resume = running;
        if (resume)
            stopReader(); // Cancel any in-flight read before reconfiguration.
        gainCode = requestedCode;
        if (resume)
            startReader(); // begin verifies the gain register by readback.
        else
            serialOutput("GAIN configured=%u dB; send S to apply\n", gainDbByCode[gainCode]);
    }

    // Accumulate G commands without waiting; X/S retain their immediate behavior.
    void readCommands()
    {
        // Bound command work too, so a host flooding queries cannot starve NFC.
        for (uint8_t consumed = 0; consumed < 32 && Serial.available(); ++consumed)
        {
            const char command = Serial.read();
            if (receivingGain)
            {
                if (command == '\r' || command == '\n')
                {
                    receivingGain = false;
                    finishGainCommand();
                }
                else if (gainLength < sizeof(gainArgument) - 1)
                {
                    gainArgument[gainLength++] = command;
                }
                else
                    gainOverflow = true;
            }
            else if (command == 'G' || command == 'g')
            {
                receivingGain = true;
                gainLength = 0;
                gainOverflow = false;
            }
            else if (command == 'X' || command == 'x')
                stopReader();
            else if ((command == 'S' || command == 's') && !running)
                startReader();
        }
    }

    // Report completed whole-read latency, including RF waits, every five seconds.
    void report(uint64_t nowUs)
    {
        const uint64_t elapsedUs = nowUs - windowStartedUs;
        if (elapsedUs < reportIntervalUs)
            return;
        const double rate = stats.polls * 1000000.0 / elapsedUs;
        const double averageMs = stats.polls ? stats.totalUs / (1000.0 * stats.polls) : 0.0;
        {
            serialOutput("REPORT polls/s=%.2f avg_poll_ms=%.3f max_poll_ms=%.3f polls=%lu tags=%lu no_tag=%lu errors=%lu gain_db=%u serial_dropped=%lu serial_short=%lu serial_max_us=%llu\n",
                          rate, averageMs, stats.longestUs / 1000.0,
                          static_cast<unsigned long>(stats.polls), static_cast<unsigned long>(stats.tags),
                          static_cast<unsigned long>(stats.noTags), static_cast<unsigned long>(stats.errors), gainDbByCode[gainCode],
                          static_cast<unsigned long>(serialDropped), static_cast<unsigned long>(serialShortWrites),
                          static_cast<unsigned long long>(serialLongestUs));
        }
        stats = {};
        windowStartedUs = nowUs;
    }
} // namespace

// Boot directly into continuous NFC monitoring on the board's D4/D5 pins.
void setup()
{
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.setTxTimeoutMs(0); // Also bounds lock/enqueue waits if USB stops draining.
    serialOutput("E9B NFC monitor: address 0x28, D4/D5; X=stop, S=start, G <dB> + Enter=gain\n");
    if (!Wire.begin(D4, D5, 400000))
    {
        serialOutput("E9B I2C initialization failed\n");
        windowStartedUs = esp_timer_get_time();
        return;
    }
    startReader();
}

// Advance one async service step and print only fresh successful UID results.
void loop()
{
    readCommands();
    if (running)
    {
        if (!reader.busy())
        {
            pollStartedUs = esp_timer_get_time();
            reader.startRead();
        }
        reader.service(esp_timer_get_time());
        uint8_t uid[4];
        const NfcAsync::Result result = reader.takeResult(uid);
        if (result != NfcAsync::Result::None)
        {
            // Timestamp before serial output: latency measures the read itself.
            const uint64_t durationUs = esp_timer_get_time() - pollStartedUs;
            ++stats.polls;
            stats.totalUs += durationUs;
            if (durationUs > stats.longestUs)
                stats.longestUs = durationUs;
            if (result == NfcAsync::Result::Uid)
            {
                ++stats.tags;
                digitalWrite(LED_BUILTIN, LOW);
                serialOutput("UID %02X:%02X:%02X:%02X\n", uid[0], uid[1], uid[2], uid[3]);
            }
            else if (result == NfcAsync::Result::NoTag)
            {
                digitalWrite(LED_BUILTIN, HIGH);
                ++stats.noTags;
            }
            else
            {
                ++stats.errors;
                digitalWrite(LED_BUILTIN, HIGH);
                if (result == NfcAsync::Result::BusError)
                    stopReader();
            }
        }
    }
    report(esp_timer_get_time());
}
