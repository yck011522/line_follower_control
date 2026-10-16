#pragma once
#include <Arduino.h>
#include <Wire.h>

// Owns only NFC address 0x28. Call service frequently; each call issues at most
// one synchronous Wire transaction and never spins waiting for an RF response.
class NfcAsync {
public:
    enum class Result : uint8_t { None, Uid, NoTag, ProtocolError, BusError, Cancelled };
    struct Options {
        uint32_t clockHz = 400000;
        uint16_t rfTimeoutUs = 1000;
        uint16_t pollUs = 710;
        bool adaptive = true;
        bool burst = true;
        bool cachePointer = true;
        bool fastTx = true;
        uint8_t gain = 6;
        bool combined = true;
        bool logInitialization = true; // Applications may own their own guarded serial output.
    };
    // Configure the chip while the application is idle; this one-time setup may block.
    bool begin(const Options &options);
    // Schedule a fresh over-the-air read; an existing UID is never returned as a new result.
    bool startRead();
    // Advance at most one bus transaction, returning immediately during RF wait intervals.
    void service(uint64_t nowUs);
    // Consume an event; uid points to four freshly received bytes only for Result::Uid.
    Result takeResult(uint8_t uid[4]);
    // Cancel pending work and attempt Idle/RF-off writes plus antenna readback.
    bool end();
    // Report whether a read is pending, so the caller can do other work between service calls.
    bool busy() const { return state_ != State::Idle; }
    // Count actual Wire transactions for benchmark overhead accounting.
    uint32_t transactions() const { return transactions_; }
    // Count rejected empty RF frames separately so a successful retry never hides them in reports.
    uint32_t emptyFrames() const { return emptyFrames_; }
    // Expose final frame diagnostics without extra bus accesses for retained experiment logs.
    void diagnostics(uint8_t values[6]) const {
        values[0] = discovery_; values[1] = irq_; values[2] = error_; values[3] = count_; values[4] = control_;
        values[5] = emptyRetries_;
    }
private:
    enum class State : uint8_t { Idle, Stop, Clear, Flush, Framing, Load, Command, Send,
        Poll, Error, Count, Control, Data, Abort };
    enum class ReadStep : uint8_t { Pending, Done, Failed };
    Options options_;
    State state_ = State::Idle;
    Result result_ = Result::None;
    bool initialized_ = false, readyTag_ = false, discovery_ = true;
    bool commandActive_ = false, fifoEmpty_ = false;
    bool retryAfterAbort_ = false;
    uint8_t emptyRetries_ = 0;
    uint8_t framing_ = 0xFF;
    uint8_t pointer_ = 0xFF, loadIndex_ = 0, dataIndex_ = 0;
    uint8_t irq_ = 0, error_ = 0, count_ = 0, control_ = 0, uid_[4] = {};
    uint8_t rx_[5] = {};
    uint64_t nextPollUs_ = 0, deadlineUs_ = 0;
    uint32_t transactions_ = 0;
    uint32_t emptyFrames_ = 0;
    // Send one register write and maintain the device's non-incrementing address latch.
    bool write(uint8_t reg, const uint8_t *values, size_t length);
    // Convenience wrapper for a single register value, still one Wire transaction.
    bool writeByte(uint8_t reg, uint8_t value);
    // Either split pointer/read across calls or combine them; optionally reuse a valid latch.
    ReadStep readStep(uint8_t reg, uint8_t *values, size_t length);
    // Read a static register during initialization, outside the asynchronous measurement window.
    bool readBlocking(uint8_t reg, uint8_t &value);
    // Verify a static setting before enabling RF output.
    bool configure(uint8_t reg, uint8_t value);
    // Start either discovery or anticollision without touching I2C in this call.
    void frame(bool discovery);
    // Record an event after aborting the active RF command on the next service call.
    void fail(Result result);
    // Validate complete frames, chaining discovery into anticollision when necessary.
    void finishFrame();
};
