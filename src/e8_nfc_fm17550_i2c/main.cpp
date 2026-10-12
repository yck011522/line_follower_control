#include <Arduino.h>
#include <Wire.h>
#include <esp_timer.h>

namespace
{
constexpr uint8_t kReaderAddress = 0x28;
constexpr uint8_t kCommandReg = 0x01;
constexpr uint8_t kComIEnReg = 0x02;
constexpr uint8_t kComIrqReg = 0x04;
constexpr uint8_t kDivIrqReg = 0x05;
constexpr uint8_t kErrorReg = 0x06;
constexpr uint8_t kFIFODataReg = 0x09;
constexpr uint8_t kFIFOLevelReg = 0x0A;
constexpr uint8_t kControlReg = 0x0C;
constexpr uint8_t kBitFramingReg = 0x0D;
constexpr uint8_t kModeReg = 0x11;
constexpr uint8_t kTxControlReg = 0x14;
constexpr uint8_t kTxAskReg = 0x15;
constexpr uint8_t kTModeReg = 0x2A;
constexpr uint8_t kTPrescalerReg = 0x2B;
constexpr uint8_t kTReloadRegH = 0x2C;
constexpr uint8_t kTReloadRegL = 0x2D;
constexpr uint8_t kCrcResultRegH = 0x21;
constexpr uint8_t kCrcResultRegL = 0x22;
constexpr uint8_t kVersionReg = 0x37;
constexpr uint8_t kCommandIdle = 0x00;
constexpr uint8_t kCommandTransceive = 0x0C;
constexpr uint8_t kCommandSoftReset = 0x0F;
constexpr uint8_t kCommandCalcCrc = 0x03;
constexpr uint32_t kCommandTimeoutUs = 50000;
constexpr uint32_t kStatsIntervalUs = 5000000;
constexpr uint8_t kMaxUidLength = 10;

enum class ExchangeResult
{
    success,
    noResponse,
    ioError,
    protocolError
};

enum class PollResult
{
    noCard,
    uidReady,
    uidError
};

struct Exchange
{
    ExchangeResult result = ExchangeResult::ioError;
    uint8_t length = 0;
    uint8_t lastBits = 0;
};

struct PollStatistics
{
    uint32_t polls = 0;
    uint32_t cards = 0;
    uint32_t uidReadErrors = 0;
    uint32_t busErrors = 0;
    uint32_t cleanupErrors = 0;
};

PollStatistics intervalStats;
uint64_t nextStatsAtUs = 0;
uint64_t antennaReadyAtUs = 0;
bool readerReady = false;
bool readerIoFault = false;

// Read one FM17550 register using its I2C register-address byte.
bool readRegister(uint8_t reg, uint8_t &value)
{
    Wire.beginTransmission(kReaderAddress);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0)
    {
        readerIoFault = true;
        return false;
    }

    const size_t received = Wire.requestFrom(kReaderAddress, static_cast<size_t>(1), true);
    if (received != 1 || Wire.available() != 1)
    {
        while (Wire.available())
            Wire.read();
        readerIoFault = true;
        return false;
    }
    value = static_cast<uint8_t>(Wire.read());
    return true;
}

// Write one FM17550 register and record I2C acknowledgement failures.
bool writeRegister(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(kReaderAddress);
    Wire.write(reg);
    Wire.write(value);
    if (Wire.endTransmission() == 0)
        return true;
    readerIoFault = true;
    return false;
}

// Flush the reader FIFO before loading a new ISO14443A frame.
bool flushFifo()
{
    return writeRegister(kFIFOLevelReg, 0x80);
}

// Exchange one frame and distinguish an expected no-tag timeout from faults.
Exchange exchangeFrame(const uint8_t *tx, uint8_t txLength, uint8_t txLastBits,
                       uint8_t *rx, uint8_t rxCapacity)
{
    Exchange exchange;
    if (!writeRegister(kComIEnReg, 0x77) ||
        !writeRegister(kComIrqReg, 0x7F) ||
        !writeRegister(kCommandReg, kCommandIdle) ||
        !flushFifo())
        return exchange;

    for (uint8_t i = 0; i < txLength; ++i)
    {
        if (!writeRegister(kFIFODataReg, tx[i]))
            return exchange;
    }

    if (!writeRegister(kBitFramingReg, txLastBits) ||
        !writeRegister(kCommandReg, kCommandTransceive) ||
        !writeRegister(kBitFramingReg, static_cast<uint8_t>(txLastBits | 0x80)))
        return exchange;

    const uint64_t startedAtUs = esp_timer_get_time();
    uint8_t irq = 0;
    while (esp_timer_get_time() - startedAtUs < kCommandTimeoutUs)
    {
        if (!readRegister(kComIrqReg, irq))
            return exchange;
        if (irq & 0x01)
        {
            if (!writeRegister(kCommandReg, kCommandIdle) ||
                !writeRegister(kBitFramingReg, txLastBits))
                return exchange;
            exchange.result = ExchangeResult::noResponse;
            return exchange;
        }
        if (irq & 0x30)
            break;
    }
    if (!(irq & 0x30))
    {
        if (!writeRegister(kCommandReg, kCommandIdle) ||
            !writeRegister(kBitFramingReg, txLastBits))
            return exchange;
        exchange.result = ExchangeResult::noResponse;
        return exchange;
    }

    uint8_t error = 0;
    uint8_t count = 0;
    if (!writeRegister(kCommandReg, kCommandIdle) ||
        !writeRegister(kBitFramingReg, txLastBits) ||
        !readRegister(kErrorReg, error) ||
        !readRegister(kFIFOLevelReg, count) ||
        !readRegister(kControlReg, exchange.lastBits))
        return exchange;

    exchange.lastBits &= 0x07;
    if ((error & 0x1B) != 0 || count > rxCapacity)
    {
        exchange.result = ExchangeResult::protocolError;
        return exchange;
    }
    for (uint8_t i = 0; i < count; ++i)
    {
        if (!readRegister(kFIFODataReg, rx[i]))
            return exchange;
    }
    exchange.length = count;
    exchange.result = ExchangeResult::success;
    return exchange;
}

// Ask the FM17550 CRC coprocessor for the two-byte CRC_A value of a frame.
bool calculateCrc(const uint8_t *data, uint8_t length, uint8_t &crcLow, uint8_t &crcHigh)
{
    if (!writeRegister(kDivIrqReg, 0x04) || !flushFifo())
        return false;
    for (uint8_t i = 0; i < length; ++i)
    {
        if (!writeRegister(kFIFODataReg, data[i]))
            return false;
    }
    if (!writeRegister(kCommandReg, kCommandCalcCrc))
        return false;

    const uint64_t startedAtUs = esp_timer_get_time();
    uint8_t irq = 0;
    while (esp_timer_get_time() - startedAtUs < kCommandTimeoutUs)
    {
        if (!readRegister(kDivIrqReg, irq))
            return false;
        if (irq & 0x04)
            return readRegister(kCrcResultRegL, crcLow) &&
                   readRegister(kCrcResultRegH, crcHigh);
    }
    return false;
}

// Transmit HLTA and stop after the final bit; a halted tag intentionally stays silent.
bool haltSelectedCard()
{
    uint8_t frame[4] = {0x50, 0x00, 0, 0};
    if (!calculateCrc(frame, 2, frame[2], frame[3]) ||
        !writeRegister(kComIEnReg, 0x40) ||
        !writeRegister(kComIrqReg, 0x7F) ||
        !writeRegister(kCommandReg, kCommandIdle) ||
        !flushFifo())
        return false;
    for (uint8_t byte : frame)
    {
        if (!writeRegister(kFIFODataReg, byte))
            return false;
    }
    if (!writeRegister(kBitFramingReg, 0) ||
        !writeRegister(kCommandReg, kCommandTransceive) ||
        !writeRegister(kBitFramingReg, 0x80))
        return false;

    const uint64_t startedAtUs = esp_timer_get_time();
    uint8_t irq = 0;
    bool transmitted = false;
    while (esp_timer_get_time() - startedAtUs < kCommandTimeoutUs)
    {
        if (!readRegister(kComIrqReg, irq))
            return false;
        if (irq & 0x40)
        {
            transmitted = true;
            break;
        }
        if (irq & 0x01)
            break;
    }
    const bool cleanupSucceeded = writeRegister(kCommandReg, kCommandIdle) &&
                                  writeRegister(kBitFramingReg, 0);
    return transmitted && cleanupSucceeded;
}

// Anticollide and select a cascade level, checking its BCC and returned SAK CRC.
bool selectCascadeLevel(uint8_t cascadeCode, uint8_t *response, uint8_t &sak)
{
    const uint8_t anticollision[] = {cascadeCode, 0x20};
    Exchange exchange = exchangeFrame(anticollision, sizeof(anticollision), 0, response, 5);
    if (exchange.result != ExchangeResult::success || exchange.length != 5 ||
        exchange.lastBits != 0)
        return false;

    uint8_t bcc = 0;
    for (uint8_t i = 0; i < 4; ++i)
        bcc ^= response[i];
    if (bcc != response[4])
        return false;

    uint8_t selectFrame[9] = {cascadeCode, 0x70,
                              response[0], response[1], response[2], response[3], response[4], 0, 0};
    if (!calculateCrc(selectFrame, 7, selectFrame[7], selectFrame[8]))
        return false;

    uint8_t selectResponse[3] = {};
    exchange = exchangeFrame(selectFrame, sizeof(selectFrame), 0, selectResponse, sizeof(selectResponse));
    if (exchange.result != ExchangeResult::success ||
        exchange.length != sizeof(selectResponse) || exchange.lastBits != 0)
        return false;

    uint8_t sakCrcLow = 0;
    uint8_t sakCrcHigh = 0;
    if (!calculateCrc(selectResponse, 1, sakCrcLow, sakCrcHigh) ||
        selectResponse[1] != sakCrcLow || selectResponse[2] != sakCrcHigh)
        return false;

    sak = selectResponse[0];
    return true;
}

// Wake a tag and select each cascade level to return its full 4/7/10-byte UID.
PollResult readUid(uint8_t *uid, uint8_t &uidLength, uint64_t &detectedAtUs)
{
    const uint8_t wakeUpCommand = 0x52;
    uint8_t atqa[2] = {};
    Exchange exchange = exchangeFrame(&wakeUpCommand, 1, 7, atqa, sizeof(atqa));
    if (exchange.result == ExchangeResult::noResponse)
        return PollResult::noCard;
    if (exchange.result != ExchangeResult::success ||
        exchange.length != sizeof(atqa) || exchange.lastBits != 0)
        return PollResult::uidError;

    detectedAtUs = esp_timer_get_time();
    uidLength = 0;
    const uint8_t cascadeCodes[] = {0x93, 0x95, 0x97};
    for (uint8_t level = 0; level < 3; ++level)
    {
        uint8_t cascadeFrame[5] = {};
        uint8_t sak = 0;
        if (!selectCascadeLevel(cascadeCodes[level], cascadeFrame, sak))
            return PollResult::uidError;

        const bool hasCascadeTag = cascadeFrame[0] == 0x88;
        const uint8_t bytesToAppend = hasCascadeTag ? 3 : 4;
        const uint8_t startIndex = hasCascadeTag ? 1 : 0;
        if (uidLength + bytesToAppend > kMaxUidLength)
            return PollResult::uidError;
        for (uint8_t i = 0; i < bytesToAppend; ++i)
            uid[uidLength++] = cascadeFrame[startIndex + i];

        const bool moreLevels = (sak & 0x04) != 0;
        if (moreLevels != hasCascadeTag)
            return PollResult::uidError;
        if (!moreLevels)
            return uidLength == 4 || uidLength == 7 || uidLength == 10
                       ? PollResult::uidReady
                       : PollResult::uidError;
    }
    return PollResult::uidError;
}

// Initialize the FM17550 using the D4/D5 I2C bus without a blocking setup delay.
bool initializeReader()
{
    if (!writeRegister(kCommandReg, kCommandSoftReset))
        return false;

    const uint64_t resetStartedAtUs = esp_timer_get_time();
    uint8_t command = kCommandSoftReset;
    while ((command & 0x10) && esp_timer_get_time() - resetStartedAtUs < 5000)
    {
        if (!readRegister(kCommandReg, command))
            return false;
    }
    if (command & 0x10)
        return false;

    if (!writeRegister(kTModeReg, 0x8D) ||
        !writeRegister(kTPrescalerReg, 0x3E) ||
        !writeRegister(kTReloadRegH, 0x00) ||
        !writeRegister(kTReloadRegL, 30) ||
        !writeRegister(kModeReg, 0x3D) ||
        !writeRegister(kTxAskReg, 0x40))
        return false;

    uint8_t txControl = 0;
    uint8_t version = 0;
    if (!readRegister(kTxControlReg, txControl) ||
        !writeRegister(kTxControlReg, static_cast<uint8_t>(txControl | 0x03)) ||
        !readRegister(kVersionReg, version))
        return false;

    Serial.printf("FM17550 at 0x%02X, VersionReg=0x%02X\n", kReaderAddress, version);
    antennaReadyAtUs = esp_timer_get_time() + 10000;
    return true;
}

// Print one UID and elapsed time measured from the successful ATQA response.
void printUid(const uint8_t *uid, uint8_t uidLength, uint64_t detectedAtUs)
{
    const uint64_t elapsedUs = esp_timer_get_time() - detectedAtUs;
    Serial.print("UID=");
    for (uint8_t i = 0; i < uidLength; ++i)
    {
        if (i != 0)
            Serial.print(' ');
        Serial.printf("%02X", uid[i]);
    }
    Serial.printf(" detection_to_uid=%llu us (%.3f ms)\n",
                  static_cast<unsigned long long>(elapsedUs), elapsedUs / 1000.0);
}

// Perform one poll, count an ATQA response as a detected card, and report a complete UID.
void pollOnce()
{
    ++intervalStats.polls;
    readerIoFault = false;

    uint8_t uid[kMaxUidLength] = {};
    uint8_t uidLength = 0;
    uint64_t detectedAtUs = 0;
    const PollResult result = readUid(uid, uidLength, detectedAtUs);
    if (result == PollResult::noCard)
        return;

    ++intervalStats.cards;
    if (result == PollResult::uidError)
    {
        if (readerIoFault)
            ++intervalStats.busErrors;
        else
            ++intervalStats.uidReadErrors;
        Serial.println("Card detected; UID read failed");
        readerIoFault = false;
        if (!haltSelectedCard())
        {
            if (readerIoFault)
                ++intervalStats.busErrors;
            else
                ++intervalStats.cleanupErrors;
        }
        return;
    }

    printUid(uid, uidLength, detectedAtUs);
    readerIoFault = false;
    if (!haltSelectedCard())
    {
        if (readerIoFault)
            ++intervalStats.busErrors;
        else
            ++intervalStats.cleanupErrors;
    }
}

// Print and reset the rolling five-second poll, card, and error counters.
void printIntervalStatistics(uint64_t nowUs)
{
    Serial.printf("Stats (last 5 s): polls=%lu cards=%lu UID errors=%lu I2C errors=%lu cleanup errors=%lu\n",
                  static_cast<unsigned long>(intervalStats.polls),
                  static_cast<unsigned long>(intervalStats.cards),
                  static_cast<unsigned long>(intervalStats.uidReadErrors),
                  static_cast<unsigned long>(intervalStats.busErrors),
                  static_cast<unsigned long>(intervalStats.cleanupErrors));
    intervalStats = PollStatistics{};
    nextStatsAtUs = nowUs + kStatsIntervalUs;
}
} // namespace

// Start serial reporting and configure the FM17550's I2C bus on D4/D5.
void setup()
{
    Serial.begin(115200);
    Wire.setTimeOut(5);
    if (!Wire.begin(D4, D5, 400000))
    {
        Serial.println("ERROR: failed to initialize I2C on SDA=D4, SCL=D5");
        nextStatsAtUs = esp_timer_get_time() + kStatsIntervalUs;
        return;
    }

    Serial.println("FM17550 ISO14443A UID poller: SDA=D4 SCL=D5 address=0x28");
    readerReady = initializeReader();
    if (!readerReady)
        Serial.println("ERROR: FM17550 initialization failed; check I2C address, wiring, and interface mode");
    else
        Serial.println("Polling continuously; no scheduled poll delay.");
    nextStatsAtUs = esp_timer_get_time() + kStatsIntervalUs;
}

// Poll continuously without delay and emit rolling statistics every five seconds.
void loop()
{
    const uint64_t nowUs = esp_timer_get_time();
    if (!readerReady)
    {
        if (nowUs >= nextStatsAtUs)
        {
            Serial.println("Stats (last 5 s): polls=0 cards=0 UID errors=0 I2C errors=0; reader unavailable");
            nextStatsAtUs = nowUs + kStatsIntervalUs;
        }
        return;
    }
    if (nowUs < antennaReadyAtUs)
        return;

    pollOnce();
    const uint64_t afterPollUs = esp_timer_get_time();
    if (afterPollUs >= nextStatsAtUs)
        printIntervalStatistics(afterPollUs);
}
