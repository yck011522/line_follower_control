# E8 FM17550 I2C ISO14443A UID poller

Build with:

```powershell
pio run -e e8_nfc_fm17550_i2c
```

The firmware targets the XIAO ESP32S3, with the FM17550 wired to SDA D4 and
SCL D5. It assumes the reader module's default I2C address is `0x28`. If the
module's address straps are configured differently, update `kReaderAddress` in
`main.cpp`.

It initializes the reader once, then polls continuously from `loop()` without
a scheduled delay. Successful ISO14443A UID selections print the UID bytes
(including 4-, 7-, or 10-byte UIDs) and `detection_to_uid` in microseconds and
milliseconds. The timer begins when the reader receives the tag's ATQA reply;
it ends after all cascade levels have been selected and the complete UID is
available. This does not include serial transmission time.

Every five seconds, the firmware prints rolling counts for poll attempts,
detected cards (ATQA responses), UID read errors, I2C errors, and card-halt
cleanup errors. A tag kept in the field is reported on each successful poll.
Serial output can limit the achieved poll rate while a card is present.

Open the serial monitor at 115200 baud. No motor or other actuator commands
are issued by this firmware.
