# E3 firmware — NFC reader

Reserved for RC522 SPI UID acquisition and present/absent-tag timing. Do not read tag payload contents. IRQ is unconnected. Confirmed board aliases: CS D1, RST D0, SCK D8, MISO D9, MOSI D10; see [pin mappings](../../docs/hardware.md). Reuse the eventual driver from `lib/`.

Host automation and results: [test/E3_nfc_reader](../../test/E3_nfc_reader/README.md).
