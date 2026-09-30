# E3 firmware — NFC reader

Reserved for RC522 SPI UID acquisition and present/absent-tag timing. Do not read tag payload contents. IRQ is unconnected; board aliases need confirmation. Reuse the eventual driver from `lib/`.

Host automation and results: [test/E3_nfc_reader](../../test/E3_nfc_reader/README.md).
