# E8 NFC I2C tag test

XIAO ESP32-S3 D4 / D5 are GPIO5 / GPIO6. The shared bus runs at 100 kHz,
with a 20 ms transaction timeout and USB serial at 115200 baud.
All I2C transactions target only the NFC reader's 7-bit address **0x28**.
The line sensor at 0x12 is not probed, read, or configured.

The tag is now read successfully. The final firmware matched the phone's UID,
ATQA, and SAK on two consecutive runs, each succeeding on its first attempt.
Both runs validated BCC and SAK CRC and verified RF-off cleanup.

The firmware waits for an explicit serial command:

- `S`: reset/wake the reader, verify Type A settings, and attempt a four-byte UID read.
- `R`: read-only register snapshot, useful immediately after a full power cycle.
- `X`: cancel an active test. No motor commands exist in this firmware.

A read uses seven-bit REQA, cascade-level-1 anticollision, BCC validation,
and SELECT with software CRC_A. It validates the SAK response CRC and prints
UID, ATQA, and SAK. The phone reference is UID `11:E5:99:23`, ATQA `0x0004`,
SAK `0x08`. This initial implementation supports one four-byte-UID tag,
reports collisions as errors, and does not authenticate or access tag memory.
A successful UID read therefore does not imply that protected MIFARE blocks
have been read. No tag memory writes are issued.

The RF test is limited to ten attempts and a five-second attempt-loop deadline,
with an independent 60 ms RF-response deadline. Initialization and I2C cleanup
have their own bounded waits. Every exit attempts Idle and RF-off writes;
`cleanup=RF_OFF` requires readback of disabled antenna bits. Bus failure can
prevent cleanup, in which case the firmware reports `I2C_ERROR`.

The supplied FM17550 datasheet sections 3.2 and 4.4 define the register map and
I2C framing. Register reads use a pointer write with STOP followed by a read,
as illustrated in Figure 4-9. The version discrepancy remains unresolved:
0x37 reads 0x88 while the supplied datasheet specifies 0xA1.
Tag activation reference: [NXP MIFARE Classic EV1 1K datasheet](https://www.nxp.com/docs/en/data-sheet/MF1S50YYX_V1.pdf).

## Build, upload, and capture

From the repository root, select the actual board port:

```powershell
$env:PYTHONIOENCODING='utf-8'
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e e8_nfc_fm17550_i2c -t upload --upload-port COM9
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" test/E8_nfc_communication/capture.py COM9 S
```

Use `R` instead of `S` for a read-only snapshot. The Python capture sets DTR on
and RTS off before opening the port, never pulses RTS, retains raw output and
incomplete-run status, and attempts cancellation if a run does not finish.
New logs include the source SHA256. Upload/reset tooling is a separate workflow.

## Hardware observations, 2026-10-12

After the full power cycle, CommandReg read 0x20 (power-down bit clear), and
all configuration writes retained their values. Reset and RF transmission then
worked. Default receiver settings still produced no complete tag response.
A bounded receiver-setting sweep first read the tag with RFCfg 0x78, Demod
0x0D, RxThreshold 0x55, RxSel 0x80, and REQA. These settings were adopted
together; the experiment does not establish which individual change was necessary.
The receiver gain is 48 dB; automatic stronger-I/Q selection is enabled while
the demodulator's reset timing bits are preserved. Antenna drive power is unchanged.

Final firmware captures:

- `test/E8_nfc_communication/run_20261012_200816_806603_COM9.log`
- `test/E8_nfc_communication/run_20261012_200832_972531_COM9.log`

Both received the same frames:

```text
REQA response:       04 00
Anticollision:      11 E5 99 23 4E
SELECT response:    08 B6 DD
TAG UID=11:E5:99:23 ATQA=0x0004 SAK=0x08 BCC=OK CRC=OK
E8 END: result=TAG_READ cleanup=RF_OFF
```

ATQA is assembled little-endian from the wire bytes 04 00. The four UID bytes
are received from the tag, then compared with the reference; they are not used
to fabricate the anticollision response. SELECT echoes that received UID.
The reader's version remains 0x88, so its exact chip identity/revision remains
unconfirmed despite successful ISO14443A identification.

### Earlier failures (retained for diagnosis)

The original communication test acknowledged 0x12 and 0x28 and read version
0x88 at 0x28. Raw historical log: `test/E8_nfc_communication/run_20261012_185731_COM9.log`.
Subsequent iterations access 0x28 exclusively. Builds/uploads succeeded with
verified flash hashes; these are firmware checks, not proof of NFC operation.

- SoftReset left CommandReg at 0x1F after a 100 ms wait; an explicit Idle/wake
  write left it at 0x10 after another 100 ms. Bit 4 is still set.
- TxMode/RxMode, TxSel, RxSel, Mode, and TxControl retained writes.
- TxAuto 0x15 and timer configuration 0x2A-0x2D did not retain requested writes.
- Repeated START and STOP-separated register reads gave the same startup failure.
- One diagnostic build attempted RF despite the configuration mismatch, using
  the independent host deadline. Ten WUPA attempts yielded IRQ 0x0C, no TxIRq
  or RxIRq, ErrorReg 0x00, and FIFO length 1: the outgoing byte remained queued.
  No ATQA, UID, or SAK was received. Raw log:
  `test/E8_nfc_communication/run_20261012_191829_COM9.log`.
- The current firmware requires successful startup/configuration before RF.
  Read-only snapshot `run_20261012_192049_486316_COM9.log` confirmed CommandReg
  0x10, TxControl 0x80 (antenna outputs disabled), and VersionReg 0x88.

The earlier register behavior suggested a reader startup/clock problem; the
full power cycle cleared it, but its physical cause is not established. The user reports a four-wire
connection, 3.3 V module supply, and no exposed reset pin; IRQ is unnecessary
for the polling implementation. If startup fails again, disconnect all four
module wires or remove all supplying power to avoid back-power through SDA/SCL,
then use `R` to inspect startup and `S` to retry with the tag in place.
All iteration logs are retained in `test/E8_nfc_communication/`.
