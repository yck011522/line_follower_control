# LineSensor

One header, `LineSensor.h`, refactored from the standalone E3 direct read.
Construct inside `setup()`: `static LineSensor sensor(D6, D7, 1000, 5);`.
The arguments are SDA, SCL, normal poll frequency and maximum total attempts.

The constructor configures I2C controller 1, 1 MHz, a 1 ms requested Wire timeout,
address 0x12 and register 0x30. Use Arduino 3.3.12 as pinned by E3/E4. The chosen
pins must not be owned by another bus object. Use from one task.

`tick()` returns `Status::Idle`, `Success` or `Failure`. It makes at most one
physical request per call. Requests and retries start at least 1000 us apart.
Rate 0 means the 1000 Hz maximum; values above 1000 are capped. Other rates use
an integer period rounded up. Polls anchor to actual starts; there is no catch-up.
Five attempts means the first request plus up to four retries. Retries are
serviced by subsequent ticks, without sleeping in the library. Slow caller ticks
reduce the actual rate and delay retries. Each I2C transaction remains blocking.
After a batch succeeds or exhausts its attempts, the next normal poll resumes
when both its normal period and the 1 ms request spacing allow it.

`reading()` returns `valid`, the original active-low `rawMask`, and dynamically
calculated `ageUs`. Before success, valid=false and ageUs=UINT64_MAX. Failed
requests retain the previous byte and timestamp. Age means time since successful
communication, not the sensor's internal conversion age. Bit 7 is X1, bit 0 X8;
zero denotes black. Filtering and edge/center estimation are deferred.

`ready()` reports successful bus initialization. `lastRequest()` supplies
`startedAtUs`, `durationUs`, `transmitError` and the one-based attempt number for
benchmarking. Inspect it only when tick returns Success or Failure.

The class does not print serial messages or automatically reset/recover hardware.
The application's CORE_DEBUG_LEVEL=0 flag silences Arduino Wire diagnostics.
E4 uses the same settings and physical-request statistics as E3, with retry
counts, current reading age and minimum request-start gap added.
