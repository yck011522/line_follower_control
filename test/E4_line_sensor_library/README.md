# E4 - Line sensor library experiment

Firmware: [src/e4_line_sensor_library](../../src/e4_line_sensor_library/README.md).
Compare against E3 at 1000 Hz on Arduino 3.3.12. Confirm physical request starts
are at least 1000 us apart, including retries when they occur. Compare success
rates and latency, and observe repeat runs. Last-reading age must keep increasing
on failures and refresh on successful communication.

Ten-second summaries include physical success/failure counts, retry count,
request latency, minimum start gap and latest reading age. Preserve captures in
`results/<UTC-run>/serial.log`. No host unit tests or Python runner are added.

## Initial hardware result

Built/uploaded to COM4. After the operator power-cycled the reader, 13
complete repeat runs returned 130000/130000 successful requests. Minimum
request-start spacing was 1000 us in both successful and failure/retry phases.
See [report and raw log](results/20261001T092116Z_repeat/README.md).
