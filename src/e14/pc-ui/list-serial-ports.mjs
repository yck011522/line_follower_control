import { SerialPort } from "serialport";

const ports = await SerialPort.list();
for (const port of ports) {
  console.log(`  ${port.path}${port.manufacturer ? ` - ${port.manufacturer}` : ""}`);
}
