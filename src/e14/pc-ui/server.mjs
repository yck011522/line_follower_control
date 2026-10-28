import { createServer } from "node:http";
import { readFile } from "node:fs/promises";
import { join } from "node:path";
import { fileURLToPath } from "node:url";
import { SerialPort } from "serialport";
import { ReadlineParser } from "@serialport/parser-readline";
import { WebSocketServer } from "ws";

const root = fileURLToPath(new URL("./public/", import.meta.url));
const portName = process.env.MODEM_PORT ?? "COM15";
const httpPort = Number(process.env.HTTP_PORT ?? 8080);
const telemetry = new Map();
const movement = [1, 1, 1, 1];
let topology = 0;
let worldSequence = 0;
let serialReady = false;
let serialError = null;
let serialWritePending = false;
const clients = new Set();

function broadcast(message) {
  const encoded = JSON.stringify(message);
  for (const client of clients) {
    if (client.readyState === 1) client.send(encoded);
  }
}

function decodeRobotState(line) {
  if (!/^[0-9a-fA-F]{42}$/.test(line)) return null;
  const bytes = Buffer.from(line, "hex");
  const robotId = bytes.readUInt8(0);
  if (robotId < 1 || robotId > 4) return null;
  return {
    robotId,
    telemetrySequence: bytes.readUInt16LE(1),
    worldSequence: bytes.readUInt16LE(3),
    worldPacketsLost5s: bytes.readUInt8(5),
    uid: [...bytes.subarray(6, 10)].map((byte) => byte.toString(16).padStart(2, "0")).join(""),
    uidAgeMs: bytes.readUInt16LE(10),
    lineSensorMask: bytes.readUInt8(12),
    controllerState: bytes.readUInt8(13),
    lineFollowerState: bytes.readUInt8(14),
    leftSpeedMmS: bytes.readInt16LE(15),
    rightSpeedMmS: bytes.readInt16LE(17),
    batteryMv: bytes.readUInt16LE(19)
  };
}

function packWorldState(sequence, mapId, movements) {
  const bytes = Buffer.alloc(7);
  bytes.writeUInt16LE(sequence, 0);
  bytes.writeUInt8(mapId, 2);
  movements.forEach((value, index) => bytes.writeUInt8(value, index + 3));
  return `W${bytes.toString("hex")}\n`;
}

function sendWorldState() {
  if (!serialReady || serialWritePending || !modemPort.isOpen) return;
  const packet = packWorldState(worldSequence, topology, movement);
  worldSequence = (worldSequence + 1) & 0xffff;
  serialWritePending = true;
  modemPort.write(packet, (error) => {
    serialWritePending = false;
    if (error) {
      console.error("World State serial write failed:", error);
      broadcast({ type: "serial", ready: false, error: error.message });
      return;
    }
  });
}

function sendSnapshot(socket) {
  const now = Date.now();
  const cars = Array.from({ length: 4 }, (_, index) => {
    const robotId = index + 1;
    const current = telemetry.get(robotId);
    return current
      ? { ...current, ageMs: now - current.receivedAt, movement: movement[index] }
      : { robotId, ageMs: null, movement: movement[index] };
  });
  socket.send(JSON.stringify({ type: "snapshot", serialReady, serialError, topology, cars }));
}

const modemPort = new SerialPort({
  path: portName,
  baudRate: 115200,
  autoOpen: false,
  rtscts: false
});
const parser = modemPort.pipe(new ReadlineParser({ delimiter: "\n" }));

parser.on("data", (rawLine) => {
  const line = rawLine.trim();
  const packet = decodeRobotState(line);
  if (!packet) return;
  telemetry.set(packet.robotId, { ...packet, receivedAt: Date.now() });
  broadcast({ type: "telemetry", packet, receivedAt: Date.now() });
});

modemPort.on("error", (error) => {
  console.error("Modem serial error:", error);
  serialReady = false;
  serialError = error.message;
  broadcast({ type: "serial", ready: false, error: error.message });
});

modemPort.open((error) => {
  if (error) {
    console.error(`Unable to open modem port ${portName}:`, error);
    serialError = error.message;
    broadcast({ type: "serial", ready: false, error: error.message });
    return;
  }
  modemPort.set({ dtr: true, rts: false }, (lineError) => {
    if (lineError) {
      console.error("Unable to set modem serial control lines:", lineError);
      serialError = lineError.message;
      broadcast({ type: "serial", ready: false, error: lineError.message });
      return;
    }
    serialReady = true;
    serialError = null;
    broadcast({ type: "serial", ready: true, port: portName });
  });
});

const httpServer = createServer(async (request, response) => {
  const requestedPath = new URL(request.url, "http://localhost").pathname;
  const fileName = requestedPath === "/" ? "index.html" : requestedPath.slice(1);
  if (!["index.html", "app.js", "styles.css"].includes(fileName)) {
    response.writeHead(404).end("Not found");
    return;
  }
  try {
    const content = await readFile(join(root, fileName));
    const contentType = fileName.endsWith(".js") ? "text/javascript"
      : fileName.endsWith(".css") ? "text/css" : "text/html";
    response.writeHead(200, { "Content-Type": `${contentType}; charset=utf-8` });
    response.end(content);
  } catch (error) {
    console.error(`Unable to serve ${fileName}:`, error);
    response.writeHead(500).end("Unable to read dashboard asset");
  }
});

const webSocketServer = new WebSocketServer({ server: httpServer });
webSocketServer.on("connection", (socket) => {
  clients.add(socket);
  sendSnapshot(socket);
  socket.on("message", (data) => {
    let message;
    try {
      message = JSON.parse(data.toString());
    } catch (error) {
      console.warn("Ignoring malformed dashboard message:", error.message);
      return;
    }
    if (message.type === "movement" && Number.isInteger(message.car) &&
        message.car >= 1 && message.car <= 4 && Number.isInteger(message.value) &&
        message.value >= 0 && message.value <= 0x1f) {
      movement[message.car - 1] = message.value;
    } else if (message.type === "topology" && Number.isInteger(message.value) &&
               message.value >= 0 && message.value <= 0xff) {
      topology = message.value;
    }
  });
  socket.on("close", () => clients.delete(socket));
});

const snapshotTimer = setInterval(() => {
  const now = Date.now();
  const cars = Array.from({ length: 4 }, (_, index) => {
    const robotId = index + 1;
    const current = telemetry.get(robotId);
    return current
      ? { ...current, ageMs: now - current.receivedAt, movement: movement[index] }
      : { robotId, ageMs: null, movement: movement[index] };
  });
  broadcast({ type: "snapshot", serialReady, serialError, topology, cars });
}, 250);

httpServer.listen(httpPort, "127.0.0.1", () => {
  console.log(`E14 dashboard listening at http://127.0.0.1:${httpPort} (modem ${portName})`);
});

for (const signal of ["SIGINT", "SIGTERM"]) {
  process.on(signal, () => {
    clearInterval(worldTimer);
    clearInterval(snapshotTimer);
    modemPort.close((error) => {
      if (error) console.error("Error closing modem serial port:", error);
      httpServer.close(() => process.exit(error ? 1 : 0));
    });
  });
}

const worldTimer = setInterval(sendWorldState, 20);
