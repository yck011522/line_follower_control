import { createServer } from "node:http";
import { readFile } from "node:fs/promises";
import { appendFileSync } from "node:fs";
import { join } from "node:path";
import { fileURLToPath } from "node:url";
import { SerialPort } from "serialport";
import { ReadlineParser } from "@serialport/parser-readline";
import { WebSocketServer } from "ws";
import { createCsvLogger } from "./csv-logger.mjs";

const root = fileURLToPath(new URL("./public/", import.meta.url));
const portName = process.env.MODEM_PORT ?? "COM15";
const httpPort = Number(process.env.HTTP_PORT ?? 8080);
const telemetry = new Map();
const movement = [1, 1, 1, 1];
const schedules = [null, null, null, null];
const scheduleStatus = ["Disabled", "Disabled", "Disabled", "Disabled"];
const maximumScheduleMs = 8 * 60 * 60 * 1000;
const telemetryTimeoutMs = 1000;
const logDirectory = fileURLToPath(new URL("../pc-ui-logs/", import.meta.url));
let topology = 0;
let worldSequence = 0;
let serialReady = false;
let serialError = null;
let serialWritePending = false;
const clients = new Set();
let loggingStatus = { name: null, error: null };
let logger;

function broadcast(message) {
  const encoded = JSON.stringify(message);
  for (const client of clients) {
    if (client.readyState === 1) client.send(encoded);
  }
}

function reportLoggingError(error) {
  console.error("CSV recording failed:", error);
  loggingStatus = { ...loggingStatus, error: error.message };
  broadcast({ type: "logging", ...loggingStatus });
  cancelAllSchedules("recording_failure");
}

try {
  logger = createCsvLogger(logDirectory, reportLoggingError);
  loggingStatus.name = logger.name;
  console.log(`Recording CSV files in src/e15/pc-ui-logs: ${logger.name}`);
} catch (error) {
  reportLoggingError(error);
}

function changeMovement(index, value) {
  const previous = movement[index];
  movement[index] = value;
  logger?.logMovementChange(index + 1, previous, value, new Date());
}

function recordSchedule(event) {
  if (!logger || loggingStatus.error) throw new Error("Recording is unavailable");
  appendFileSync(join(logDirectory, `${logger.name}_schedules.jsonl`),
    JSON.stringify({ time: new Date().toISOString(), ...event }) + "\n");
}

function cancelSchedule(index, reason) {
  const schedule = schedules[index];
  if (!schedule) return;
  schedules[index] = null;
  scheduleStatus[index] = `Ended: ${reason}`;
  changeMovement(index, 1);
  try {
    recordSchedule({ event: "end", car: index + 1, id: schedule.id, reason,
      completed: reason === "completed" });
  } catch (error) {
    console.error("Schedule metadata recording failed:", error);
    loggingStatus = { ...loggingStatus, error: error.message };
    broadcast({ type: "logging", ...loggingStatus });
  }
}

function cancelAllSchedules(reason) {
  for (let index = 0; index < schedules.length; index++) cancelSchedule(index, reason);
}

function communicationFailure(error) {
  serialReady = false;
  serialError = error;
  cancelAllSchedules("communication_failure");
  for (let index = 0; index < movement.length; index++) changeMovement(index, 1);
  broadcast({ type: "serial", ready: false, error });
}

function startSchedule(index, forwardMs, stopMs) {
  const current = telemetry.get(index + 1);
  if (!serialReady || !modemPort.isOpen || !current ||
      Date.now() - current.receivedAt > telemetryTimeoutMs ||
      Date.now() - current.echoChangedAt > telemetryTimeoutMs || !logger || loggingStatus.error)
    throw new Error("A connected modem, fresh car telemetry, and working recording are required");
  if (!Number.isSafeInteger(forwardMs) || !Number.isSafeInteger(stopMs) ||
      forwardMs < 1000 || stopMs < 1000 ||
      forwardMs > maximumScheduleMs || stopMs > maximumScheduleMs)
    throw new Error("Forward and stop durations must each be between 1 second and 8 hours");
  if (schedules[index]) throw new Error("Schedule already enabled");
  const now = performance.now();
  const schedule = { id: `${index + 1}-${Date.now()}`, forwardMs, stopMs,
    intendedMs: maximumScheduleMs, startedAt: now, phaseStartedAt: now, phase: "forward" };
  recordSchedule({ event: "start", car: index + 1, id: schedule.id,
    forwardMs, stopMs, intendedMs: maximumScheduleMs });
  schedules[index] = schedule;
  scheduleStatus[index] = "Running";
  changeMovement(index, 3);
}

function serviceSchedules() {
  const now = performance.now();
  for (let index = 0; index < schedules.length; index++) {
    const schedule = schedules[index];
    if (!schedule) continue;
    const current = telemetry.get(index + 1);
    if (!serialReady || !modemPort.isOpen || !current ||
        Date.now() - current.receivedAt > telemetryTimeoutMs ||
        Date.now() - current.echoChangedAt > telemetryTimeoutMs || loggingStatus.error) {
      cancelSchedule(index, "communication_or_recording_failure");
      continue;
    }
    if (now - schedule.startedAt >= schedule.intendedMs) {
      cancelSchedule(index, "completed");
      continue;
    }
    const phaseDuration = schedule.phase === "forward" ? schedule.forwardMs : schedule.stopMs;
    if (now - schedule.phaseStartedAt >= phaseDuration) {
      if (now - schedule.phaseStartedAt > phaseDuration + 2000) {
        cancelSchedule(index, "scheduler_delayed");
        continue;
      }
      schedule.phase = schedule.phase === "forward" ? "stop" : "forward";
      schedule.phaseStartedAt = now;
      changeMovement(index, schedule.phase === "forward" ? 3 : 1);
    }
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
      communicationFailure(error.message);
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
      ? { ...current, ageMs: now - current.receivedAt, movement: movement[index], schedule: schedules[index], scheduleStatus: scheduleStatus[index] }
      : { robotId, ageMs: null, movement: movement[index], schedule: schedules[index], scheduleStatus: scheduleStatus[index] };
  });
  socket.send(JSON.stringify({ type: "snapshot", serialReady, serialError, logging: loggingStatus, topology, cars }));
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
  const receivedAt = new Date();
  logger?.logTelemetry(packet, receivedAt);
  const previous = telemetry.get(packet.robotId);
  const echoChangedAt = !previous || previous.worldSequence !== packet.worldSequence
    ? receivedAt.getTime() : previous.echoChangedAt;
  telemetry.set(packet.robotId, { ...packet, receivedAt: receivedAt.getTime(), echoChangedAt });
  broadcast({ type: "telemetry", packet, receivedAt: receivedAt.getTime() });
});

modemPort.on("error", (error) => {
  console.error("Modem serial error:", error);
  communicationFailure(error.message);
});
modemPort.on("close", () => communicationFailure("Modem port closed"));

modemPort.open((error) => {
  if (error) {
    console.error(`Unable to open modem port ${portName}:`, error);
    communicationFailure(error.message);
    return;
  }
  modemPort.set({ dtr: true, rts: false }, (lineError) => {
    if (lineError) {
      console.error("Unable to set modem serial control lines:", lineError);
      communicationFailure(lineError.message);
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
      cancelSchedule(message.car - 1, "manual_command");
      changeMovement(message.car - 1, message.value);
    } else if (message.type === "schedule" && Number.isInteger(message.car) &&
               message.car >= 1 && message.car <= 4 && typeof message.enabled === "boolean") {
      try {
        if (message.enabled) startSchedule(message.car - 1, message.forwardMs, message.stopMs);
        else cancelSchedule(message.car - 1, "cancelled");
      } catch (error) {
        socket.send(JSON.stringify({ type: "schedule_error", car: message.car, error: error.message }));
      }
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
      ? { ...current, ageMs: now - current.receivedAt, movement: movement[index], schedule: schedules[index], scheduleStatus: scheduleStatus[index] }
      : { robotId, ageMs: null, movement: movement[index], schedule: schedules[index], scheduleStatus: scheduleStatus[index] };
  });
  broadcast({ type: "snapshot", serialReady, serialError, logging: loggingStatus, topology, cars });
}, 250);

httpServer.listen(httpPort, "127.0.0.1", () => {
  console.log(`E15 dashboard listening at http://127.0.0.1:${httpPort} (modem ${portName})`);
});

const worldTimer = setInterval(sendWorldState, 20);
const scheduleTimer = setInterval(serviceSchedules, 100);

let shuttingDown = false;
async function shutdown() {
  if (shuttingDown) return;
  shuttingDown = true;
  cancelAllSchedules("server_shutdown");
  for (let index = 0; index < movement.length; index++) changeMovement(index, 1);
  clearInterval(worldTimer);
  clearInterval(scheduleTimer);
  clearInterval(snapshotTimer);
  if (serialReady && modemPort.isOpen) {
    const deadline = Date.now() + 500;
    while (serialWritePending && Date.now() < deadline)
      await new Promise((resolve) => setTimeout(resolve, 10));
    if (!serialWritePending && serialReady && modemPort.isOpen) {
      try {
        await Promise.race([
          new Promise((resolve, reject) => modemPort.write(packWorldState(worldSequence, topology, movement),
            (error) => error ? reject(error) : modemPort.drain((drainError) => drainError ? reject(drainError) : resolve()))),
          new Promise((_, reject) => setTimeout(() => reject(new Error("stop write timed out")), 500))
        ]);
      } catch (error) {
        console.error("Unable to send shutdown stop:", error);
        process.exitCode = 1;
      }
    } else {
      console.error("Unable to send shutdown stop: serial write still pending or modem unavailable");
      process.exitCode = 1;
    }
  }
  for (const client of clients) client.terminate();
  webSocketServer.close();
  const serverClosed = new Promise((resolve) => httpServer.close(resolve));
  const serialClosed = new Promise((resolve) => {
    if (!modemPort.isOpen) return resolve();
    modemPort.close((error) => {
      if (error) {
        console.error("Error closing modem serial port:", error);
        process.exitCode = 1;
      }
      resolve();
    });
  });
  try {
    await logger?.close();
  } catch (error) {
    process.exitCode = 1;
  }
  await Promise.all([serverClosed, serialClosed]);
}

for (const signal of ["SIGINT", "SIGTERM"]) {
  process.on(signal, () => void shutdown());
}
