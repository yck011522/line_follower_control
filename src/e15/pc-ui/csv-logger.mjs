import { closeSync, createWriteStream, mkdirSync, openSync, unlinkSync } from "node:fs";
import { join } from "node:path";
import { finished } from "node:stream/promises";

const telemetryHeader = "time_local,robot_id,telemetry_sequence,echoed_world_sequence,world_packets_lost_5s,nfc_uid_hex,nfc_age_ms,line_sensor_mask_hex,controller_state,line_follower_state,left_speed_mm_s,right_speed_mm_s,battery_mv\n";
const commandsHeader = "time_local,event_type,car_id,movement_byte_hex\n";
const maxBufferedBytes = 4 * 1024 * 1024;
const pad = (value, width = 2) => String(value).padStart(width, "0");
const hexByte = (value) => value.toString(16).padStart(2, "0");

export function localTimestamp(date) {
  const offsetMinutes = -date.getTimezoneOffset();
  const offset = `${offsetMinutes < 0 ? "-" : "+"}${pad(Math.floor(Math.abs(offsetMinutes) / 60))}:${pad(Math.abs(offsetMinutes) % 60)}`;
  return `${date.getFullYear()}-${pad(date.getMonth() + 1)}-${pad(date.getDate())}T${pad(date.getHours())}:${pad(date.getMinutes())}:${pad(date.getSeconds())}.${pad(date.getMilliseconds(), 3)}${offset}`;
}

function openLogFiles(directory, startTime) {
  mkdirSync(directory, { recursive: true });
  const prefix = localTimestamp(startTime).slice(0, 19).replace("T", "_").replaceAll(":", "-");
  for (let suffix = 0; ; suffix++) {
    const name = suffix === 0 ? prefix : `${prefix}_${suffix}`;
    const telemetryPath = join(directory, `${name}_telemetry.csv`);
    const commandsPath = join(directory, `${name}_commands.csv`);
    let telemetryFd;
    try {
      telemetryFd = openSync(telemetryPath, "wx");
    } catch (error) {
      if (error.code === "EEXIST") continue;
      throw error;
    }
    try {
      const commandsFd = openSync(commandsPath, "wx");
      return { name, telemetryPath, commandsPath, telemetryFd, commandsFd };
    } catch (error) {
      closeSync(telemetryFd);
      unlinkSync(telemetryPath);
      if (error.code !== "EEXIST") throw error;
    }
  }
}

export function createCsvLogger(directory, onError, startTime = new Date()) {
  const files = openLogFiles(directory, startTime);
  const telemetryStream = createWriteStream(files.telemetryPath, { fd: files.telemetryFd });
  const commandsStream = createWriteStream(files.commandsPath, { fd: files.commandsFd });
  const pendingRows = new Map([
    [telemetryStream, { rows: [], bytes: 0 }],
    [commandsStream, { rows: [], bytes: 0 }]
  ]);
  let flushTimer;
  let failure = null;
  let closed = false;

  function fail(error) {
    if (failure) return;
    failure = error;
    onError(error);
  }

  telemetryStream.on("error", fail);
  commandsStream.on("error", fail);

  function flush() {
    flushTimer = undefined;
    for (const [stream, pending] of pendingRows) {
      if (pending.rows.length === 0 || stream.destroyed) continue;
      stream.write(pending.rows.join(""));
      pending.rows = [];
      pending.bytes = 0;
    }
  }

  function write(stream, row) {
    if (failure || closed) return;
    const pending = pendingRows.get(stream);
    const bytes = Buffer.byteLength(row);
    if (stream.writableLength + pending.bytes + bytes > maxBufferedBytes) {
      fail(new Error("CSV logging buffer exceeded 4 MiB; recording is incomplete"));
      return;
    }
    pending.rows.push(row);
    pending.bytes += bytes;
    if (!flushTimer) flushTimer = setTimeout(flush, 100);
  }

  write(telemetryStream, telemetryHeader);
  write(commandsStream, commandsHeader);

  return {
    name: files.name,
    logTelemetry(packet, timestamp) {
      write(telemetryStream, [
        localTimestamp(timestamp), packet.robotId, packet.telemetrySequence, packet.worldSequence,
        packet.worldPacketsLost5s, packet.uid, packet.uidAgeMs, hexByte(packet.lineSensorMask),
        packet.controllerState, packet.lineFollowerState, packet.leftSpeedMmS,
        packet.rightSpeedMmS, packet.batteryMv
      ].join(",") + "\n");
    },
    logMovementChange(carId, previousMovement, newMovement, timestamp) {
      const changedBits = previousMovement ^ newMovement;
      const time = localTimestamp(timestamp);
      if (changedBits & 0x01)
        write(commandsStream, `${time},mode_change,${carId},${hexByte(newMovement)}\n`);
      if (changedBits & 0x1e)
        write(commandsStream, `${time},movement_change,${carId},${hexByte(newMovement)}\n`);
    },
    async close() {
      if (closed) return;
      closed = true;
      clearTimeout(flushTimer);
      flush();
      telemetryStream.end();
      commandsStream.end();
      const results = await Promise.allSettled([finished(telemetryStream), finished(commandsStream)]);
      for (const result of results) {
        if (result.status === "rejected") fail(result.reason);
      }
      if (failure) throw failure;
    }
  };
}
