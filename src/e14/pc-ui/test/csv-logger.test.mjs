import { test } from "node:test";
import assert from "node:assert/strict";
import { mkdtempSync, readFileSync, rmSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { fileURLToPath } from "node:url";
import { setTimeout as delay } from "node:timers/promises";
import { createCsvLogger, localTimestamp } from "../csv-logger.mjs";

test("records every telemetry packet and only changed movement fields", async () => {
  const directory = mkdtempSync(join(tmpdir(), "e14-csv-test-"));
  try {
    const time = new Date("2026-11-01T02:20:04.127Z");
    const errors = [];
    const logger = createCsvLogger(directory, (error) => errors.push(error), time);
    const packet = {
      robotId: 1, telemetrySequence: 845, worldSequence: 1042, worldPacketsLost5s: 0,
      uid: "12ab34cd", uidAgeMs: 18, lineSensorMask: 0x18, controllerState: 0,
      lineFollowerState: 5, leftSpeedMmS: 97, rightSpeedMmS: -102, batteryMv: 12140
    };
    logger.logTelemetry(packet, time);
    logger.logTelemetry({ ...packet, robotId: 2, telemetrySequence: 846 }, time);
    logger.logMovementChange(1, 1, 1, time);
    logger.logMovementChange(1, 1, 3, time);
    logger.logMovementChange(1, 3, 10, time);
    await logger.close();

    const telemetry = readFileSync(join(directory, `${logger.name}_telemetry.csv`), "utf8").trim().split("\n");
    const commands = readFileSync(join(directory, `${logger.name}_commands.csv`), "utf8").trim().split("\n");
    const timestamp = localTimestamp(time);
    assert.match(timestamp, /^\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d\.\d{3}[+-]\d\d:\d\d$/);
    assert.equal(telemetry[0], "time_local,robot_id,telemetry_sequence,echoed_world_sequence,world_packets_lost_5s,nfc_uid_hex,nfc_age_ms,line_sensor_mask_hex,controller_state,line_follower_state,left_speed_mm_s,right_speed_mm_s,battery_mv");
    assert.equal(telemetry[1], `${timestamp},1,845,1042,0,12ab34cd,18,18,0,5,97,-102,12140`);
    assert.equal(telemetry[2], `${timestamp},2,846,1042,0,12ab34cd,18,18,0,5,97,-102,12140`);
    assert.deepEqual(commands, [
      "time_local,event_type,car_id,movement_byte_hex",
      `${timestamp},movement_change,1,03`,
      `${timestamp},mode_change,1,0a`,
      `${timestamp},movement_change,1,0a`
    ]);
    assert.deepEqual(errors, []);
  } finally {
    rmSync(directory, { recursive: true, force: true });
  }
});

test("uses distinct paired files for runs started in the same second", async () => {
  const directory = mkdtempSync(join(tmpdir(), "e14-csv-test-"));
  try {
    const time = new Date("2026-11-01T02:20:04.127Z");
    const first = createCsvLogger(directory, assert.fail, time);
    const second = createCsvLogger(directory, assert.fail, time);
    assert.equal(second.name, `${first.name}_1`);
    await Promise.all([first.close(), second.close()]);
    for (const logger of [first, second]) {
      assert.match(readFileSync(join(directory, `${logger.name}_telemetry.csv`), "utf8"), /^time_local,/);
      assert.match(readFileSync(join(directory, `${logger.name}_commands.csv`), "utf8"), /^time_local,/);
    }
  } finally {
    rmSync(directory, { recursive: true, force: true });
  }
});

test("surfaces errors when the log directory cannot be created", () => {
  assert.throws(() => createCsvLogger(join(fileURLToPath(import.meta.url), "logs"), assert.fail));
});

test("retains six hours of one-car telemetry at 20 Hz", async () => {
  const directory = mkdtempSync(join(tmpdir(), "e14-csv-test-"));
  try {
    const errors = [];
    const logger = createCsvLogger(directory, (error) => errors.push(error));
    const time = new Date();
    const packet = {
      robotId: 1, telemetrySequence: 0, worldSequence: 0, worldPacketsLost5s: 0,
      uid: "00000000", uidAgeMs: 65535, lineSensorMask: 0x18, controllerState: 0,
      lineFollowerState: 0, leftSpeedMmS: 100, rightSpeedMmS: 100, batteryMv: 12000
    };
    const sampleCount = 6 * 60 * 60 * 20;
    for (let index = 0; index < sampleCount; index++) {
      packet.telemetrySequence = index & 0xffff;
      logger.logTelemetry(packet, time);
      if (index % 1000 === 999) await delay(1);
    }
    await logger.close();
    const data = readFileSync(join(directory, `${logger.name}_telemetry.csv`), "utf8");
    assert.equal(data.split("\n").length - 1, sampleCount + 1);
    assert.deepEqual(errors, []);
  } finally {
    rmSync(directory, { recursive: true, force: true });
  }
});
