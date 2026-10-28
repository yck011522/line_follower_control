const carsElement = document.querySelector("#cars");
const serialStatus = document.querySelector("#serial-status");
const topologyInput = document.querySelector("#topology");
const controllerNames = { 0: "AUTO", 1: "MANUAL", 14: "LOW_BATT", 15: "STOPPED" };
const lineNames = { 0: "FOLLOW_LINE", 4: "BRANCHING_CENTER", 5: "BRANCHING_LEFT", 6: "BRANCHING_RIGHT", 10: "LINE_LOST" };
let socket;
let currentCars = Array.from({ length: 4 }, (_, index) => ({
  robotId: index + 1,
  ageMs: null,
  movement: 1
}));
const carCards = [];

function movementValue(car) {
  let value = car.mode === "auto" ? 1 : 0;
  if (car.forward) value |= 2;
  if (car.backward) value |= 4;
  if (car.left) value |= 8;
  if (car.right) value |= 16;
  return value;
}

function sendMovement(car) {
  if (socket?.readyState === WebSocket.OPEN)
    socket.send(JSON.stringify({ type: "movement", car: car.robotId, value: movementValue(car) }));
}

function createCarCard(car, index) {
  const card = document.createElement("section");
  card.className = "car-card";
  card.innerHTML = `
      <div class="card-heading"><h2>Car ${car.robotId}</h2><span class="presence"></span></div>
      <dl>
        <dt>Controller</dt><dd data-value="controller">—</dd>
        <dt>Line state</dt><dd data-value="line-state">—</dd>
        <dt>Wheel speed</dt><dd data-value="wheel-speed">—</dd>
        <dt>Battery</dt><dd data-value="battery">—</dd>
        <dt>NFC UID</dt><dd data-value="uid">—</dd>
        <dt>NFC age</dt><dd data-value="uid-age">—</dd>
        <dt>World echo</dt><dd data-value="world-echo">—</dd>
      </dl>
      <div class="controls">
        <label>Mode
          <select data-field="mode" aria-label="Car ${car.robotId} mode">
            <option value="auto">Auto</option>
            <option value="manual">Manual</option>
          </select>
        </label>
        <label><input type="checkbox" data-field="forward" aria-label="Car ${car.robotId} forward"> Forward</label>
        <label><input type="checkbox" data-field="backward" aria-label="Car ${car.robotId} backward"> Backward</label>
        <label><input type="checkbox" data-field="left" aria-label="Car ${car.robotId} left"> Left</label>
        <label><input type="checkbox" data-field="right" aria-label="Car ${car.robotId} right"> Right</label>
        <button type="button" data-stop>Stop</button>
      </div>`;
  const view = {
    element: card,
    presence: card.querySelector(".presence"),
    controller: card.querySelector('[data-value="controller"]'),
    lineState: card.querySelector('[data-value="line-state"]'),
    wheelSpeed: card.querySelector('[data-value="wheel-speed"]'),
    battery: card.querySelector('[data-value="battery"]'),
    uid: card.querySelector('[data-value="uid"]'),
    uidAge: card.querySelector('[data-value="uid-age"]'),
    worldEcho: card.querySelector('[data-value="world-echo"]'),
    controls: Object.fromEntries(
      [...card.querySelectorAll("[data-field]")].map((input) => [input.dataset.field, input])
    )
  };
  Object.values(view.controls).forEach((input) => {
    input.addEventListener("change", () => {
      const current = currentCars[index];
      const field = input.dataset.field;
      current[field] = input.type === "checkbox" ? input.checked : input.value;
      sendMovement(current);
    });
  });
  card.querySelector("[data-stop]").addEventListener("click", () => {
    const current = currentCars[index];
    current.mode = "auto";
    current.forward = false;
    current.backward = false;
    current.left = false;
    current.right = false;
    sendMovement(current);
    updateControls(current, view);
  });
  updateControls(car, view);
  return view;
}

function updateControls(car, view) {
  view.controls.mode.value = car.mode ?? ((car.movement & 1) ? "auto" : "manual");
  for (const field of ["forward", "backward", "left", "right"]) {
    const bit = { forward: 2, backward: 4, left: 8, right: 16 }[field];
    view.controls[field].checked = car[field] ?? Boolean(car.movement & bit);
  }
}

function updateCard(car, view) {
  const stale = car.ageMs !== null && car.ageMs > 1000;
  view.presence.textContent = car.ageMs === null
    ? "No data"
    : stale ? `Stale · ${Math.floor(car.ageMs)} ms` : `${Math.floor(car.ageMs)} ms ago`;
  view.presence.classList.toggle("stale", stale);
  view.controller.textContent = car.controllerState === undefined
    ? "—" : controllerNames[car.controllerState] ?? `Unknown (${car.controllerState})`;
  view.lineState.textContent = car.lineFollowerState === undefined
    ? "—" : lineNames[car.lineFollowerState] ?? `Unknown (${car.lineFollowerState})`;
  view.wheelSpeed.textContent = `${car.leftSpeedMmS ?? "—"} / ${car.rightSpeedMmS ?? "—"} mm/s`;
  view.battery.textContent = car.batteryMv === undefined ? "—" : `${(car.batteryMv / 1000).toFixed(2)} V`;
  view.uid.textContent = car.uid ?? "—";
  view.uidAge.textContent = car.uidAgeMs === undefined ? "—" : `${car.uidAgeMs} ms`;
  view.worldEcho.textContent = `${car.worldSequence ?? "—"} · gaps ${car.worldPacketsLost5s ?? "—"}`;
}

function render() {
  currentCars.forEach((car, index) => {
    if (!carCards[index]) {
      carCards[index] = createCarCard(car, index);
      carsElement.append(carCards[index].element);
    }
    updateCard(car, carCards[index]);
  });
}

function connect() {
  const scheme = location.protocol === "https:" ? "wss:" : "ws:";
  socket = new WebSocket(`${scheme}//${location.host}`);
  socket.addEventListener("open", () => {
    serialStatus.textContent = "Dashboard connected";
    serialStatus.className = "pill";
  });
  socket.addEventListener("message", ({ data }) => {
    const message = JSON.parse(data);
    if (message.type === "serial") {
      serialStatus.textContent = message.ready ? `Modem ${message.port} ready` : `Modem unavailable${message.error ? `: ${message.error}` : ""}`;
      serialStatus.className = message.ready ? "pill good" : "pill bad";
    } else if (message.type === "snapshot") {
      topologyInput.value = message.topology;
      if (message.serialReady) {
        serialStatus.textContent = "Modem serial ready";
        serialStatus.className = "pill good";
      } else if (message.serialError) {
        serialStatus.textContent = `Modem unavailable: ${message.serialError}`;
        serialStatus.className = "pill bad";
      }
      currentCars = message.cars.map((incoming, index) => ({
        ...currentCars[index],
        ...incoming,
        mode: currentCars[index].mode ?? ((incoming.movement & 1) ? "auto" : "manual"),
        forward: currentCars[index].forward ?? Boolean(incoming.movement & 2),
        backward: currentCars[index].backward ?? Boolean(incoming.movement & 4),
        left: currentCars[index].left ?? Boolean(incoming.movement & 8),
        right: currentCars[index].right ?? Boolean(incoming.movement & 16)
      }));
      render();
    } else if (message.type === "telemetry") {
      const index = message.packet.robotId - 1;
      currentCars[index] = {
        ...currentCars[index],
        ...message.packet,
        ageMs: Date.now() - message.receivedAt
      };
      render();
    }
  });
  socket.addEventListener("close", () => {
    serialStatus.textContent = "Dashboard disconnected";
    serialStatus.className = "pill bad";
    setTimeout(connect, 1000);
  });
}

topologyInput.addEventListener("change", () => {
  const value = Number(topologyInput.value);
  if (Number.isInteger(value) && value >= 0 && value <= 255)
    socket?.send(JSON.stringify({ type: "topology", value }));
});

render();
connect();
