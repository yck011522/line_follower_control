#include "ble_telemetry.h"

#include <NimBLEDevice.h>
#include <atomic>
#include <cstring>

#include <ble14_protocol.h>

namespace {
portMUX_TYPE worldMux = portMUX_INITIALIZER_UNLOCKED;
Ble14::World latestWorld{};
bool haveWorld = false;
uint32_t lastWorldMs = 0;
uint32_t worldGeneration = 0;
uint32_t consumedWorldGeneration = 0;
uint32_t windowWorldLost = 0;
uint32_t lastLossWindowMs = 0;
uint8_t lastWorldLossCount = 0;
std::atomic<bool> connected{false};
std::atomic<bool> subscribed{false};
uint16_t telemetrySequence = 0;
NimBLEServer* bleServer = nullptr;
NimBLECharacteristic* telemetryCharacteristic = nullptr;

// Accepts valid-size World State writes and updates the shared latest snapshot.
class WorldCallbacks : public NimBLECharacteristicCallbacks {
    // Copies valid World State bytes and records sequence gaps inside a short critical section.
    void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo&) override {
        const auto value = characteristic->getValue();
        if (value.size() != sizeof(Ble14::World)) return;

        Ble14::World incoming{};
        memcpy(&incoming, value.data(), sizeof(incoming));
        const uint32_t now = millis();
        portENTER_CRITICAL(&worldMux);
        if (haveWorld) {
            const uint16_t delta = uint16_t(incoming.sequence-latestWorld.sequence);
            const uint32_t plausible = (now-lastWorldMs)/20+20;
            if (delta > 0 && delta < 0x8000 && delta <= plausible) {
                windowWorldLost += delta-1;
            }
        }
        latestWorld = incoming;
        haveWorld = true;
        lastWorldMs = now;
        ++worldGeneration;
        portEXIT_CRITICAL(&worldMux);
    }
};

// Tracks connection state and clears stale World State at each new modem session.
class ServerCallbacks : public NimBLEServerCallbacks {
    // Resets the received-World baseline for a new BLE session.
    void onConnect(NimBLEServer*, NimBLEConnInfo&) override {
        portENTER_CRITICAL(&worldMux);
        haveWorld = false;
        portEXIT_CRITICAL(&worldMux);
        connected = true;
        subscribed = false;
    }

    // Marks the session disconnected; the foreground service restarts advertising.
    void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
        connected = false;
        subscribed = false;
    }
};

// Tracks whether the modem enabled Robot State notifications.
class TelemetryCallbacks : public NimBLECharacteristicCallbacks {
    // Tracks whether notification delivery is enabled by the connected modem.
    void onSubscribe(NimBLECharacteristic*, NimBLEConnInfo&, uint16_t value) override {
        subscribed = (value&1) != 0;
    }
};

WorldCallbacks worldCallbacks;
ServerCallbacks serverCallbacks;
TelemetryCallbacks telemetryCallbacks;
}

// Creates the E13 GATT service and the Test 14-compatible identity advertisement.
bool BleTelemetry::begin(uint8_t carId) {
    if (carId < 1 || carId > Ble14::CARS) return false;
    NimBLEDevice::init(("e13_car_"+String(carId)).c_str());
    NimBLEDevice::setMTU(64);

    bleServer = NimBLEDevice::createServer();
    if (!bleServer) return false;
    bleServer->setCallbacks(&serverCallbacks);
    bleServer->advertiseOnDisconnect(false);

    auto* service = bleServer->createService(Ble14::SERVICE_UUID);
    if (!service) return false;
    auto* worldCharacteristic = service->createCharacteristic(
        Ble14::WORLD_UUID, NIMBLE_PROPERTY::WRITE_NR, sizeof(Ble14::World));
    telemetryCharacteristic = service->createCharacteristic(
        Ble14::TELEMETRY_UUID, NIMBLE_PROPERTY::NOTIFY, sizeof(Ble14::Robot));
    if (!worldCharacteristic || !telemetryCharacteristic) return false;
    worldCharacteristic->setCallbacks(&worldCallbacks);
    telemetryCharacteristic->setCallbacks(&telemetryCallbacks);
    const uint8_t identity[]{Ble14::VERSION, carId};
    NimBLEAdvertisementData advertisement;
    NimBLEAdvertisementData scanResponse;
    advertisement.setFlags(0x06);
    advertisement.setServiceData(NimBLEUUID(Ble14::SERVICE_UUID), identity, sizeof(identity));
    scanResponse.setName(("e13_car_"+String(carId)).c_str());
    auto* advertising = NimBLEDevice::getAdvertising();
    advertising->setAdvertisementData(advertisement);
    advertising->setScanResponseData(scanResponse);
    advertising->enableScanResponse(true);
    advertising->setAdvertisingInterval(160);
    if (!advertising->start()) return false;
    return true;
}

// Keeps advertising state synchronized with the current connection state.
void BleTelemetry::service(uint32_t) {
    if (!bleServer || !NimBLEDevice::getAdvertising()) return;
    auto* advertising = NimBLEDevice::getAdvertising();
    if (!connected && !advertising->isAdvertising()) advertising->start();
    else if (connected && advertising->isAdvertising()) advertising->stop();
}

// Packages actual car telemetry and sends it at the configured notification cadence.
void BleTelemetry::publishIfDue(uint32_t nowMs, uint32_t periodMs, const RobotTelemetrySnapshot& snapshot) {
    static uint32_t lastPublishMs = 0;
    if (!connected || !subscribed || nowMs-lastPublishMs < periodMs) return;
    lastPublishMs = nowMs;

    Ble14::Robot packet{};
#ifndef BLE_CAR_ID
#define BLE_CAR_ID 1
#endif
    packet.car = BLE_CAR_ID;
    packet.sequence = telemetrySequence++;
    portENTER_CRITICAL(&worldMux);
    packet.worldSequence = haveWorld ? latestWorld.sequence : 0;
    if (nowMs-lastLossWindowMs >= 5000) {
        lastWorldLossCount = uint8_t(windowWorldLost > UINT8_MAX ? UINT8_MAX : windowWorldLost);
        windowWorldLost = 0;
        lastLossWindowMs = nowMs;
    }
    packet.worldLostPrevious5s = lastWorldLossCount;
    portEXIT_CRITICAL(&worldMux);

    memcpy(packet.nfc, snapshot.nfcUid, sizeof(packet.nfc));
    packet.nfcAge = snapshot.nfcAgeMs;
    packet.lineSensorMask = snapshot.lineSensorMask;
    packet.controllerState = snapshot.controllerState;
    packet.lineFollowerState = snapshot.lineFollowerState;
    packet.leftSpeed = snapshot.leftSpeedMmS;
    packet.rightSpeed = snapshot.rightSpeedMmS;
    packet.battery = snapshot.batteryMv;
    telemetryCharacteristic->setValue(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
    telemetryCharacteristic->notify();
}

// Transfers only the newest received World State to the main control loop.
bool BleTelemetry::takeWorldState(Ble14::World& world) {
    bool available = false;
    portENTER_CRITICAL(&worldMux);
    if (haveWorld && worldGeneration != consumedWorldGeneration) {
        world = latestWorld;
        consumedWorldGeneration = worldGeneration;
        available = true;
    }
    portEXIT_CRITICAL(&worldMux);
    return available;
}

// Reports whether the current BLE connection has delivered a World State.
bool BleTelemetry::hasWorldState() const {
    bool seen;
    portENTER_CRITICAL(&worldMux);
    seen = haveWorld;
    portEXIT_CRITICAL(&worldMux);
    return seen;
}

// Returns elapsed time since the current connection's last World State write.
uint32_t BleTelemetry::worldStateAgeMs(uint32_t nowMs) const {
    uint32_t receivedAt;
    bool seen;
    portENTER_CRITICAL(&worldMux);
    receivedAt = lastWorldMs;
    seen = haveWorld;
    portEXIT_CRITICAL(&worldMux);
    return seen ? nowMs-receivedAt : UINT32_MAX;
}
