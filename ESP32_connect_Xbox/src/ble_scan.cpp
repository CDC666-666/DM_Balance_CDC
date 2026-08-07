#include <Arduino.h>
#include <NimBLEDevice.h>

namespace {

struct SeenDevice {
  std::string address;
  unsigned long printedAt = 0;
};

SeenDevice seenDevices[24];

bool shouldPrint(const std::string& address) {
  const unsigned long now = millis();
  for (SeenDevice& seen : seenDevices) {
    if (seen.address == address) {
      if (now - seen.printedAt < 5000) {
        return false;
      }
      seen.printedAt = now;
      return true;
    }
  }
  for (SeenDevice& seen : seenDevices) {
    if (seen.address.empty()) {
      seen.address = address;
      seen.printedAt = now;
      return true;
    }
  }
  return false;
}

class ScanCallbacks : public NimBLEAdvertisedDeviceCallbacks {
 public:
  void onResult(NimBLEAdvertisedDevice* device) override {
    const std::string name = device->getName();
    const std::string address = device->getAddress().toString();
    const bool isTarget = name.find("GameSir") != std::string::npos ||
                          name.find("Nova") != std::string::npos;

    if (!shouldPrint(address)) {
      return;
    }

    Serial.printf("%s rssi=%d %s\n", isTarget ? "[TARGET]" : "[BLE]",
                  device->getRSSI(), device->toString().c_str());
  }
};

ScanCallbacks scanCallbacks;

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("BLE_SCAN_READY");
  Serial.println("Keep the controller in pairing mode.");

  NimBLEDevice::init("");
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(&scanCallbacks, true);
  scan->setActiveScan(true);
  scan->setInterval(45);
  scan->setWindow(30);
  scan->start(0, nullptr, false);
}

void loop() {
  static unsigned long lastHeartbeatAt = 0;
  if (millis() - lastHeartbeatAt >= 3000) {
    Serial.println("[SCAN] searching for GameSir/Nova...");
    lastHeartbeatAt = millis();
  }
  delay(50);
}
