#include <Arduino.h>
#include <NimBLEDevice.h>

// LEGO Wireless Protocol UUIDs
static NimBLEUUID kServiceUuid("00001623-1212-efde-1623-785feabcd123");
static NimBLEUUID kCharUuid("00001624-1212-efde-1623-785feabcd123");

// Manufacturer data prefix for LEGO (0x0397 little endian)
static const uint8_t kLegoMfgPrefix[] = {0x97, 0x03};

// Hardware mappings (adjust to your wiring)
static const int kPinEncA = 25;    // KY-040 DT
static const int kPinEncB = 26;    // KY-040 CLK
static const int kPinBtnStop = 27; // Stop
static const int kPinBtnHorn = 14; // Horn
static const int kPinBtnLight = 12;// Cycle light colors
static const int kPinBtnWater = 13;// Water/sfx (if supported)

// LED color packets (rotate on light button)
// second last byte is color code
static const uint8_t kLedPackets[][20] = {
    {0x0B, 0x00, 0x81, 0x34, 0x11, 0x51, 0x01, 0x04, 0x01, 0x00, 0x00}, // Off
    {0x0B, 0x00, 0x81, 0x34, 0x11, 0x51, 0x01, 0x04, 0x01, 0x01, 0x00}, // White
    {0x0B, 0x00, 0x81, 0x34, 0x11, 0x51, 0x01, 0x04, 0x01, 0x07, 0x00}, // Green
    {0x0B, 0x00, 0x81, 0x34, 0x11, 0x51, 0x01, 0x04, 0x01, 0x08, 0x00}, // Yellow
    {0x0B, 0x00, 0x81, 0x34, 0x11, 0x51, 0x01, 0x04, 0x01, 0x09, 0x00}, // Light Blue
    {0x0B, 0x00, 0x81, 0x34, 0x11, 0x51, 0x01, 0x04, 0x01, 0x0F, 0x00}, // Dark Blue
    {0x0B, 0x00, 0x81, 0x34, 0x11, 0x51, 0x01, 0x04, 0x01, 0x0A, 0x00}, // Purple
    {0x0B, 0x00, 0x81, 0x34, 0x11, 0x51, 0x01, 0x04, 0x01, 0x0E, 0x00}, // Purple Pink
    {0x0B, 0x00, 0x81, 0x34, 0x11, 0x51, 0x01, 0x04, 0x01, 0x0B, 0x00}, // Light Pink
    {0x0B, 0x00, 0x81, 0x34, 0x11, 0x51, 0x01, 0x04, 0x01, 0x0D, 0x00}, // Red Pink
    {0x0B, 0x00, 0x81, 0x34, 0x11, 0x51, 0x01, 0x04, 0x01, 0x0C, 0x00}, // Red
};
static size_t g_ledIndex = 0;

// Violet action block configurations (cycle on water button)
static const uint8_t kActionPackets[][11] = {
  {0x0B, 0x00, 0x81, 0x34, 0x11, 0x51, 0x01, 0x06, 0x01, 0x06, 0x00}, // Recorded sound
  {0x0B, 0x00, 0x81, 0x34, 0x11, 0x51, 0x01, 0x06, 0x01, 0x02, 0x00}, // cat something
  {0x0B, 0x00, 0x81, 0x34, 0x11, 0x51, 0x01, 0x06, 0x01, 0x00, 0x00}, // nothing
  {0x0B, 0x00, 0x81, 0x34, 0x11, 0x51, 0x01, 0x06, 0x01, 0x01, 0x00}, // beach
  {0x0B, 0x00, 0x81, 0x34, 0x11, 0x51, 0x01, 0x06, 0x01, 0x05, 0x00}, // rain
  {0x0B, 0x00, 0x81, 0x34, 0x11, 0x51, 0x01, 0x06, 0x01, 0x04, 0x00}, // birthday
  {0x0B, 0x00, 0x81, 0x34, 0x11, 0x51, 0x01, 0x06, 0x01, 0x03, 0x00}, // night
};
static size_t g_actionIndex = 0;

//Second last byte is speed 0x64 = 100% speed
static const uint8_t kBckKick2[] = {0x0B, 0x00, 0x81, 0x34, 0x11, 0x51, 0x01, 0x02, 0x01, 0x45, 0x00};
static const uint8_t kFwdKick2[] = {0x0B, 0x00, 0x81, 0x34, 0x11, 0x51, 0x01, 0x01, 0x01, 0x45, 0x00};

// Horn packet (sound effect) from hint
static const uint8_t kHornPacket[] = {0x0B, 0x00, 0x81, 0x34, 0x11, 0x51, 0x01, 0x07, 0x01, 0x00, 0x00};

//0b00813411510104010000
static const uint8_t kStopPacket[] = {0x0B, 0x00, 0x81, 0x34, 0x11, 0x51, 0x01, 0x03, 0x01, 0x00, 0x00};

NimBLEAddress g_targetAddr;
bool g_haveTarget = false;
NimBLEClient *g_client = nullptr;
NimBLERemoteCharacteristic *g_char = nullptr;
bool g_doWriteTest = false;
bool g_scanRunning = false;
unsigned long g_lastScanLogMs = 0;
uint32_t g_advSeen = 0;
bool g_forward = true;

static bool sendForwardSequence() {
  bool ok1 = true;
  bool ok2 = g_char && g_char->writeValue(kFwdKick2, sizeof(kFwdKick2), true);
  return ok1 && ok2;
}

static bool sendBackwardSequence() {
  bool ok1 = true;
  bool ok2 = g_char && g_char->writeValue(kBckKick2, sizeof(kBckKick2), true);
  return ok1 && ok2;
}

struct ButtonState {
  int pin;
  bool last;
};

static ButtonState g_buttons[] = {
  {kPinBtnStop, false},
  {kPinBtnHorn, false}, {kPinBtnLight, false}, {kPinBtnWater, false},
};

// Encoder state
static uint8_t g_encLast = 0;

static bool isLegoTrain(const NimBLEAdvertisedDevice &adv) {
  if (!adv.haveManufacturerData()) return false;
  std::string m = adv.getManufacturerData();
  if (m.size() < 2) return false;
  return memcmp(m.data(), kLegoMfgPrefix, 2) == 0;
}

class AdvCallbacks : public NimBLEScanCallbacks {
  void handleResult(const NimBLEAdvertisedDevice *adv) {
    g_advSeen++;
    size_t mlen = adv->haveManufacturerData() ? adv->getManufacturerData().size() : 0;
    if (g_advSeen <= 5 || (g_advSeen % 50 == 0)) {
      Serial.printf("Adv #%u from %s RSSI=%d mfgLen=%u\n", g_advSeen, adv->getAddress().toString().c_str(), adv->getRSSI(), (unsigned)mlen);
    }
    if (isLegoTrain(*adv)) {
      Serial.printf("Found candidate %s RSSI=%d\n", adv->getAddress().toString().c_str(), adv->getRSSI());
      g_targetAddr = adv->getAddress();
      g_haveTarget = true;
      NimBLEDevice::getScan()->stop();
      g_scanRunning = false;
    }
  }
  // Cover both signatures to match NimBLE 2.3.x headers.
  void onResult(const NimBLEAdvertisedDevice *adv) override { handleResult(adv); }
  void onResult(NimBLEAdvertisedDevice *adv) { handleResult(adv); }
  void onScanEnd(NimBLEScanResults results) {
    Serial.println("Scan ended");
    g_scanRunning = false;
  }
  void onScanEnd(const NimBLEScanResults *results) {
    Serial.println("Scan ended (ptr)");
    g_scanRunning = false;
  }
};

class ClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient *c) {
    Serial.println("Connected");
    c->updateConnParams(24, 24, 0, 120);  // reasonable defaults
  }
  void onDisconnect(NimBLEClient *c) {
    Serial.println("Disconnected; restarting scan");
    g_char = nullptr;
    g_client = nullptr;
    if (!g_scanRunning) {
      Serial.println("Restarting scan after disconnect");
      NimBLEDevice::getScan()->start(0, false, false);
      g_scanRunning = true;
    }
  }
  bool onConnParamsUpdateRequest(NimBLEClient *c, const ble_gap_upd_params *params) {
    return true;
  }
};

static bool connectAndDiscover() {
  if (!g_haveTarget) return false;
  Serial.printf("Connecting to %s\n", g_targetAddr.toString().c_str());
  g_client = NimBLEDevice::createClient();
  g_client->setClientCallbacks(new ClientCallbacks(), false);

  // Ensure we allow bonding + encryption like the phone does.
  NimBLEDevice::setSecurityAuth(true, true, true);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

  if (!g_client->connect(g_targetAddr)) {
    Serial.println("Connect failed; restarting scan");
    if (!g_scanRunning) {
      NimBLEDevice::getScan()->start(0, false, false);
      g_scanRunning = true;
    }
    g_haveTarget = false;
    return false;
  }

  NimBLERemoteService *svc = g_client->getService(kServiceUuid);
  if (!svc) {
    Serial.println("Service not found");
    g_client->disconnect();
    if (!g_scanRunning) {
      NimBLEDevice::getScan()->start(0, false, false);
      g_scanRunning = true;
    }
    return false;
  }

  g_char = svc->getCharacteristic(kCharUuid);
  if (!g_char) {
    Serial.println("Characteristic not found");
    g_client->disconnect();
    if (!g_scanRunning) {
      NimBLEDevice::getScan()->start(0, false, false);
      g_scanRunning = true;
    }
    return false;
  }

  // Try to enable notifications; some hubs ignore, but it's harmless.
  if (g_char->canNotify()) {
    g_char->subscribe(true, [](NimBLERemoteCharacteristic *, uint8_t *data, size_t len, bool) {
      Serial.print("Notify: ");
      for (size_t i = 0; i < len; ++i) Serial.printf("%02X ", data[i]);
      Serial.println();
    });
  }

  g_doWriteTest = true;
  Serial.println("Ready; will send LED test");
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Duplo NimBLE test");

  NimBLEDevice::init("");
  // Clear any stale bonds so the hub will pair fresh on each boot.
  NimBLEDevice::deleteAllBonds();
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setSecurityAuth(true, true, true);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_RANDOM);
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(new AdvCallbacks(), true);  // allow duplicates so we keep seeing adverts
  scan->setInterval(45);
  scan->setWindow(30);
  scan->setActiveScan(true);
  Serial.println("Starting scan");
  bool ok = scan->start(0, false, false);  // continuous until stop
  Serial.printf("Scan start %s\n", ok ? "ok" : "FAILED");
  g_scanRunning = ok;

  pinMode(kPinEncA, INPUT_PULLUP);
  pinMode(kPinEncB, INPUT_PULLUP);
  g_encLast = (digitalRead(kPinEncA) << 1) | digitalRead(kPinEncB);
  for (auto &b : g_buttons) {
    pinMode(b.pin, INPUT_PULLUP);
    b.last = digitalRead(b.pin) == LOW;
  }
}

void loop() {
  if (!g_client || !g_client->isConnected()) {
    // Scanner will pick up and set g_target; connect when found.
    if (g_haveTarget && !g_client) {
      connectAndDiscover();
    } else if (!g_scanRunning) {
      Serial.println("Scan not running; restarting");
      NimBLEDevice::getScan()->start(0, false, false);
      g_scanRunning = true;
    } else {
      unsigned long now = millis();
      if (now - g_lastScanLogMs > 3000) {
        Serial.printf("Scanning... no LEGO adverts yet (isScanning=%d, advSeen=%u)\n", NimBLEDevice::getScan()->isScanning(), g_advSeen);
        if (!NimBLEDevice::getScan()->isScanning()) {
          Serial.println("Scan stopped unexpectedly; restarting");
          NimBLEDevice::getScan()->start(0, false, false);
          g_scanRunning = true;
        }
        g_lastScanLogMs = now;
        if (g_advSeen == 0 && now > 10000) {
          Serial.println("Still no adverts seen at all; is the train awake and blinking?");
        }
      }
    }
    delay(50);
    return;
  }

  // Read buttons with edge detection (active low)
  for (auto &b : g_buttons) {
    bool now = digitalRead(b.pin) == LOW;
    if (now && !b.last) {
      // Rising edge
      if (b.pin == kPinBtnStop) {
        g_char->writeValue(kStopPacket, sizeof(kStopPacket), true);
        Serial.println("Button: Stop (stop packet)");
      } else if (b.pin == kPinBtnHorn) {
        g_char->writeValue(kHornPacket, sizeof(kHornPacket), true);
        Serial.println("Button: Horn");
      } else if (b.pin == kPinBtnLight) {
        g_ledIndex = (g_ledIndex + 1) % (sizeof(kLedPackets) / sizeof(kLedPackets[0]));
        g_char->writeValue(kLedPackets[g_ledIndex], sizeof(kLedPackets[0]), true);
        Serial.printf("Button: Light idx=%u\n", (unsigned)g_ledIndex);
      } else if (b.pin == kPinBtnWater) {
        g_actionIndex = (g_actionIndex + 1) % (sizeof(kActionPackets) / sizeof(kActionPackets[0]));
        g_char->writeValue(kActionPackets[g_actionIndex], sizeof(kActionPackets[0]), true);
        Serial.printf("Button: Water/action idx=%u\n", (unsigned)g_actionIndex);
      }
    }
    b.last = now;
  }

  // Encoder -> direction
  uint8_t enc = (digitalRead(kPinEncA) << 1) | digitalRead(kPinEncB);
  if (enc != g_encLast) {
    // Simple quadrature decode
    if ((g_encLast == 0b00 && enc == 0b01) ||
        (g_encLast == 0b01 && enc == 0b11) ||
        (g_encLast == 0b11 && enc == 0b10) ||
        (g_encLast == 0b10 && enc == 0b00)) {
      bool ok = sendForwardSequence();
      g_forward = true;
      Serial.printf("Encoder -> Forward (%s) state %u->%u\n", ok ? "ok" : "fail", g_encLast, enc);
    } else if ((g_encLast == 0b00 && enc == 0b10) ||
               (g_encLast == 0b10 && enc == 0b11) ||
               (g_encLast == 0b11 && enc == 0b01) ||
               (g_encLast == 0b01 && enc == 0b00)) {
      bool ok = sendBackwardSequence();
      g_forward = false;
      Serial.printf("Encoder -> Backward (%s) state %u->%u\n", ok ? "ok" : "fail", g_encLast, enc);
    }
    g_encLast = enc;
  }

  delay(50);
}
