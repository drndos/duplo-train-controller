#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Preferences.h>

// Report layout of Xbox Series / One S (BLE firmware) controllers.
// Defined before any function so the Arduino-generated prototypes can see it.
struct PadState {
  uint16_t lx, ly, rx, ry;  // sticks 0..65535, centre ~32768
  uint16_t lt, rt;          // triggers 0..1023
  uint8_t dpad;             // 0 none, 1 up, 2 up-right ... 8 up-left
  bool a, b, x, y, lb, rb, view, menu, xbox, ls, rs, share;
};

// LEGO Wireless Protocol UUIDs
static NimBLEUUID kServiceUuid("00001623-1212-efde-1623-785feabcd123");
static NimBLEUUID kCharUuid("00001624-1212-efde-1623-785feabcd123");

// Bluetooth HID (Xbox controller) UUIDs
static NimBLEUUID kHidServiceUuid((uint16_t)0x1812);
static NimBLEUUID kHidReportUuid((uint16_t)0x2A4D);
static const uint16_t kAppearanceGamepad = 0x03C4;

// Manufacturer data prefix for LEGO (0x0397 little endian)
static const uint8_t kLegoMfgPrefix[] = {0x97, 0x03};

// Hardware mappings (adjust to your wiring)
static const int kPinEncA = 25;    // KY-040 DT
static const int kPinEncB = 26;    // KY-040 CLK
static const int kPinBtnStop = 27; // Stop (hold during boot to forget the Xbox controller)
static const int kPinBtnHorn = 14; // Horn
static const int kPinBtnLight = 12;// Cycle light colors
static const int kPinBtnWater = 13;// Water/sfx (if supported)

// Keep the same ESP32 BLE address across reboots so a paired Xbox controller
// reconnects on its own. If the train stops connecting with this enabled, set it
// to false (the controller then has to be put into pairing mode on every boot).
static const bool kStableAddress = true;

// Print controller discovery, raw HID reports and report counts to Serial.
static const bool kXboxDebug = false;

// Throttle tuning. Triggers (RT forward, LT backward) and the vertical axis of
// both sticks (up forward, down backward) all drive; the strongest input wins.
static const uint16_t kTriggerDeadzone = 80;      // triggers report 0..1023
static const uint16_t kStickDeadzone = 200;       // stick tilt on the same 0..1023 scale
static const bool kStickUpIsLow = true;           // Xbox reports stick up as low values
static const uint8_t kMinSpeed = 0x14;            // speed byte at a light press
static const uint8_t kMaxSpeed = 0x64;            // 0x64 = 100%
static const uint8_t kSpeedStep = 5;              // only resend when speed moves this much
static const unsigned long kThrottleIntervalMs = 120;

// LED color packets (rotate on light button)
// second last byte is color code
static const uint8_t kLedPackets[][11] = {
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
static const size_t kLedCount = sizeof(kLedPackets) / sizeof(kLedPackets[0]);
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
static const size_t kActionCount = sizeof(kActionPackets) / sizeof(kActionPackets[0]);
static size_t g_actionIndex = 0;

//Second last byte is speed 0x64 = 100% speed
static const uint8_t kDefaultSpeed = 0x45;
static const uint8_t kCmdForward = 0x01;
static const uint8_t kCmdBackward = 0x02;

// Horn packet (sound effect) from hint
static const uint8_t kHornPacket[] = {0x0B, 0x00, 0x81, 0x34, 0x11, 0x51, 0x01, 0x07, 0x01, 0x00, 0x00};

//0b00813411510104010000
static const uint8_t kStopPacket[] = {0x0B, 0x00, 0x81, 0x34, 0x11, 0x51, 0x01, 0x03, 0x01, 0x00, 0x00};

// ---------------------------------------------------------------------------
// Connection state
// ---------------------------------------------------------------------------

Preferences g_prefs;

NimBLEAddress g_trainAddr;
volatile bool g_haveTrainTarget = false;
NimBLEClient *g_trainClient = nullptr;
NimBLERemoteCharacteristic *g_char = nullptr;

NimBLEAddress g_padAddr;          // candidate found by the scanner
NimBLEAddress g_savedPadAddr;     // controller we bonded with earlier (may be null)
volatile bool g_havePadTarget = false;
NimBLEClient *g_padClient = nullptr;

unsigned long g_lastScanLogMs = 0;
uint32_t g_advSeen = 0;
bool g_forward = true;

static bool trainConnected() {
  return g_trainClient && g_trainClient->isConnected() && g_char;
}

static bool padConnected() {
  return g_padClient && g_padClient->isConnected();
}

// ---------------------------------------------------------------------------
// Train actions (shared by the physical buttons and the Xbox controller)
// ---------------------------------------------------------------------------

static bool trainSend(const uint8_t *data, size_t len) {
  if (!trainConnected()) return false;
  return g_char->writeValue(data, len, true);
}

static bool trainDrive(bool forward, uint8_t speed) {
  uint8_t pkt[] = {0x0B, 0x00, 0x81, 0x34, 0x11, 0x51, 0x01,
                   forward ? kCmdForward : kCmdBackward, 0x01, speed, 0x00};
  g_forward = forward;
  return trainSend(pkt, sizeof(pkt));
}

static void trainStop() {
  trainSend(kStopPacket, sizeof(kStopPacket));
  Serial.println("Stop");
}

static void trainHorn() {
  trainSend(kHornPacket, sizeof(kHornPacket));
  Serial.println("Horn");
}

static void trainLightStep(int dir) {
  g_ledIndex = (g_ledIndex + kLedCount + dir) % kLedCount;
  trainSend(kLedPackets[g_ledIndex], sizeof(kLedPackets[0]));
  Serial.printf("Light idx=%u\n", (unsigned)g_ledIndex);
}

static void trainActionStep(int dir) {
  g_actionIndex = (g_actionIndex + kActionCount + dir) % kActionCount;
  trainSend(kActionPackets[g_actionIndex], sizeof(kActionPackets[0]));
  Serial.printf("Action idx=%u\n", (unsigned)g_actionIndex);
}

// ---------------------------------------------------------------------------
// Xbox controller input
// ---------------------------------------------------------------------------

static portMUX_TYPE g_padMux = portMUX_INITIALIZER_UNLOCKED;
static PadState g_padShared = {};
static volatile bool g_padFresh = false;
static uint8_t g_padRaw[20];
static volatile size_t g_padRawLen = 0;
static volatile uint32_t g_padReports = 0;      // every notification received
static volatile uint32_t g_padShortReports = 0; // notifications too short to parse
static volatile size_t g_padLastLen = 0;
static unsigned long g_lastPadStatsMs = 0;
static uint8_t g_padLastPrinted[sizeof(g_padRaw)];

static PadState g_padPrev = {};
static bool g_throttleActive = false;
static bool g_throttleForward = true;
static uint8_t g_throttleSpeed = 0;
static unsigned long g_lastThrottleMs = 0;

static void parsePadReport(const uint8_t *d, PadState &s) {
  s.lx = d[0] | (d[1] << 8);
  s.ly = d[2] | (d[3] << 8);
  s.rx = d[4] | (d[5] << 8);
  s.ry = d[6] | (d[7] << 8);
  s.lt = d[8] | (d[9] << 8);
  s.rt = d[10] | (d[11] << 8);
  s.dpad = d[12];
  s.a = d[13] & 0x01;
  s.b = d[13] & 0x02;
  s.x = d[13] & 0x08;
  s.y = d[13] & 0x10;
  s.lb = d[13] & 0x40;
  s.rb = d[13] & 0x80;
  s.view = d[14] & 0x04;
  s.menu = d[14] & 0x08;
  s.xbox = d[14] & 0x10;
  s.ls = d[14] & 0x20;
  s.rs = d[14] & 0x40;
  s.share = d[15] & 0x01;
}

// Runs on the NimBLE host task: only copy the data, never write to the train here.
static void onPadReport(NimBLERemoteCharacteristic *, uint8_t *data, size_t len, bool) {
  g_padReports = g_padReports + 1;
  g_padLastLen = len;
  if (len < 16) {
    g_padShortReports = g_padShortReports + 1;
    return;
  }
  PadState s;
  parsePadReport(data, s);
  portENTER_CRITICAL(&g_padMux);
  g_padShared = s;
  g_padFresh = true;
  size_t n = len < sizeof(g_padRaw) ? len : sizeof(g_padRaw);
  memcpy(g_padRaw, data, n);
  g_padRawLen = n;
  portEXIT_CRITICAL(&g_padMux);
}

static bool dpadUp(uint8_t d) { return d == 1 || d == 2 || d == 8; }
static bool dpadDown(uint8_t d) { return d == 4 || d == 5 || d == 6; }

// Scale a 0..1023 input past its deadzone to 0..1023 (0 = inside the deadzone).
static int32_t pastDeadzone(int32_t v, int32_t deadzone) {
  if (v > 1023) v = 1023;
  if (v <= deadzone) return 0;
  return (v - deadzone) * 1023 / (1023 - deadzone);
}

// Stick vertical axis -> -1023 (full down) .. +1023 (full up).
static int32_t stickUp(uint16_t raw) {
  int32_t v = ((int32_t)raw - 32768) / 32;
  return kStickUpIsLow ? -v : v;
}

static uint8_t amountToSpeed(int32_t amount) {
  uint32_t speed = kMinSpeed + (uint32_t)amount * (kMaxSpeed - kMinSpeed) / 1023;
  speed = (speed / kSpeedStep) * kSpeedStep;
  if (speed < kMinSpeed) speed = kMinSpeed;
  return (uint8_t)speed;
}

static void releaseThrottle(const char *why) {
  if (!g_throttleActive) return;
  g_throttleActive = false;
  trainStop();
  Serial.printf("Throttle released (%s)\n", why);
}

static void handleThrottle(const PadState &s) {
  // Every input gives a signed amount (+ forward, - backward); the strongest one wins.
  int32_t ly = stickUp(s.ly), ry = stickUp(s.ry);
  const int32_t inputs[] = {
    pastDeadzone(s.rt, kTriggerDeadzone),
    -pastDeadzone(s.lt, kTriggerDeadzone),
    ly >= 0 ? pastDeadzone(ly, kStickDeadzone) : -pastDeadzone(-ly, kStickDeadzone),
    ry >= 0 ? pastDeadzone(ry, kStickDeadzone) : -pastDeadzone(-ry, kStickDeadzone),
  };
  int32_t best = 0;
  for (int32_t v : inputs) {
    if (abs(v) > abs(best)) best = v;
  }
  if (best == 0) {
    releaseThrottle("triggers and sticks released");
    return;
  }
  bool forward = best > 0;
  uint8_t speed = amountToSpeed(abs(best));
  bool changed = !g_throttleActive || forward != g_throttleForward || speed != g_throttleSpeed;
  unsigned long now = millis();
  if (!changed || now - g_lastThrottleMs < kThrottleIntervalMs) return;
  bool ok = trainDrive(forward, speed);
  Serial.printf("Throttle %s speed=0x%02X (%s)\n", forward ? "forward" : "backward", speed, ok ? "ok" : "fail");
  g_throttleActive = true;
  g_throttleForward = forward;
  g_throttleSpeed = speed;
  g_lastThrottleMs = now;
}

static void processPad() {
  if (!padConnected()) {
    releaseThrottle("controller lost");
    g_padPrev = PadState{};
    return;
  }

  PadState s;
  uint8_t raw[sizeof(g_padRaw)];
  size_t rawLen;
  bool fresh;
  portENTER_CRITICAL(&g_padMux);
  s = g_padShared;
  fresh = g_padFresh;
  g_padFresh = false;
  rawLen = g_padRawLen;
  memcpy(raw, g_padRaw, rawLen);
  portEXIT_CRITICAL(&g_padMux);

  // Print raw reports, skipping ones where only the sticks (bytes 0-7) moved.
  if (fresh && kXboxDebug && rawLen > 8 && memcmp(raw + 8, g_padLastPrinted + 8, rawLen - 8) != 0) {
    memcpy(g_padLastPrinted, raw, rawLen);
    Serial.print("Pad: ");
    for (size_t i = 0; i < rawLen; ++i) Serial.printf("%02X ", raw[i]);
    Serial.println();
  }
  if (kXboxDebug && millis() - g_lastPadStatsMs > 5000) {
    Serial.printf("Pad stats: reports=%u short=%u lastLen=%u\n", (unsigned)g_padReports,
                  (unsigned)g_padShortReports, (unsigned)g_padLastLen);
    g_lastPadStatsMs = millis();
  }

  if (!trainConnected()) {
    g_padPrev = s;
    return;
  }

  const PadState &p = g_padPrev;
  if (s.a && !p.a) trainHorn();
  if (s.b && !p.b) {
    g_throttleActive = false;
    trainStop();
  }
  if (s.x && !p.x) trainLightStep(+1);
  if (s.lb && !p.lb) trainLightStep(-1);
  if (s.y && !p.y) trainActionStep(+1);
  if (s.rb && !p.rb) trainActionStep(-1);
  if (dpadUp(s.dpad) && !dpadUp(p.dpad)) {
    g_throttleActive = false;
    Serial.printf("D-pad -> Forward (%s)\n", trainDrive(true, kDefaultSpeed) ? "ok" : "fail");
  }
  if (dpadDown(s.dpad) && !dpadDown(p.dpad)) {
    g_throttleActive = false;
    Serial.printf("D-pad -> Backward (%s)\n", trainDrive(false, kDefaultSpeed) ? "ok" : "fail");
  }

  handleThrottle(s);
  g_padPrev = s;
}

// ---------------------------------------------------------------------------
// Scanning and connecting
// ---------------------------------------------------------------------------

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

static bool isGamepad(const NimBLEAdvertisedDevice &adv) {
  if (!g_savedPadAddr.isNull() && adv.getAddress() == g_savedPadAddr) return true;
  if (adv.isAdvertisingService(kHidServiceUuid)) return true;
  if (adv.haveAppearance() && adv.getAppearance() == kAppearanceGamepad) return true;
  return adv.haveName() && adv.getName().find("Xbox") != std::string::npos;
}

static void startScan() {
  NimBLEScan *scan = NimBLEDevice::getScan();
  if (!scan->isScanning()) scan->start(0, false, false);
}

class AdvCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice *adv) override {
    g_advSeen++;
    size_t mlen = adv->haveManufacturerData() ? adv->getManufacturerData().size() : 0;
    if (g_advSeen <= 5 || (g_advSeen % 50 == 0)) {
      Serial.printf("Adv #%u from %s RSSI=%d mfgLen=%u\n", (unsigned)g_advSeen, adv->getAddress().toString().c_str(), adv->getRSSI(), (unsigned)mlen);
    }
    if (!trainConnected() && !g_haveTrainTarget && isLegoTrain(*adv)) {
      Serial.printf("Found train %s RSSI=%d\n", adv->getAddress().toString().c_str(), adv->getRSSI());
      g_trainAddr = adv->getAddress();
      g_haveTrainTarget = true;
      NimBLEDevice::getScan()->stop();
    } else if (!padConnected() && !g_havePadTarget && isGamepad(*adv)) {
      Serial.printf("Found controller %s \"%s\" RSSI=%d\n", adv->getAddress().toString().c_str(),
                    adv->getName().c_str(), adv->getRSSI());
      g_padAddr = adv->getAddress();
      g_havePadTarget = true;
      NimBLEDevice::getScan()->stop();
    }
  }
};

class TrainClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient *c) override {
    Serial.println("Train connected");
    c->updateConnParams(24, 24, 0, 120);  // reasonable defaults
  }
  void onDisconnect(NimBLEClient *c, int reason) override {
    Serial.printf("Train disconnected (reason %d)\n", reason);
    g_char = nullptr;
  }
};

class PadClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient *c) override {
    Serial.println("Controller connected");
    // The controller asks for 7.5 ms intervals, which leaves too little airtime to
    // connect the train afterwards. 15 ms is plenty for driving a train.
    c->updateConnParams(12, 12, 0, 200);
  }
  void onDisconnect(NimBLEClient *c, int reason) override {
    Serial.printf("Controller disconnected (reason %d)\n", reason);
  }
  void onAuthenticationComplete(NimBLEConnInfo &info) override {
    Serial.printf("Controller pairing %s (bonded=%d)\n", info.isEncrypted() ? "ok" : "FAILED", info.isBonded());
  }
};

static bool connectTrain() {
  g_haveTrainTarget = false;
  Serial.printf("Connecting to train %s\n", g_trainAddr.toString().c_str());
  if (!g_trainClient) {
    g_trainClient = NimBLEDevice::createClient();
    g_trainClient->setClientCallbacks(new TrainClientCallbacks(), true);
    g_trainClient->setConnectTimeout(5000);
  }

  if (!g_trainClient->connect(g_trainAddr)) {
    Serial.println("Train connect failed");
    return false;
  }

  NimBLERemoteService *svc = g_trainClient->getService(kServiceUuid);
  NimBLERemoteCharacteristic *chr = svc ? svc->getCharacteristic(kCharUuid) : nullptr;
  if (!chr) {
    Serial.println("Train service/characteristic not found");
    g_trainClient->disconnect();
    return false;
  }

  // Try to enable notifications; some hubs ignore, but it's harmless.
  if (chr->canNotify()) {
    chr->subscribe(true, [](NimBLERemoteCharacteristic *, uint8_t *data, size_t len, bool) {
      Serial.print("Train notify: ");
      for (size_t i = 0; i < len; ++i) Serial.printf("%02X ", data[i]);
      Serial.println();
    });
  }

  g_char = chr;
  Serial.println("Train ready");
  return true;
}

static bool connectPad() {
  g_havePadTarget = false;
  Serial.printf("Connecting to controller %s\n", g_padAddr.toString().c_str());
  if (!g_padClient) {
    g_padClient = NimBLEDevice::createClient();
    g_padClient->setClientCallbacks(new PadClientCallbacks(), true);
    g_padClient->setConnectTimeout(5000);
  }

  if (!g_padClient->connect(g_padAddr)) {
    Serial.println("Controller connect failed");
    return false;
  }

  // The Xbox controller only sends input after the link is encrypted (paired).
  if (!g_padClient->secureConnection()) {
    // Stale keys (e.g. the controller was re-paired): forget them so the next attempt pairs fresh.
    if (NimBLEDevice::isBonded(g_padAddr)) {
      NimBLEDevice::deleteBond(g_padAddr);
      Serial.println("Controller pairing failed; deleted old keys, will pair fresh on next attempt");
    } else {
      Serial.println("Controller pairing failed; hold its pair button until it blinks fast");
    }
    g_padClient->disconnect();
    return false;
  }

  // Behave like a regular HID-over-GATT host: discover everything and read every
  // readable value (report map, report references, device info). The controller
  // keeps blinking and sends no input until the host has done this.
  for (NimBLERemoteService *svc : g_padClient->getServices(true)) {
    for (NimBLERemoteCharacteristic *chr : svc->getCharacteristics(true)) {
      size_t len = chr->canRead() ? chr->readValue().length() : 0;
      if (kXboxDebug) {
        Serial.printf("  %s / %s len=%u\n", svc->getUUID().toString().c_str(),
                      chr->getUUID().toString().c_str(), (unsigned)len);
      }
      for (NimBLERemoteDescriptor *dsc : chr->getDescriptors(true)) {
        if (dsc->getUUID() == NimBLEUUID((uint16_t)0x2908)) dsc->readValue();  // Report Reference
      }
    }
  }

  NimBLERemoteService *hid = g_padClient->getService(kHidServiceUuid);
  if (!hid) {
    Serial.println("Controller has no HID service (not a gamepad?)");
    g_padClient->disconnect();
    return false;
  }

  int subscribed = 0;
  for (NimBLERemoteCharacteristic *chr : hid->getCharacteristics(true)) {
    if (chr->getUUID() == kHidReportUuid && chr->canNotify() && chr->subscribe(true, onPadReport)) {
      subscribed++;
    }
  }
  if (subscribed == 0) {
    Serial.println("Controller: could not subscribe to input reports");
    g_padClient->disconnect();
    return false;
  }

  NimBLEAddress addr = g_padClient->getPeerAddress();
  if (!(addr == g_savedPadAddr)) {
    g_savedPadAddr = addr;
    g_prefs.putString("padAddr", addr.toString().c_str());
    g_prefs.putUChar("padType", addr.getType());
    Serial.printf("Remembered controller %s\n", addr.toString().c_str());
  }
  Serial.printf("Controller ready (%d input reports)\n", subscribed);
  return true;
}

// Delete every bond except the remembered controller so the train pairs fresh on each boot.
static void pruneBonds() {
  for (int i = NimBLEDevice::getNumBonds() - 1; i >= 0; --i) {
    NimBLEAddress addr = NimBLEDevice::getBondedAddress(i);
    if (!g_savedPadAddr.isNull() && addr == g_savedPadAddr) continue;
    NimBLEDevice::deleteBond(addr);
  }
}

static void forgetController() {
  Serial.println("Stop button held at boot: forgetting the Xbox controller");
  g_prefs.remove("padAddr");
  g_prefs.remove("padType");
  g_savedPadAddr = NimBLEAddress();
}

static void loadOrCreateOwnAddress() {
  uint8_t addr[6];
  if (g_prefs.getBytes("ownAddr", addr, sizeof(addr)) != sizeof(addr)) {
    esp_fill_random(addr, sizeof(addr));
    addr[5] |= 0xC0;  // random static address: two top bits set
    g_prefs.putBytes("ownAddr", addr, sizeof(addr));
  }
  NimBLEDevice::setOwnAddr(addr);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Duplo NimBLE + Xbox controller");

  pinMode(kPinEncA, INPUT_PULLUP);
  pinMode(kPinEncB, INPUT_PULLUP);
  g_encLast = (digitalRead(kPinEncA) << 1) | digitalRead(kPinEncB);
  for (auto &b : g_buttons) {
    pinMode(b.pin, INPUT_PULLUP);
    b.last = digitalRead(b.pin) == LOW;
  }

  g_prefs.begin("duplo", false);
  if (digitalRead(kPinBtnStop) == LOW) {
    forgetController();
  } else if (g_prefs.isKey("padAddr")) {
    g_savedPadAddr = NimBLEAddress(std::string(g_prefs.getString("padAddr").c_str()), g_prefs.getUChar("padType"));
    Serial.printf("Known controller %s\n", g_savedPadAddr.toString().c_str());
  }

  NimBLEDevice::init("");
  pruneBonds();
  NimBLEDevice::setPower(ESP_PWR_LVL_P6);
  NimBLEDevice::setSecurityAuth(true, true, true);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  if (kStableAddress) loadOrCreateOwnAddress();
  NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_RANDOM);
  Serial.printf("ESP32 BLE address %s\n", NimBLEDevice::getAddress().toString().c_str());

  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(new AdvCallbacks(), true);  // allow duplicates so we keep seeing adverts
  scan->setInterval(45);
  scan->setWindow(30);
  scan->setActiveScan(true);
  Serial.println("Starting scan");
  startScan();
}

static void maintainConnections() {
  // Connecting blocks, so the scan is stopped by the scanner before we get here.
  if (g_haveTrainTarget && !trainConnected()) {
    connectTrain();
  } else if (g_havePadTarget && !padConnected()) {
    connectPad();
  }

  bool needScan = !trainConnected() || !padConnected();
  NimBLEScan *scan = NimBLEDevice::getScan();
  if (needScan && !g_haveTrainTarget && !g_havePadTarget) {
    startScan();
  } else if (!needScan && scan->isScanning()) {
    scan->stop();
  }

  unsigned long now = millis();
  if (needScan && now - g_lastScanLogMs > 3000) {
    Serial.printf("Scanning... train=%s controller=%s advSeen=%u\n",
                  trainConnected() ? "connected" : "searching",
                  padConnected() ? "connected" : "searching", (unsigned)g_advSeen);
    if (!trainConnected() && g_advSeen == 0 && now > 10000) {
      Serial.println("Still no adverts seen at all; is the train awake and blinking?");
    }
    g_lastScanLogMs = now;
  }
}

static void processPhysicalControls() {
  // Read buttons with edge detection (active low)
  for (auto &b : g_buttons) {
    bool now = digitalRead(b.pin) == LOW;
    if (now && !b.last) {
      // Rising edge
      if (b.pin == kPinBtnStop) {
        trainStop();
      } else if (b.pin == kPinBtnHorn) {
        trainHorn();
      } else if (b.pin == kPinBtnLight) {
        trainLightStep(+1);
      } else if (b.pin == kPinBtnWater) {
        trainActionStep(+1);
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
      bool ok = trainDrive(true, kDefaultSpeed);
      Serial.printf("Encoder -> Forward (%s) state %u->%u\n", ok ? "ok" : "fail", g_encLast, enc);
    } else if ((g_encLast == 0b00 && enc == 0b10) ||
               (g_encLast == 0b10 && enc == 0b11) ||
               (g_encLast == 0b11 && enc == 0b01) ||
               (g_encLast == 0b01 && enc == 0b00)) {
      bool ok = trainDrive(false, kDefaultSpeed);
      Serial.printf("Encoder -> Backward (%s) state %u->%u\n", ok ? "ok" : "fail", g_encLast, enc);
    }
    g_encLast = enc;
  }
}

void loop() {
  maintainConnections();
  processPhysicalControls();
  processPad();
  delay(trainConnected() ? 20 : 50);
}
