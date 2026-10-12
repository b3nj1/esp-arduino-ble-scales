#include "remote_scales.h"
#include "remote_scales_plugin_registry.h"

// ---------------------------------------------------------------------------------------
// ------------------------   Common RemoteScales methods    ------------------------------
// ---------------------------------------------------------------------------------------

RemoteScales::RemoteScales(const DiscoveredDevice& device) : device(device) {}

void RemoteScales::log(std::string msgFormat, ...) {
  if (!this->logCallback) return;

  va_list args;
  va_start(args, msgFormat);
  int length = vsnprintf(nullptr, 0, msgFormat.c_str(), args); // Find length of string
  va_end(args); // End before restarting

  if (length < 0) {
    logCallback("Scale[" + device.getName() + "] Error: Invalid message format");
    return;
  }

  va_start(args, msgFormat); // Restart for the actual printing
  std::string formattedMessage(length + 1, '\0'); // Instantiate formatted strigng with correct length
  vsnprintf(&formattedMessage[0], length + 1, msgFormat.c_str(), args); // print formatted message in the string
  formattedMessage.resize(length); // Remove the trailing null character
  va_end(args);
  logCallback("Scale[" + device.getName() + "] " + formattedMessage);
}

void RemoteScales::setWeight(float newWeight) {
  float previousWeight = weight;
  weight = newWeight;

  if (weightCallback == nullptr) {
    return;
  }
  if (weightCallbackOnlyChanges && previousWeight == newWeight) {
    return;
  }
  weightCallback(newWeight);
}

void RemoteScales::setWeightUpdatedCallback(void (*callback)(float), bool onlyChanges) {
  weightCallbackOnlyChanges = onlyChanges;
  this->weightCallback = callback;
}

// NimBLE's 30 s default keeps the caller blocked and the radio initiating long after a scale has stopped advertising.
static constexpr uint8_t CONNECT_TIMEOUT_SECONDS = 5;

bool RemoteScales::clientConnect() {
  clientCleanup();
  log("Connecting to BLE client\n");
  client = NimBLEDevice::createClient(device.getAddress());
  client->setConnectTimeout(CONNECT_TIMEOUT_SECONDS);
  return client->connect();
}

void RemoteScales::clientCleanup() {
  if (client == nullptr) {
    return;
  }
  log("Cleaning up BLE client\n");
  NimBLEDevice::deleteClient(client);
  client = nullptr;
}

NimBLERemoteService* RemoteScales::clientGetService(const NimBLEUUID uuid) {
  if (!clientIsConnected()) {
    log("Cannot get service, client is not connected\n");
    return nullptr;
  }
  return client->getService(uuid);
}

bool RemoteScales::clientIsConnected() { return client != nullptr && client->isConnected(); };

std::string RemoteScales::byteArrayToHexString(const uint8_t* byteArray, size_t length) {
  std::string hexString;
  hexString.reserve(length * 3); // Reserve space for the resulting string

  char hex[4];
  for (size_t i = 0; i < length; i++) {
    snprintf(hex, sizeof(hex), "%02X ", byteArray[i]);
    hexString.append(hex);
  }

  return hexString;
}


// ---------------------------------------------------------------------------------------
// ------------------------   RemoteScales methods    ------------------------------
// ---------------------------------------------------------------------------------------

void RemoteScalesScanner::setReconnectTarget(const NimBLEAddress& address, const std::string& name) {
  reconnectAddress = address;
  reconnectName = name;
  hasReconnectTarget = true;
}

void RemoteScalesScanner::clearReconnectTarget() { hasReconnectTarget = false; }

void RemoteScalesScanner::initializeAsyncScan() {
  if (isRunning) {
    if (startedWithTarget == hasReconnectTarget) return;
    stopAsyncScan(); // the other mode: restart
  }
  cleanupDiscoveredScales();

  // Reconnect mode needs the target on the allow list before the scan starts (the host refuses to change the list
  // while scanning). If the list cannot be set, scan as discovery rather than not at all.
  bool reconnect = hasReconnectTarget;
  if (reconnect) {
    allowListed = NimBLEDevice::whiteListAdd(reconnectAddress);
    if (!allowListed) reconnect = false;
  }

  // We set the second parameter to true to prevent the library from storing BLEAdvertisedDevice objects
  // for devices we're not interested in. This is important because the library will otherwise run out of
  // memory after a while.
  NimBLEDevice::getScan()->setAdvertisedDeviceCallbacks(this, true);
  NimBLEDevice::getScan()->setInterval(500);
  NimBLEDevice::getScan()->setWindow(100);
  NimBLEDevice::getScan()->setMaxResults(0);
  NimBLEDevice::getScan()->setDuplicateFilter(false);
  // Reconnect: passive, allow-list filtered. A passive scan sends no scan request, so it neither waits for a scan
  // response nor is subject to the controller's scan backoff. Discovery: active, unfiltered.
  NimBLEDevice::getScan()->setFilterPolicy(reconnect ? BLE_HCI_SCAN_FILT_USE_WL : BLE_HCI_SCAN_FILT_NO_WL);
  NimBLEDevice::getScan()->setActiveScan(!reconnect);
  NimBLEDevice::getScan()->start(0, nullptr, false); // Set to 0 for continuous
  reconnectScan = reconnect;
  startedWithTarget = hasReconnectTarget;
  isRunning = true;
}

void RemoteScalesScanner::stopAsyncScan() {
  if (!isRunning) return;
  NimBLEDevice::getScan()->stop();
  NimBLEDevice::getScan()->clearResults();
  alreadySeenAddresses.cleanup();
  isRunning = false;
  leaveReconnectScan();
}

// The NimBLEScan is shared with other users of the stack that never set the filter policy: leave it as found
// (no allow list) and take the scanner's entry off the list.
void RemoteScalesScanner::leaveReconnectScan() {
  if (allowListed) {
    NimBLEDevice::whiteListRemove(reconnectAddress);
    allowListed = false;
  }
  if (reconnectScan) {
    NimBLEDevice::getScan()->setFilterPolicy(BLE_HCI_SCAN_FILT_NO_WL);
    reconnectScan = false;
  }
}

void RemoteScalesScanner::restartAsyncScan() {
  stopAsyncScan();
  initializeAsyncScan();
}

// Reconnect mode: only the target is listed, once, under the name from its advert or else the saved one (a passive
// scan sees no scan response, which is where many scales put their name).
void RemoteScalesScanner::onReconnectResult(NimBLEAdvertisedDevice* advertisedDevice) {
  if (advertisedDevice->getAddress() != reconnectAddress || !discoveredScales.empty()) return;
  std::string name = advertisedDevice->getName();
  DiscoveredDevice device(advertisedDevice, name.empty() ? reconnectName : name);
  if (RemoteScalesPluginRegistry::getInstance()->containsPluginForDevice(device)) {
    discoveredScales.push_back(device);
  }
}

void RemoteScalesScanner::onResult(NimBLEAdvertisedDevice* advertisedDevice) {
  if (reconnectScan) {
    onReconnectResult(advertisedDevice);
    return;
  }
  // Only dedupe addresses that have already matched a plugin. A device whose
  // local name arrives in a separate scan-response packet (common: many
  // peripherals omit the name from the primary ADV_IND to save space) would
  // otherwise fail containsPluginForDevice() on the first, nameless callback,
  // get cached as "seen" regardless, and then be silently ignored forever
  // once the scan response with the real name arrives.
  const uint8_t* addrBytes = advertisedDevice->getAddress().getNative();
  std::string addrStr(addrBytes, addrBytes + 6);
  if (RemoteScalesPluginRegistry::getInstance()->containsPluginForDevice(advertisedDevice)) {
    if (!alreadySeenAddresses.exists(addrStr)) {
      discoveredScales.emplace_back(advertisedDevice);
    }
  }
}

void RemoteScalesScanner::cleanupDiscoveredScales() {
  discoveredScales.clear();
}

bool RemoteScalesScanner::isScanRunning() const {
  return isRunning;
}



// ---------------------------------------------------------------------------------------
// ------------------------   RemoteScalesFactory methods    -----------------------------
// ---------------------------------------------------------------------------------------
RemoteScalesFactory* RemoteScalesFactory::instance = nullptr;

std::unique_ptr<RemoteScales> RemoteScalesFactory::create(DiscoveredDevice device) {
  if (!RemoteScalesPluginRegistry::getInstance()->containsPluginForDevice(device)) {
    return nullptr;
  }
  return RemoteScalesPluginRegistry::getInstance()->initialiseRemoteScales(device);
}
