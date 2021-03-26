
/** SteamVR V1 HTC / SteamVR V2.0 Lighthouse/Base Station ESP32 Control:
 *
 *  Created: on March 25 2021
 *      Author: nullstalgia
 *
 * Based on NimBLE BLE Client Example:
 * https://github.com/h2zero/NimBLE-Arduino/blob/master/examples/NimBLE_Client/NimBLE_Client.ino
 *
 */

#define CONFIG_BT_NIMBLE_MAX_CONNECTIONS 5

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "JC_Button.h"

// For Version 1 (HTC) Base Stations:

// You can place up to CONFIG_BT_NIMBLE_MAX_CONNECTIONS amount of IDs here.
// (ESP32 max is 9) Find this on the back of your Base Station. Technically you
// only need to enter the B station ID, but C will look around for B for a while
// before shutting down, so I personally put both in :) This is required to turn
// the Base Station off immediately, and as such, this app is configured so if
// you want this to even turn it ON, you need the ID in here.
// My Base Stations for example: "7F35E5C5", "034996AB"
const char* lighthouseHTCIDs[] = {"7F35E5C5", "034996AB"};

// For Version 2.0 Base Stations:

// You can have the app just turn on/off every 2.0 Base Station it finds by
// turning this false If it's true, it will use the following filter to turn
// on/off Base Stations (in case you have multiple sets for some reason)
// Also no, this app is not set up for changing RF channels
const bool lighthouseV2Filtering = false;
// Enter the full MAC Address of your desired Base Stations below
// You can find this with NRF Connect or a similar app on your smartphone
// Example: lighthouseLHBMACs[] = {NimBLEAddress("D3:EA:E4:A4:58:DF")};

static NimBLEAddress lighthouseV2MACs[] = {};

// Connect an LED to this pin to get info on if there was an issue during the
// command (if an error does happen, just try it again a couple times) Just a
// couple slow-ish blinks: Success Many fast blinks: Error, try again
const uint8_t ledPin = 25;

Button offButton(32);  // define the pin for button (pull to ground to activate)

Button onButton(33);  // define the pin for button (pull to ground to activate)

// The remote service we wish to connect to.
// static NimBLEUUID serviceUUIDHTC("0000cb00-0000-1000-8000-00805f9b34fb");
// ^ V1 Service long UUID
static NimBLEUUID serviceUUIDHTC("CB00");
// The characteristic of the remote service we are interested in.
// static NimBLEUUID
// characteristicUUIDHTC("0000cb01-0000-1000-8000-00805f9b34fb"); ^ V1
// Characteristic long UUID
static NimBLEUUID characteristicUUIDHTC("CB01");

static NimBLEUUID serviceUUIDV2("00001523-1212-efde-1523-785feabcd124");
static NimBLEUUID characteristicUUIDV2("00001525-1212-efde-1523-785feabcd124");

enum { NOTHING = 0, TURN_ON_PERM = 1, TURN_OFF = 2 };

#define MAX_DISCOVERABLE_LH CONFIG_BT_NIMBLE_MAX_CONNECTIONS

static NimBLEAdvertisedDevice* discoveredLighthouses[MAX_DISCOVERABLE_LH];

uint8_t discoveredLighthouseVersions[MAX_DISCOVERABLE_LH];

uint8_t currentCommand = NOTHING;

int lighthouseCount = 0;

void scanEndedCB(NimBLEScanResults results);

static bool readyToConnect = false;
static uint32_t scanTime = 5; /** 0 = scan forever. In seconds */

class ClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* pClient) {
    Serial.println("Connected");
    /** After connection we should change the parameters if we don't need fast
     * response times. These settings are 150ms interval, 0 latency, 450ms
     * timout. Timeout should be a multiple of the interval, minimum is 100ms.
     *  I find a multiple of 3-5 * the interval works best for quick
     * response/reconnect. Min interval: 120 * 1.25ms = 150, Max interval: 120
     * * 1.25ms = 150, 0 latency, 60 * 10ms = 600ms timeout
     */
    pClient->updateConnParams(120, 120, 0, 60);
  };

  void onDisconnect(NimBLEClient* pClient) {
    Serial.print(pClient->getPeerAddress().toString().c_str());
    Serial.println(" - Disconnected");
    // NimBLEDevice::getScan()->start(scanTime, scanEndedCB);
  };

  /** Called when the peripheral requests a change to the connection parameters.
   *  Return true to accept and apply them or false to reject and keep
   *  the currently used parameters. Default will return true.
   */
  bool onConnParamsUpdateRequest(NimBLEClient* pClient,
                                 const ble_gap_upd_params* params) {
    if (params->itvl_min < 24) { /** 1.25ms units */
      return false;
    } else if (params->itvl_max > 40) { /** 1.25ms units */
      return false;
    } else if (params->latency > 2) { /** Number of intervals allowed to skip */
      return false;
    } else if (params->supervision_timeout > 100) { /** 10ms units */
      return false;
    }

    return true;
  };
};

/** Define a class to handle the callbacks when advertisments are received */
class AdvertisedDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
    Serial.println(__LINE__);
    Serial.print("Advertised Device found: ");
    Serial.println(advertisedDevice->toString().c_str());
    // V1 HTC Base Stations discovery
    if (advertisedDevice->isAdvertisingService(serviceUUIDHTC) &&
        advertisedDevice->haveName() &&
        advertisedDevice->getName().substr(0, 6) == "HTC BS") {
      char advertisedID[7];
      strcpy(advertisedID, advertisedDevice->getName().substr(7, 6).c_str());

      Serial.println("V1 Base Station Found!");
      Serial.print("Advertised ID: ");
      Serial.println(advertisedID);

      // Have we already discovered this lighthouse?
      for (uint8_t i = 0; i < MAX_DISCOVERABLE_LH; i++) {
        if (discoveredLighthouses[i] == nullptr)
          continue;
        if (discoveredLighthouses[i]->getAddress() ==
            advertisedDevice->getAddress()) {
          return;
        }
      }
      // Are we expecting this lighthouse?
      bool expected = false;

      for (uint8_t i = 0; i < sizeof(lighthouseHTCIDs) / sizeof(char*); i++) {
        std::string advertisedIDchomp =
            advertisedDevice->getName().substr(9, 4);
        std::string registeredIDchomp = std::string(lighthouseHTCIDs[i]);
        registeredIDchomp = registeredIDchomp.substr(4, 4);
        if (advertisedIDchomp == registeredIDchomp) {
          expected = true;
        }
      }

      if (!expected) {
        Serial.println("Ignoring this Base Station, ID doesn't match.");
        return;
      }
      Serial.print("Controllable devices: ");
      Serial.println(lighthouseCount + 1);
      discoveredLighthouses[lighthouseCount] = advertisedDevice;
      discoveredLighthouseVersions[lighthouseCount] = 1;

      lighthouseCount++;
      if (lighthouseCount >= MAX_DISCOVERABLE_LH ||
          lighthouseCount >= sizeof(lighthouseHTCIDs) / sizeof(char*))
        NimBLEDevice::getScan()->stop();
    } else
        // V2 Base Stations discovery
        if (advertisedDevice->isAdvertisingService(serviceUUIDV2) &&
            advertisedDevice->haveName() &&
            advertisedDevice->getName().substr(0, 3) == "LHB") {
      Serial.println("V2 Base Station Found!");
      Serial.print("MAC Address: ");
      Serial.println(advertisedDevice->getAddress().toString().c_str());

      // Have we already discovered this lighthouse?
      for (uint8_t i = 0; i < MAX_DISCOVERABLE_LH; i++) {
        if (discoveredLighthouses[i] == nullptr)
          continue;
        if (discoveredLighthouses[i]->getAddress() ==
            advertisedDevice->getAddress()) {
          return;
        }
      }

      if (lighthouseV2Filtering) {
        // Are we expecting this lighthouse?
        bool expected = false;

        for (NimBLEAddress address : lighthouseV2MACs) {
          if (address == advertisedDevice->getAddress()) {
            expected = true;
          }
        }

        if (!expected) {
          Serial.println("Ignoring this Base Station, MAC doesn't match.");
          return;
        }
      }

      Serial.print("Controllable devices: ");
      Serial.println(lighthouseCount + 1);
      discoveredLighthouses[lighthouseCount] = advertisedDevice;
      discoveredLighthouseVersions[lighthouseCount] = 2;

      lighthouseCount++;
      if (lighthouseCount >= MAX_DISCOVERABLE_LH ||
          lighthouseCount >= sizeof(lighthouseHTCIDs) / sizeof(char*))
        NimBLEDevice::getScan()->stop();
    }
    // Discovery Done
  };
};

/** Callback to process the results of the last scan or restart it */
void scanEndedCB(NimBLEScanResults results) {
  Serial.println("Scan Ended");
  if (lighthouseCount)
    readyToConnect = true;
}

/** Create a single global instance of the callback class to be used by all
 * clients */
static ClientCallbacks clientCB;

bool connectToLighthouse(NimBLEClient* pClient,
                         NimBLEAdvertisedDevice* advDevice) {
  bool connected = false;
  // Let's give it a few tries just in case, I've had it fail once but work
  // again right after.
  for (uint8_t i = 0; i < 3; i++) {
    delay(1500);
    Serial.print("Connection Attempt #");
    Serial.print(i+1);
    Serial.print(": ");
    connected = pClient->connect(advDevice, false);
    if (connected) {
      Serial.println("Success");
      break;
    } else {
      Serial.println("Fail");
    }
  }
  if (connected) {
    return true;
  }
  return false;
}

/** Handles the provisioning of clients and connects / interfaces with the
 * server */
bool sendLighthouseCommands() {
  NimBLEClient* pClient = nullptr;

  NimBLEAdvertisedDevice* advDevice = nullptr;

  bool fullSuccess = true;

  if (!lighthouseCount) {
    Serial.println("No lighthouses found to send commands to");
    return false;
  }

  for (uint8_t i = 0; i < lighthouseCount; i++) {
    // Serial.println(__LINE__);
    advDevice = discoveredLighthouses[i];

    /** Check if we have a client we should reuse first **/
    if (NimBLEDevice::getClientListSize()) {
      /** Special case when we already know this device, we send false as the
       *  second argument in connect() to prevent refreshing the service
       * database. This saves considerable time and power.
       */
      pClient = NimBLEDevice::getClientByPeerAddress(advDevice->getAddress());
      if (pClient) {
        if (connectToLighthouse(pClient, advDevice)) {
          Serial.println("Reconnected!");
        } else {
          Serial.println("Reconnect failed!");
          return false;
        }
      }
      /** We don't already have a client that knows this device,
       *  we will check for a client that is disconnected that we can use.
       */
      else {
        pClient = NimBLEDevice::getDisconnectedClient();
      }
    }

    /** No client to reuse? Create a new one. */
    if (!pClient) {
      if (NimBLEDevice::getClientListSize() >= NIMBLE_MAX_CONNECTIONS) {
        Serial.println("Max clients reached - no more connections available");
        return false;
      }

      pClient = NimBLEDevice::createClient();

      Serial.println("New client created");

      pClient->setClientCallbacks(&clientCB, false);
      /** Set initial connection parameters: These settings are 15ms interval, 0
       * latency, 120ms timout. These settings are safe for 3 clients to connect
       * reliably, can go faster if you have less connections. Timeout should be
       * a multiple of the interval, minimum is 100ms. Min interval: 12 * 1.25ms
       * = 15, Max interval: 12 * 1.25ms = 15, 0 latency, 51 * 10ms = 510ms
       * timeout
       */
      pClient->setConnectionParams(12, 12, 0, 51);
      /** Set how long we are willing to wait for the connection to complete
       * (seconds), default is 30. */
      pClient->setConnectTimeout(10);

      if (!connectToLighthouse(pClient, advDevice)) {
        /** Created a client but failed to connect, don't need to keep it as it
         * has no data */
        NimBLEDevice::deleteClient(pClient);
        Serial.println("Failed to connect, deleted client");
        return false;
      }
    }

    if (!pClient->isConnected()) {
      if (!connectToLighthouse(pClient, advDevice)) {
        Serial.println("Failed to connect");
        return false;
      }
    }

    Serial.print("Connected to: ");
    Serial.println(pClient->getPeerAddress().toString().c_str());
    Serial.print("RSSI: ");
    Serial.println(pClient->getRssi());

    /** Now we can read/write/subscribe the charateristics of the services we
     * are interested in */
    NimBLERemoteService* pSvc = nullptr;
    NimBLERemoteCharacteristic* pChr = nullptr;

    if (discoveredLighthouseVersions[i] == 1) {
      pSvc = pClient->getService(serviceUUIDHTC);
      if (pSvc) { /** make sure it's not null */
        pChr = pSvc->getCharacteristic(characteristicUUIDHTC);

        if (pChr) { /** make sure it's not null */
          if (pChr->canWrite()) {
            uint8_t array[20] = {0x12, 0xCC, 0x00, 0xFF, 0xDD, 0xCC, 0xBB,
                                 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
            if (currentCommand == TURN_OFF) {
              array[1] = 0x02;
              array[3] = 0x01;
              std::string fullID = std::string(lighthouseHTCIDs[i]);
              char buffer[5];
              strcpy(buffer, fullID.substr(0, 2).c_str());
              array[7] = strtoul(buffer, NULL, 16);
              strcpy(buffer, fullID.substr(2, 2).c_str());
              array[6] = strtoul(buffer, NULL, 16);
              strcpy(buffer, fullID.substr(4, 2).c_str());
              array[5] = strtoul(buffer, NULL, 16);
              strcpy(buffer, fullID.substr(6, 2).c_str());
              array[4] = strtoul(buffer, NULL, 16);
              Serial.println(" - Turning Off");
            } else if (currentCommand == TURN_ON_PERM) {
              array[1] = 0x00;
              array[3] = 0x00;
              array[7] = 0xFF;
              array[6] = 0xFF;
              array[5] = 0xFF;
              array[4] = 0xFF;
              /*
              std::string fullID = std::string(lighthouseHTCIDs[i]);
              char buffer[5];
              strcpy(buffer, fullID.substr(0, 2).c_str());
              array[7] = strtoul(buffer, NULL, 16);
              strcpy(buffer, fullID.substr(2, 2).c_str());
              array[6] = strtoul(buffer, NULL, 16);
              strcpy(buffer, fullID.substr(4, 2).c_str());
              array[5] = strtoul(buffer, NULL, 16);
              strcpy(buffer, fullID.substr(6, 2).c_str());
              array[4] = strtoul(buffer, NULL, 16);
              */
              Serial.println(" - Turning On");
            }
            if (pChr->writeValue(array, 20)) {
              Serial.println(" - Wrote command successfully");
            } else {
              Serial.println(" - Issue with writing command!!");
              fullSuccess = false;
            }
            // Serial.println(pChr->getHandle());
            // Serial.printf("%d 0x%.2x", pRemoteCharacteristic->getHandle(),
            // pRemoteCharacteristic->getHandle());
          }
        }
      } else {
        fullSuccess = false;
      }
    } else if (discoveredLighthouseVersions[i] == 2) {
      pSvc = pClient->getService(serviceUUIDV2);
      if (pSvc) { /** make sure it's not null */
        pChr = pSvc->getCharacteristic(characteristicUUIDV2);

        if (pChr) { /** make sure it's not null */
          if (pChr->canWrite()) {
            if (currentCommand == TURN_OFF) {
              Serial.println(" - Turning Off");
              if (pChr->writeValue(0)) {
                Serial.println(" - Wrote command successfully");
              } else {
                fullSuccess = false;
              }
            } else if (currentCommand == TURN_ON_PERM) {
              Serial.println(" - Turning On");
              if (pChr->writeValue(1)) {
                Serial.println(" - Wrote command successfully");
              } else {
                fullSuccess = false;
              }
            }
          }
        }
      } else {
        fullSuccess = false;
      }
    }
    Serial.println(" - Done with this device!");
    pClient->disconnect();
  }
  return fullSuccess;
}

// Runs once on power on
void setup() {
  Serial.begin(115200);
  Serial.println("Starting NimBLE Client");
  offButton.begin();
  onButton.begin();
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  Serial.println("Amount of registered V1 (HTC) Lighthouses: ");
  Serial.println(sizeof(lighthouseHTCIDs) / sizeof(char*));
  Serial.println("Amount of registered V2.0 Lighthouses: ");
  int v2count = 0;
  for (NimBLEAddress address : lighthouseV2MACs) {
    v2count++;
  }
  Serial.println(v2count);
  /** Initialize NimBLE, no device name spcified as we are not advertising */
  NimBLEDevice::init("");

  /** Optional: set the transmit power, default is 3db */
  NimBLEDevice::setPower(ESP_PWR_LVL_P9); /** +9db */

  /** create new scan */
  NimBLEScan* pScan = NimBLEDevice::getScan();

  /** create a callback that gets called when advertisers are found */
  pScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());

  /** Set scan interval (how often) and window (how long) in milliseconds */
  pScan->setInterval(45);
  pScan->setWindow(15);

  /** Active scan will gather scan response data from advertisers
   *  but will use more energy from both devices
   */
  pScan->setActiveScan(true);
  /** Start scanning for advertisers for the scan time specified (in seconds) 0
   * = forever Optional callback for when scanning stops.
   */
  // pScan->start(scanTime, scanEndedCB);
  delay(500);
  digitalWrite(ledPin, LOW);
}

// This is the Arduino main loop function.
void loop() {
  offButton.read();
  onButton.read();

  if (offButton.wasPressed()) {
    digitalWrite(ledPin, HIGH);
    lighthouseCount = 0;
    for (uint8_t i = 0; i < MAX_DISCOVERABLE_LH; i++) {
      discoveredLighthouses[i] = nullptr;
      discoveredLighthouseVersions[i] = 0;
    }
    NimBLEDevice::getScan()->start(scanTime, scanEndedCB);
    currentCommand = TURN_OFF;
  } else if (onButton.wasPressed()) {
    digitalWrite(ledPin, HIGH);
    lighthouseCount = 0;
    for (uint8_t i = 0; i < MAX_DISCOVERABLE_LH; i++) {
      discoveredLighthouses[i] = nullptr;
      discoveredLighthouseVersions[i] = 0;
    }
    NimBLEDevice::getScan()->start(scanTime, scanEndedCB);
    currentCommand = TURN_ON_PERM;
  }

  if (currentCommand != NOTHING && readyToConnect) {
    Serial.println(__LINE__);
    if (sendLighthouseCommands()) {
      Serial.println("We should have sent the commands");
      for (uint8_t i = 0; i < 3; i++) {
        digitalWrite(ledPin, !digitalRead(ledPin));
        delay(200);
      }
    } else {
      Serial.println(
          "We have failed to connect to a server that should have been there; "
          "there is nothing more we "
          "will do this time.");
      for (uint8_t i = 0; i < 15; i++) {
        digitalWrite(ledPin, !digitalRead(ledPin));
        delay(100);
      }
    }
    digitalWrite(ledPin, LOW);
    readyToConnect = false;
    currentCommand = NOTHING;
  }
}