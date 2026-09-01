#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <Preferences.h>

/*
   Tesla BLE TPMS v0.4 - Continuous Scanning
   Auto-starts on boot, watchdog restarts scan every 10s.
*/

const char* VERSION = "0.4";

bool isScanningActive = true;
bool isDebugActive   = false;
bool isFilterActive  = true;

const float PRESS_OFFSET  = 100.0;
const float PRESS_DIVISOR = 7.0;
const float PSI_TO_BAR     = 0.0689476;

Preferences preferences;
unsigned int totalReadings = 0;
unsigned int uniqueSensors = 0;
String inputString = "";
BLEScan* pBLEScan = NULL;
String  targetMAC          = "";
bool    targetFound        = false;
unsigned long targetSearchStart = 0;
unsigned long lastScanRestart = 0;

void startContinuousScan();
void processCommand(String fullCmd);

String formatMAC(String raw) {
  raw.replace(":", ""); raw.replace("-", "");
  if (raw.length() != 12) return "";
  String formatted = "";
  for (int i = 0; i < 12; i += 2) {
    if (i > 0) formatted += ":";
    formatted += raw.substring(i, i + 2);
  }
  formatted.toUpperCase();
  return formatted;
}

class MyCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (!isScanningActive && targetMAC == "") return;
    String name = advertisedDevice.getName().c_str();
    String currentMAC = advertisedDevice.getAddress().toString().c_str();
    currentMAC.toUpperCase();
    bool isTesla = (name.indexOf("tsTPMS") != -1);
    if (isFilterActive && !isTesla) return;
    if (!isFilterActive && !isTesla) {
      Serial.printf("[BLE] MAC: %s | RSSI: %d\n", currentMAC.c_str(), advertisedDevice.getRSSI());
      return;
    }
    if (targetMAC != "" && currentMAC != targetMAC) return;
    uint8_t* payload = advertisedDevice.getPayload();
    size_t payloadLen = advertisedDevice.getPayloadLength();
    int startIdx = -1;
    for (int i = 0; i < (int)payloadLen - 1; i++) {
      if (payload[i] == 0x2B && payload[i+1] == 0x02) { startIdx = i; break; }
    }
    if (startIdx != -1) {
      if (targetMAC != "") targetFound = true;
      totalReadings++;
      preferences.putUInt("count", totalReadings);
      bool known = false;
      for (int i = 1; i <= (int)uniqueSensors; i++) {
        if (preferences.getString(("m"+String(i)).c_str(), "") == currentMAC) { known = true; break; }
      }
      if (!known && targetMAC == "") {
        uniqueSensors++;
        preferences.putString(("m"+String(uniqueSensors)).c_str(), currentMAC);
        preferences.putUInt("uCount", uniqueSensors);
      }
      uint8_t type = payload[startIdx + 4];
      Serial.printf("### %s #%u ###\n", (targetMAC != "" ? "PROBE DATA" : "TPMS READ"), totalReadings);
      Serial.printf("%s\nMAC: %s\nRSSI: %d dBm\n", known ? "[KNOWN]" : "[NEW]", currentMAC.c_str(), advertisedDevice.getRSSI());
      if (type < 0x05) {
        Serial.println("@ SLEEP MODE [NO DATA]");
      } else {
        float psi = (((payload[startIdx+6]<<8)|payload[startIdx+5]) - PRESS_OFFSET) / PRESS_DIVISOR;
        int tF = payload[startIdx+7] - 1;
        Serial.printf("PRES: %.1fpsi [%.2fb]\nTEMP: %dF [%.1fC]\nBATT: %dmV\n", psi, psi*PSI_TO_BAR, tF, (tF-32)*5.0/9.0, (payload[startIdx+9]<<8)|payload[startIdx+8]);
      }
      if (isDebugActive) {
        Serial.print("RAW: ");
        for (int i = 0; i < (int)payloadLen; i++) Serial.printf("%02X ", payload[i]);
        Serial.println("");
      }
      Serial.println("####################\n");
      if (targetMAC != "") { targetMAC = ""; Serial.println("Target found."); }
    }
  }
};

void startContinuousScan() {
  if (pBLEScan == NULL) {
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyCallbacks(), false);
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(150);
    pBLEScan->setWindow(120);
  }
  pBLEScan->stop();
  delay(100);
  pBLEScan->start(5, NULL, false);
  isScanningActive = true;
  lastScanRestart = millis();
  Serial.println("Scanning: CONTINUOUS (v0.4 - auto restart every 10s)");
}

void processCommand(String fullCmd) {
  fullCmd.trim();
  int spaceIdx = fullCmd.indexOf(' ');
  String cmd = (spaceIdx == -1) ? fullCmd : fullCmd.substring(0, spaceIdx);
  String param = (spaceIdx == -1) ? "" : fullCmd.substring(spaceIdx+1);
  cmd.toUpperCase(); param.toUpperCase();
  if (cmd == "HELP") {
    Serial.println("Commands: help, list, scan <on/off/MAC>, debug <on/off>, filter <on/off>, reset <param>, uptime, about");
  } else if (cmd == "FILTER") {
    if (param == "ON") { isFilterActive = true; Serial.println("Filter ON (Tesla only)"); }
    else if (param == "OFF") { isFilterActive = false; Serial.println("Filter OFF (all BLE)"); }
  } else if (cmd == "DEBUG") {
    if (param == "ON") { isDebugActive = true; Serial.println("Debug ON"); }
    else if (param == "OFF") { isDebugActive = false; Serial.println("Debug OFF"); }
  } else if (cmd == "ABOUT") {
    Serial.printf("Tesla TPMS v%s | Reads: %u | MACs: %u\n", VERSION, totalReadings, uniqueSensors);
  } else if (cmd == "SCAN") {
    if (param == "ON") { startContinuousScan(); }
    else if (param == "OFF") { isScanningActive = false; pBLEScan->stop(); Serial.println("Scan OFF"); }
    else if (param.length() == 12) { targetMAC = formatMAC(param); targetFound = false; targetSearchStart = millis(); Serial.printf("Probing: %s\n", targetMAC.c_str()); }
    else Serial.println("Error: scan on / scan off / scan <12-char MAC>");
  } else if (cmd == "LIST") {
    for (int i = 1; i <= (int)uniqueSensors; i++) Serial.printf("#%d: %s\n", i, preferences.getString(("m"+String(i)).c_str(),"ERR").c_str());
  } else if (cmd == "UPTIME") {
    unsigned long sec = millis()/1000;
    Serial.printf("%02lu:%02lu:%02lu\n", sec/3600, (sec%3600)/60, sec%60);
  } else if (cmd == "RESET") {
    if (param == "LIST") { preferences.putUInt("uCount",0); Serial.println("MAC list cleared"); }
    else if (param == "TOTAL") { preferences.putUInt("count",0); Serial.println("Count cleared"); }
    else if (param == "ALL") { preferences.clear(); Serial.println("All cleared"); }
    delay(500); esp_restart();
  } else if (cmd == "DSLEEP") {
    Serial.println("Deep sleep..."); delay(500); esp_deep_sleep_start();
  } else {
    Serial.printf("Unknown: '%s'\n", cmd.c_str());
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  preferences.begin("tpms-data", false);
  totalReadings = preferences.getUInt("count", 0);
  uniqueSensors = preferences.getUInt("uCount", 0);
  Serial.printf("Tesla TPMS v%s | Reads: %u | MACs: %u\n", VERSION, totalReadings, uniqueSensors);
  BLEDevice::init("");
  startContinuousScan();
}

void loop() {
  if (millis() - lastScanRestart > 10000) {
    startContinuousScan();
  }
  if (targetMAC != "" && (millis() - targetSearchStart > 15000)) {
    if (!targetFound) Serial.printf("Target %s not found.\n", targetMAC.c_str());
    targetMAC = "";
  }
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputString.length() > 0) { processCommand(inputString); inputString = ""; }
    } else inputString += c;
  }
  yield();
}
