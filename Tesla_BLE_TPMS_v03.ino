#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <Preferences.h>

/*
   Tesla BLE TPMS v0.4 by Conny (c)2026 - Modified for continuous scanning
   ============================================================
   v0.4: scanning auto-starts on boot, plus a watchdog that restarts scan if it stops.
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
  raw.replace(":", "");
  raw.replace("-", "");
  if (raw.length() != 12) return "";
  String formatted = "";
  for (int i = 0; i < 12; i += 2) {
    if (i > 0) formatted += ":";
    formatted += raw.substring(i, i + 2);
  }
  formatted.toUpperCase();
  return formatted;
}

struct HelpDetail { const char* command; const char* content; };
const HelpDetail helpTable[] = {
  { "HELP",   "\n--- <help> shows the list of available commands." },
  { "LIST",   "\n--- <list> shows the list of MAC addresses in memory." },
  { "SCAN",   "\n--- HELP: SCAN <on/off/MAC_ADDRESS> ---\n  on/off -> Starts/stops scanning.\n  MAC_ADDR -> Listens only to specified MAC for 15s." },
  { "DEBUG",  "\n--- HELP: DEBUG <on/off> ---\n  on -> Show raw hex. off -> (Default) hide raw hex." },
  { "FILTER", "\n--- HELP: FILTER <on/off> ---\n  on -> (Default) Tesla only. off -> All BLE devices." },
  { "RESET",  "\n--- HELP: RESET <param> ---\n  none -> reboot. total -> reset count. list -> reset MACs. all -> reset everything." },
  { "UPTIME", "\n--- HELP: UPTIME ---\n  Shows time since boot." },
  { "ABOUT",  "\n--- HELP: ABOUT ---\n  Shows status and version." },
};

void printHelp(String subCmd = "") {
  subCmd.toUpperCase();
  if (subCmd == "") {
    Serial.println("Commands: help, list, scan <p>, debug <p>, filter <p>, reset <p>, uptime, about");
    Serial.println("Advanced: help <command>");
    return;
  }
  for (const auto& entry : helpTable) {
    if (subCmd == entry.command) { Serial.println(entry.content); return; }
  }
  Serial.printf("No detailed help for '%s'.\n", subCmd.c_str());
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
      Serial.printf("[BLE] MAC: %s | RSSI: %d | Data: ", currentMAC.c_str(), advertisedDevice.getRSSI());
      uint8_t* p = advertisedDevice.getPayload();
      for (int i = 0; i < advertisedDevice.getPayloadLength(); i++) Serial.printf("%02X ", p[i]);
      Serial.println("");
      return;
    }

    if (targetMAC != "" && currentMAC != targetMAC) return;

    uint8_t* payload   = advertisedDevice.getPayload();
    size_t   payloadLen = advertisedDevice.getPayloadLength();
    int      startIdx   = -1;
    for (int i = 0; i < (int)payloadLen - 1; i++) {
      if (payload[i] == 0x2B && payload[i + 1] == 0x02) { startIdx = i; break; }
    }

    if (startIdx != -1) {
      if (targetMAC != "") targetFound = true;
      totalReadings++;
      preferences.putUInt("count", totalReadings);

      bool known = false;
      for (int i = 1; i <= (int)uniqueSensors; i++) {
        if (preferences.getString(("m" + String(i)).c_str(), "") == currentMAC) { known = true; break; }
      }
      if (!known && targetMAC == "") {
        uniqueSensors++;
        preferences.putString(("m" + String(uniqueSensors)).c_str(), currentMAC);
        preferences.putUInt("uCount", uniqueSensors);
      }

      uint8_t type = payload[startIdx + 4];
      Serial.printf("### %s #%u ###\n", (targetMAC != "" ? "PROBE DATA" : "TPMS READ"), totalReadings);
      Serial.printf("%s\nMAC:  %s\nRSSI: %d dBm\n",
                    known ? "[ KNOWN DEVICE ]" : "[ NEW DEVICE ] !!!",
                    currentMAC.c_str(), advertisedDevice.getRSSI());

      if (type < 0x05) {
        Serial.println("@ SLEEP MODE [NO DATA]");
      } else {
        float psi = (((payload[startIdx + 6] << 8) | payload[startIdx + 5]) - PRESS_OFFSET) / PRESS_DIVISOR;
        int   tF  = payload[startIdx + 7] - 1;
        Serial.printf("PRES: %.1fpsi [%.2fb]\nTEMP: %dF [%.1fC]\nBATT: %dmV\n",
                      psi, psi * PSI_TO_BAR, tF, (tF - 32) * 5.0 / 9.0,
                      (payload[startIdx + 9] << 8) | payload[startIdx + 8]);
      }
      if (isDebugActive) {
        Serial.print("RAW DATA: ");
        for (int i = 0; i < (int)payloadLen; i++) Serial.printf("%02X ", payload[i]);
        Serial.println("");
      }
      Serial.println("####################\n");

      if (targetMAC != "") { targetMAC = ""; Serial.println("Target found. Resuming scan."); }
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
  pBLEScan->start(0, NULL, false);
  isScanningActive = true;
  lastScanRestart  = millis();
  Serial.println("Scanning: CONTINUOUS (v0.4 - no EN button needed)");
}

void processCommand(String fullCmd) {
  fullCmd.trim();
  int spaceIdx = fullCmd.indexOf(' ');
  String cmd   = (spaceIdx == -1) ? fullCmd : fullCmd.substring(0, spaceIdx);
  String param = (spaceIdx == -1) ? ""      : fullCmd.substring(spaceIdx + 1);
  cmd.toUpperCase();
  param.toUpperCase();

  if (cmd == "HELP") {
    printHelp(param);
  } else if (cmd == "FILTER") {
    if (param == "ON") { if (isFilterActive) Serial.println("Filter is ALREADY ON."); else { isFilterActive = true;  Serial.println("Filter ENABLED (Tesla only)."); } }
    else if (param == "OFF") { if (!isFilterActive) Serial.println("Filter is ALREADY OFF."); else { isFilterActive = false; Serial.println("Filter DISABLED (All BLE)."); } }
    else Serial.println("Error: Use 'filter on' or 'filter off'");
  } else if (cmd == "DEBUG") {
    if (param == "ON") { if (isDebugActive) Serial.println("Debug is ALREADY ON."); else { isDebugActive = true;  Serial.println("Debug ON."); } }
    else if (param == "OFF") { if (!isDebugActive) Serial.println("Debug is ALREADY OFF."); else { isDebugActive = false; Serial.println("Debug OFF."); } }
    else Serial.println("Error: Use 'debug on' or 'debug off'");
  } else if (cmd == "ABOUT") {
    Serial.println("\n--- DEVICE INFO ---");
    Serial.printf("# Tesla BLE TPMS Reader v%s by Conny (c)2026\n", VERSION);
    Serial.printf("# Tot readings: %u | Unique MACs: %u\n", totalReadings, uniqueSensors);
    Serial.printf("# STATUS: Scan: %s | Debug: %s | Filter: %s\n",
                  isScanningActive ? "ON" : "OFF", isDebugActive ? "ON" : "OFF", isFilterActive ? "ON" : "OFF");
    unsigned long sec = millis() / 1000;
    Serial.printf("# UPTIME: %02lu:%02lu:%02lu\n", sec / 3600, (sec % 3600) / 60, sec % 60);
  } else if (cmd == "SCAN") {
    if (param == "ON") { if (isScanningActive) Serial.println("Scan is ALREADY ON."); else { isScanningActive = true; startContinuousScan(); Serial.println("Scan RESUMED."); } }
    else if (param == "OFF") { if (!isScanningActive) Serial.println("Scan is ALREADY OFF."); else { isScanningActive = false; pBLEScan->stop(); Serial.println("Scan STOPPED."); } }
    else if (param.length() == 12) { targetMAC = formatMAC(param); targetFound = false; targetSearchStart = millis(); Serial.printf("Probing for MAC: %s...\n", targetMAC.c_str()); }
    else Serial.println("Error: Use 'scan on', 'scan off' or 'scan <MAC_ADDRESS>' (12 chars, no separators).");
  } else if (cmd == "LIST") {
    Serial.println("\n--- MAC DATABASE ---");
    for (int i = 1; i <= (int)uniqueSensors; i++) Serial.printf("#%d: %s\n", i, preferences.getString(("m" + String(i)).c_str(), "ERR").c_str());
  } else if (cmd == "UPTIME") {
    unsigned long sec = millis() / 1000;
    Serial.printf("Uptime: %02lu:%02lu:%02lu\n", sec / 3600, (sec % 3600) / 60, sec % 60);
  } else if (cmd == "RESET") {
    if (param == "LIST") { preferences.putUInt("uCount", 0); Serial.println("Cleared MAC LIST!"); }
    else if (param == "TOTAL") { preferences.putUInt("count", 0); Serial.println("Cleared TOTAL COUNTS!"); }
    else if (param == "ALL") { preferences.clear(); Serial.println("Cleared ALL memory!"); }
    Serial.println("Restarting in 500ms...");
    delay(500);
    esp_restart();
  } else if (cmd == "DSLEEP") {
    Serial.println("Going in DEEP SLEEP! Hard reset to restart!");
    delay(500);
    esp_deep_sleep_start();
  } else {
    Serial.printf("Unknown: '%s'! Type 'help'!\n", cmd.c_str());
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  preferences.begin("tpms-data", false);
  totalReadings = preferences.getUInt("count",  0);
  uniqueSensors = preferences.getUInt("uCount", 0);

  Serial.println("\n###########################");
  Serial.printf("@ Tesla BLE TPMS Reader v%s\n", VERSION);
  Serial.printf("@ Tot readings: %u\n", totalReadings);
  Serial.printf("@ Unique MACs:  %u\n", uniqueSensors);
  Serial.println("###########################");

  BLEDevice::init("");
  startContinuousScan();
}

void loop() {
  if (pBLEScan && !pBLEScan->isScanning()) {
    if (millis() - lastScanRestart > 10000) {
      Serial.println("[WATCHDOG] Scan stopped unexpectedly, restarting...");
      startContinuousScan();
    }
  }

  if (targetMAC != "" && (millis() - targetSearchStart > 15000)) {
    if (!targetFound) Serial.printf("\n[ERROR] Target %s not found.\n", targetMAC.c_str());
    targetMAC = "";
  }

  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      if (inputString.length() > 0) { processCommand(inputString); inputString = ""; }
    } else inputString += inChar;
  }
  yield();
}
