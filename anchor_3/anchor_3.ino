/*
 * ESP32 DW1000 ANCHOR (kompatibel zu jremington's DW1000_library)
 * - Callback: attachNewRange(void (*)())  -> ohne Parameter
 * - Adresse:  startAsAnchor(char*, const byte*, bool) -> char* nötig
 * - Druckt gemessene Distanzen (und RX-Power, falls verfügbar)
 */

#include <Arduino.h>
#include <DW1000.h>
#include <DW1000Ranging.h>

// ---------- Pins ----------
const uint8_t PIN_SS  = 4;   // CS
const uint8_t PIN_RST = 27;  // RST
const uint8_t PIN_IRQ = 34;  // IRQ

// ---------- Adresse dieses Anchors (als String!) ----------
// A1:
char ANCHOR_ADDR[] = "83:00:00:EF:BE:01:CA:DE";
// Für A2/A3/A4 jeweils die letzte Gruppe ändern:
// A2: "DE:CA:01:BE:EF:00:00:82"
// A3: "DE:CA:01:BE:EF:00:00:83"
// A4: "DE:CA:01:BE:EF:00:00:84"

// ---------- Kalibrierter Antenna-Delay (aus Autokalibrierung übernehmen) ----------
uint16_t ANCHOR_ADELAY = 16570; // Beispielwert – bitte anpassen!

// ---------- Optionale Anzeige ----------
bool showRanges = true;
unsigned long lastPrint = 0;
const unsigned long PRINT_MS = 200;

// ---------- Callback: neue Distanz verfügbar (alte Signatur: ohne Parameter) ----------
void newRange() {
  if (!showRanges) return;
  if (millis() - lastPrint < PRINT_MS) return;

  DW1000Device* dev = DW1000Ranging.getDistantDevice(); // aktuelles Gegenüber
  if (!dev) return;

  lastPrint = millis();

  Serial.print(F("Tag 0x"));
  Serial.print(dev->getShortAddress(), HEX);
  Serial.print(F(" -> Range: "));
  Serial.print(dev->getRange(), 2);
  Serial.print(F(" m"));

  // RX-Power (falls in deiner Bibliothek vorhanden)
  // In jremingtons Lib ist getRXPower() verfügbar:
  Serial.print(F("  RXpower: "));
  Serial.print(dev->getRXPower(), 1);
  Serial.println(F(" dBm"));
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("# Anchor booting..."));

  // DW1000 initialisieren
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);
  DW1000.setAntennaDelay(ANCHOR_ADELAY);

  // Stabiler Funkmodus aus dem README: MODE_LONGDATA_RANGE_LOWPOWER
  DW1000Ranging.startAsAnchor(ANCHOR_ADDR,
                              DW1000.MODE_LONGDATA_RANGE_LOWPOWER,
                              false);

  // Callback registrieren (ohne Parameter)
  DW1000Ranging.attachNewRange(newRange);

  Serial.print(F("# Anchor started with address "));
  Serial.println(ANCHOR_ADDR);
}

void loop() {
  // Pflicht – Ranging-Library abarbeiten
  DW1000Ranging.loop();
}
