#include <SPI.h>
#include "DW1000Ranging.h"

#define VIBRO_PIN 12

// Raumgröße
const float ROOM_SIZE = 3.0;     // 3m x 3m Fläche

const float SAFE_MIN_X = 0.0;
const float SAFE_MAX_X = 3.0;
const float SAFE_MIN_Y = 0.0;
const float SAFE_MAX_Y = 3.0;

// Parameter
const float TAG_HEIGHT = 1.75;   // 1.75m über Boden
const float FILTER_ALPHA = 0.50;  // Etwas schneller reagierend
const float HYST = 0.05;         // 5 cm Hysterese gegen Flattern
const unsigned long SIGNAL_TIMEOUT = 2000; // 2 s ohne Signal → Alarm aus

// Messdaten
float dist2D[4] = {0, 0, 0, 0};
bool hasData[4] = {false, false, false, false};

// Status
bool vibrationActive = false;
unsigned long lastUpdate = 0;

// Gefilterte Position
float xFilt = ROOM_SIZE / 2.0;
float yFilt = ROOM_SIZE / 2.0;

void setup() {
  Serial.begin(115200);
  delay(3000);
  
  SPI.begin(18, 19, 23);
  DW1000Ranging.initCommunication(27, 4, 34); // RST, SS, IRQ
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
  
  pinMode(VIBRO_PIN, OUTPUT);
  digitalWrite(VIBRO_PIN, LOW);
  
  Serial.println("QUADRATISCHE 5x5m RAUMERKENNUNG MIT GLÄTTUNG");
  Serial.println("Glättung α=" + String(FILTER_ALPHA,2) + ", Hysterese=" + String(HYST*100,0) + " cm");
}

void loop() {
  DW1000Ranging.loop();

  // Timeout → Alarm aus
  if (millis() - lastUpdate > SIGNAL_TIMEOUT && vibrationActive) {
    digitalWrite(VIBRO_PIN, LOW);
    vibrationActive = false;
    Serial.println("Kein Signal – Alarm deaktiviert");
  }
}

void newRange() {
  DW1000Device* device = DW1000Ranging.getDistantDevice();
  if (!device) return;

  uint16_t addr = device->getShortAddress();
  float range3D = device->getRange();
  int idx = -1;

  if (addr == 0x81) idx = 0;
  else if (addr == 0x82) idx = 1;
  else if (addr == 0x83) idx = 2;
  else if (addr == 0x84) idx = 3;

  if (idx >= 0) {
    // 3D → 2D Umrechnung
    float range2D = sqrt(range3D * range3D - TAG_HEIGHT * TAG_HEIGHT);
    if (isnan(range2D) || range2D <= 0) range2D = range3D;

    // Exponentielle Glättung
    dist2D[idx] = (1.0 - FILTER_ALPHA) * dist2D[idx] + FILTER_ALPHA * range2D;

    hasData[idx] = true;
    lastUpdate = millis();

    checkQuadrat();
    
    // Kurze Ausgabe
    Serial.print("A");
    Serial.print(idx + 1);
    Serial.print(":");
    Serial.print(range2D, 1);
    Serial.println("m ");
  }
}

void checkQuadrat() {
  int validAnchors = 0;
  for (int i = 0; i < 4; i++) if (hasData[i]) validAnchors++;
  if (validAnchors < 3) return;

  // Anchor-Positionen (Quadrat 3x3m)
  const float ax[4] = {0, 3, 0, 3};  // const für Performance
  const float ay[4] = {0, 0, 3, 3};

  // ---- Position berechnen aus 3 Anchors ----
  float xSum = 0, ySum = 0;
  int combos = 0;

  auto trilaterate = [&](int a, int b, int c) -> bool {
    if (!hasData[a] || !hasData[b] || !hasData[c]) return false;
    
    float A = 2*(ax[b]-ax[a]);
    float B = 2*(ay[b]-ay[a]);
    float C = dist2D[a]*dist2D[a] - dist2D[b]*dist2D[b] 
              - ax[a]*ax[a] + ax[b]*ax[b] - ay[a]*ay[a] + ay[b]*ay[b];

    float D = 2*(ax[c]-ax[a]);
    float E = 2*(ay[c]-ay[a]);
    float F = dist2D[a]*dist2D[a] - dist2D[c]*dist2D[c] 
              - ax[a]*ax[a] + ax[c]*ax[c] - ay[a]*ay[a] + ay[c]*ay[c];

    float denom = (A*E - B*D);
    if (fabs(denom) < 0.0001) return false;

    float x = (C*E - B*F) / denom;
    float y = (A*F - C*D) / denom;
    
    xSum += x; ySum += y; combos++;
    return true;
  };

  // Alle möglichen 3er-Kombinationen
  trilaterate(0,1,2);
  trilaterate(0,1,3); 
  trilaterate(0,2,3);
  trilaterate(1,2,3);

  if (combos == 0) return;
  
  float x = xSum / combos;
  float y = ySum / combos;

  x = constrain(x, -1.0, ROOM_SIZE + 1.0);
  y = constrain(y, -1.0, ROOM_SIZE + 1.0);

  // Glättung der Position
  xFilt = (1.0 - FILTER_ALPHA) * xFilt + FILTER_ALPHA * x;
  yFilt = (1.0 - FILTER_ALPHA) * yFilt + FILTER_ALPHA * y;

  // HYSTERESE für stabiles Verhalten
  bool outside = false;
  
  if (vibrationActive) {
    // Wenn Alarm aktiv: erst bei SAFE_MIN_X + HYST zurück ins Haus
    outside = (xFilt < SAFE_MIN_X + HYST || xFilt > SAFE_MAX_X - HYST || 
               yFilt < SAFE_MIN_Y + HYST || yFilt > SAFE_MAX_Y - HYST);
  } else {
    // Wenn kein Alarm: schon bei SAFE_MIN_X - HYST Alarm auslösen
    outside = (xFilt < SAFE_MIN_X - HYST || xFilt > SAFE_MAX_X + HYST || 
               yFilt < SAFE_MIN_Y - HYST || yFilt > SAFE_MAX_Y + HYST);
  }

  if (outside && !vibrationActive) {
    digitalWrite(VIBRO_PIN, HIGH);
    vibrationActive = true;
    Serial.println();
    Serial.print("🔴 AUSSERHALB! (");
    Serial.print(xFilt, 2);
    Serial.print(", ");
    Serial.print(yFilt, 2);
    Serial.println(")");
  } 
  else if (!outside && vibrationActive) {
    digitalWrite(VIBRO_PIN, LOW);
    vibrationActive = false;
    Serial.println();
    Serial.print("🟢 DRINNEN (");
    Serial.print(xFilt, 2);
    Serial.print(", ");
    Serial.print(yFilt, 2);
    Serial.println(")");
  }
}