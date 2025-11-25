#include <SPI.h>
#include "DW1000Ranging.h"

#define VIBRO_PIN 12

// Raumgröße
const float ROOM_SIZE = 4.0;

const float SAFE_MIN_X = 0.5;
const float SAFE_MAX_X = 3.5;
const float SAFE_MIN_Y = 0.5;
const float SAFE_MAX_Y = 3.5;

// Optimierte Parameter - BESTE KOMBINATION
const float TAG_HEIGHT = 1.75;              
const float FILTER_ALPHA = 0.60;            // Kleiner = glatter (20%)
const float HYST = 0.10;                    // 20 cm Hysterese
const unsigned long ALARM_ON_DELAY = 10;   // 500ms draußen → Alarm AN
const unsigned long ALARM_OFF_DELAY = 10;  // 300ms drinnen → Alarm AUS
const unsigned long SIGNAL_TIMEOUT = 3000;  

// Qualitätsprüfung
const float MIN_DIST = 0.1;                 
const float MAX_DIST = 20.0;                
const float MAX_JUMP = 1.5;                 // Max 1.5m Positions-Sprung

// Messdaten
float dist2D[4] = { 0, 0, 0, 0 };
bool hasData[4] = { false, false, false, false };

// Status mit Zeitverwaltung
bool vibrationActive = false;
unsigned long lastUpdate = 0;

// State Machine für verzögerte Reaktion
bool potentialOutside = false;
bool potentialInside = false;
unsigned long timeFirstOutside = 0;
unsigned long timeFirstInside = 0;

// Gefilterte Position
float xFilt = ROOM_SIZE / 2.0;
float yFilt = ROOM_SIZE / 2.0;
bool posValid = false;  // Flag: Haben wir schon eine echte Position?

void setup() {
  Serial.begin(115200);
  delay(3000);

  SPI.begin(18, 19, 23);
  DW1000Ranging.initCommunication(27, 4, 34);
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);

  pinMode(VIBRO_PIN, OUTPUT);
  digitalWrite(VIBRO_PIN, LOW);

  Serial.println("╔════════════════════════════════════════════════════╗");
  Serial.println("║      ULTRA-STABILER UWB POSITIONS-TRACKER         ║");
  Serial.println("╚════════════════════════════════════════════════════╝");
  Serial.println("Glättung: " + String(FILTER_ALPHA, 2));
  Serial.println("Hysterese: " + String(HYST * 100, 0) + " cm");
  Serial.println("Alarm AN Verzögerung: " + String(ALARM_ON_DELAY) + " ms");
  Serial.println("Alarm AUS Verzögerung: " + String(ALARM_OFF_DELAY) + " ms");
  Serial.println("Max Sprung: " + String(MAX_JUMP, 1) + " m");
  Serial.println();
}

void loop() {
  DW1000Ranging.loop();

  // Timeout → alles zurücksetzen
  if (millis() - lastUpdate > SIGNAL_TIMEOUT) {
    if (vibrationActive) {
      digitalWrite(VIBRO_PIN, LOW);
      vibrationActive = false;
      Serial.println("⏱️  Signal-Timeout – System zurückgesetzt");
    }
    potentialOutside = false;
    potentialInside = false;
    posValid = false;  // Position muss neu initialisiert werden
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
    // Qualitätsprüfung der 3D-Distanz
    if (range3D < MIN_DIST || range3D > MAX_DIST) {
      Serial.print("⚠️  A");
      Serial.print(idx + 1);
      Serial.println(": Ungültige 3D-Distanz ignoriert");
      return;
    }

    // Verbesserte 3D → 2D Projektion
    float range2D;
    if (range3D > TAG_HEIGHT) {
      range2D = sqrt(range3D * range3D - TAG_HEIGHT * TAG_HEIGHT);
    } else {
      range2D = 0.1;  // Wenn Tag tiefer als erwartet
    }

    // WICHTIG: Initialisierung beim ersten Wert (verhindert Sprünge)
    if (!hasData[idx]) {
      dist2D[idx] = range2D;  // Sofort übernehmen
      hasData[idx] = true;
    } else {
      // Exponentielle Glättung der Distanzen
      dist2D[idx] = (1.0 - FILTER_ALPHA) * dist2D[idx] + FILTER_ALPHA * range2D;
    }

    lastUpdate = millis();
    checkQuadrat();

    // Kompakte Ausgabe
    Serial.print("A");
    Serial.print(idx + 1);
    Serial.print(":");
    Serial.print(range2D, 1);
    Serial.println("m ");
  }
}

void checkQuadrat() {
  int validAnchors = 0;
  for (int i = 0; i < 4; i++)
    if (hasData[i]) validAnchors++;
  if (validAnchors < 3) return;

  // Anchor-Positionen
  const float ax[4] = { 0, 4, 4, 0 };
  const float ay[4] = { 0, 0, 4, 4 };

  float xSum = 0, ySum = 0;
  int combos = 0;

  auto trilaterate = [&](int a, int b, int c) -> bool {
    if (!hasData[a] || !hasData[b] || !hasData[c]) return false;

    float A = 2 * (ax[b] - ax[a]);
    float B = 2 * (ay[b] - ay[a]);
    float C = dist2D[a] * dist2D[a] - dist2D[b] * dist2D[b]
              - ax[a] * ax[a] + ax[b] * ax[b] - ay[a] * ay[a] + ay[b] * ay[b];

    float D = 2 * (ax[c] - ax[a]);
    float E = 2 * (ay[c] - ay[a]);
    float F = dist2D[a] * dist2D[a] - dist2D[c] * dist2D[c]
              - ax[a] * ax[a] + ax[c] * ax[c] - ay[a] * ay[a] + ay[c] * ay[c];

    float denom = (A * E - B * D);
    if (fabs(denom) < 0.01) return false;

    float x = (C * E - B * F) / denom;
    float y = (A * F - C * D) / denom;

    // Plausibilitäts-Check: Unmögliche Positionen ignorieren
    if (x < -5 || x > 10 || y < -5 || y > 10) return false;

    xSum += x;
    ySum += y;
    combos++;
    return true;
  };

  trilaterate(0, 1, 2);
  trilaterate(0, 1, 3);
  trilaterate(0, 2, 3);
  trilaterate(1, 2, 3);

  if (combos == 0) return;

  float xRaw = xSum / combos;
  float yRaw = ySum / combos;

  // Ausreißer-Erkennung: NUR wenn wir schon eine valide Position haben!
  if (posValid) {
    float dx = xRaw - xFilt;
    float dy = yRaw - yFilt;
    float jump = sqrt(dx * dx + dy * dy);

    if (jump > MAX_JUMP) {
      Serial.println();
      Serial.print("⚠️  Positions-Ausreißer ignoriert (Sprung: ");
      Serial.print(jump, 2);
      Serial.println("m)");
      return;
    }
  } else {
    // Erste Position → direkt übernehmen ohne Sprung-Check
    Serial.println();
    Serial.print("✓ Erste Position initialisiert: (");
    Serial.print(xRaw, 2);
    Serial.print(", ");
    Serial.print(yRaw, 2);
    Serial.println(")");
    xFilt = xRaw;
    yFilt = yRaw;
    posValid = true;
    return;  // Diese Runde keine weitere Verarbeitung
  }

  // Glättung der Position
  xFilt = (1.0 - FILTER_ALPHA) * xFilt + FILTER_ALPHA * xRaw;
  yFilt = (1.0 - FILTER_ALPHA) * yFilt + FILTER_ALPHA * yRaw;

  // --- STATE MACHINE MIT DOPPELTER VERZÖGERUNG ---

  bool currentlyOutside = false;

  if (vibrationActive) {
    // Bereits Alarm → muss DEUTLICH drinnen sein (mit Hysterese)
    if (xFilt < SAFE_MIN_X + HYST || xFilt > SAFE_MAX_X - HYST ||
        yFilt < SAFE_MIN_Y + HYST || yFilt > SAFE_MAX_Y - HYST) {
      currentlyOutside = true;
    }
  } else {
    // Kein Alarm → muss DEUTLICH draußen sein
    if (xFilt < SAFE_MIN_X - HYST || xFilt > SAFE_MAX_X + HYST ||
        yFilt < SAFE_MIN_Y - HYST || yFilt > SAFE_MAX_Y + HYST) {
      currentlyOutside = true;
    }
  }

  // ALARM AKTIVIEREN mit Verzögerung
  if (currentlyOutside && !vibrationActive) {
    potentialInside = false;  // Reset "Drinnen"-Timer

    if (!potentialOutside) {
      // Erster Moment draußen → Timer starten
      potentialOutside = true;
      timeFirstOutside = millis();
      Serial.print(" [OUT-Timer Start] ");
    } else {
      // Immer noch draußen → prüfen wie lange
      unsigned long elapsed = millis() - timeFirstOutside;
      if (elapsed >= ALARM_ON_DELAY) {
        vibrationActive = true;
        digitalWrite(VIBRO_PIN, HIGH);
        Serial.println();
        Serial.println("╔══════════════════════════════════════╗");
        Serial.print("║  🔴 ALARM AN (");
        Serial.print(xFilt, 2);
        Serial.print(", ");
        Serial.print(yFilt, 2);
        Serial.println(")      ║");
        Serial.println("╚══════════════════════════════════════╝");
      } else {
        Serial.print(" [OUT:");
        Serial.print(elapsed);
        Serial.print("ms] ");
      }
    }
  }
  // ALARM DEAKTIVIEREN mit Verzögerung
  else if (!currentlyOutside && vibrationActive) {
    potentialOutside = false;  // Reset "Draußen"-Timer

    if (!potentialInside) {
      // Erster Moment drinnen → Timer starten
      potentialInside = true;
      timeFirstInside = millis();
      Serial.print(" [IN-Timer Start] ");
    } else {
      // Immer noch drinnen → prüfen wie lange
      unsigned long elapsed = millis() - timeFirstInside;
      if (elapsed >= ALARM_OFF_DELAY) {
        vibrationActive = false;
        digitalWrite(VIBRO_PIN, LOW);
        Serial.println();
        Serial.println("╔══════════════════════════════════════╗");
        Serial.print("║  🟢 ALARM AUS (");
        Serial.print(xFilt, 2);
        Serial.print(", ");
        Serial.print(yFilt, 2);
        Serial.println(")     ║");
        Serial.println("╚══════════════════════════════════════╝");
      } else {
        Serial.print(" [IN:");
        Serial.print(elapsed);
        Serial.print("ms] ");
      }
    }
  }
  // Stabil drinnen/draußen → Timer zurücksetzen
  else {
    if (!currentlyOutside) potentialOutside = false;
    if (currentlyOutside) potentialInside = false;
  }

  // Status-Ausgabe
  Serial.print(" | (");
  Serial.print(xFilt, 2);
  Serial.print(",");
  Serial.print(yFilt, 2);
  Serial.println(")");
}
