#include <Arduino.h>
#include <RadioLib.h>
#include <math.h>

#define LORA_NSS    8
#define LORA_DIO1   14
#define LORA_RST    12
#define LORA_BUSY   13
#define LORA_SCK    9
#define LORA_MISO   11
#define LORA_MOSI   10

#define LORA_FREQ_S1       915.0
#define LORA_FREQ_S2       916.0
#define LORA_BW            125.0
#define LORA_SF            7
#define LORA_CR            5
#define LORA_SYNC_WORD     0x12
#define LORA_POWER         2
#define LORA_PREAMBLE      8

// ── Frequency hopping ────────────────────────────────────────────
#define HOP_INTERVAL_MS    150

// ── Variance detection ───────────────────────────────────────────
#define VAR_WINDOW         8
#define STDDEV_TRIP        1.5f
#define TRIP_COUNT         2
#define NUM_SENDERS        2

// ── Person count classification ──────────────────────────────────
// Combined stddev (S1 + S2) thresholds from observed data:
//   1 person:  ~6.0 combined (2.3 + 3.8)
//   2 people:  ~9.3 combined (3.4 + 5.9)
// Threshold set between them. Tune based on your environment.
#define COMBINED_STDDEV_2P 7.5f

// Both links must trigger within this window to count as simultaneous
#define SIMUL_WINDOW_MS    2000

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

struct SenderState {
  char     id[4];
  float    freq;
  float    rssiWindow[VAR_WINDOW];
  int      winIdx;
  int      winFill;
  int      tripCount;
  bool     tripped;
  float    latestStdDev;         // Most recent stddev
  float    peakStdDev;           // Peak stddev during current event
  unsigned long tripStartTime;   // When this link first tripped
  uint32_t lastSeq;
  bool     seqInit;
  uint32_t totalPkts;
};

SenderState senders[NUM_SENDERS];

volatile bool rxFlag = false;
void IRAM_ATTR onRxDone() { rxFlag = true; }

int currentSlot = 0;
unsigned long lastHop = 0;

// ── Event classification state ───────────────────────────────────
bool eventActive = false;
unsigned long eventStartTime = 0;
float eventPeakCombined = 0;
int eventPeakCount = 0;          // 1 or 2
bool eventClassified = false;
int firstTrippedIdx = -1;        // Which link triggered first
bool directionReported = false;  // Already reported movement direction

int senderIndex(const char *id) {
  for (int i = 0; i < NUM_SENDERS; i++) {
    if (strcmp(senders[i].id, id) == 0) return i;
  }
  return -1;
}

float computeStdDev(SenderState &s) {
  if (s.winFill < 3) return 0.0f;
  int n = (s.winFill < VAR_WINDOW) ? s.winFill : VAR_WINDOW;

  float sum = 0;
  for (int i = 0; i < n; i++) sum += s.rssiWindow[i];
  float mean = sum / n;

  float varSum = 0;
  for (int i = 0; i < n; i++) {
    float diff = s.rssiWindow[i] - mean;
    varSum += diff * diff;
  }
  return sqrtf(varSum / n);
}

void switchFreq(int slot) {
  radio.setFrequency(senders[slot].freq);
  radio.startReceive();
  currentSlot = slot;
}

void classifyEvent() {
  int linksTripped = 0;
  float combinedStdDev = 0;
  bool bothSimultaneous = false;

  for (int i = 0; i < NUM_SENDERS; i++) {
    if (senders[i].tripped) {
      linksTripped++;
      combinedStdDev += senders[i].peakStdDev;
    }
  }

  // Check if both links tripped within the time window
  if (senders[0].tripped && senders[1].tripped) {
    unsigned long timeDiff;
    if (senders[0].tripStartTime > senders[1].tripStartTime)
      timeDiff = senders[0].tripStartTime - senders[1].tripStartTime;
    else
      timeDiff = senders[1].tripStartTime - senders[0].tripStartTime;

    bothSimultaneous = (timeDiff < SIMUL_WINDOW_MS);

    // Direction: which link triggered first → which triggered second
    if (!directionReported) {
      int secondIdx = (firstTrippedIdx == 0) ? 1 : 0;
      Serial.printf("========================================\n");
      Serial.printf(">>> MOVED from %s to %s\n",
                    senders[firstTrippedIdx].id, senders[secondIdx].id);
      Serial.printf(">>>   %s triggered first, %s triggered %.1fs later\n",
                    senders[firstTrippedIdx].id, senders[secondIdx].id,
                    timeDiff / 1000.0f);
      Serial.printf("========================================\n");
      directionReported = true;
    }
  }

  if (combinedStdDev > eventPeakCombined) {
    eventPeakCombined = combinedStdDev;
  }

  // Person count
  int estimate = 1;
  if (bothSimultaneous && eventPeakCombined >= COMBINED_STDDEV_2P) {
    estimate = 2;
  }

  if (estimate != eventPeakCount) {
    eventPeakCount = estimate;
    if (estimate == 2) {
      Serial.printf(">>> 2 PEOPLE DETECTED (combined stddev=%.1f)\n", eventPeakCombined);
    }
    eventClassified = true;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n[RECEIVER] FDMA + variance + person count");
  Serial.println("Hopping: 915.0 MHz (S1) <-> 916.0 MHz (S2)");

  memset(senders, 0, sizeof(senders));
  strcpy(senders[0].id, "S1");
  senders[0].freq = LORA_FREQ_S1;
  strcpy(senders[1].id, "S2");
  senders[1].freq = LORA_FREQ_S2;

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

  int state = radio.begin(LORA_FREQ_S1, LORA_BW, LORA_SF, LORA_CR,
                          LORA_SYNC_WORD, LORA_POWER, LORA_PREAMBLE);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("radio.begin failed: %d\n", state);
    while (true) delay(1000);
  }

  radio.setCRC(true);
  radio.setPacketReceivedAction(onRxDone);

  Serial.println("# FORMAT: DATA,rx_millis,sender,seq,rssi,stddev,status");
  Serial.printf("Hop=%dms, window=%d, trip=%.1f stddev, 2-person threshold=%.1f combined\n",
                HOP_INTERVAL_MS, VAR_WINDOW, STDDEV_TRIP, COMBINED_STDDEV_2P);

  lastHop = millis();
  switchFreq(0);
}

void processPacket() {
  String payload;
  int state = radio.readData(payload);
  if (state != RADIOLIB_ERR_NONE) {
    radio.startReceive();
    return;
  }

  float rawRssi = radio.getRSSI();
  unsigned long now = millis();

  int colon = payload.indexOf(':');
  if (colon < 1 || colon > 3) {
    radio.startReceive();
    return;
  }

  char senderId[4] = {0};
  payload.substring(0, colon).toCharArray(senderId, sizeof(senderId));

  int idx = senderIndex(senderId);
  if (idx < 0) {
    radio.startReceive();
    return;
  }

  String seqStr = "";
  for (int i = colon + 1; i < (int)payload.length(); i++) {
    char c = payload.charAt(i);
    if (c >= '0' && c <= '9') seqStr += c;
    else break;
  }
  uint32_t seq = seqStr.toInt();

  SenderState &s = senders[idx];
  s.totalPkts++;

  if (s.seqInit && seq > s.lastSeq + 1) {
    // Gaps expected at ~50% receive rate
  }
  s.lastSeq = seq;
  s.seqInit = true;

  // Add to variance window
  s.rssiWindow[s.winIdx] = rawRssi;
  s.winIdx = (s.winIdx + 1) % VAR_WINDOW;
  if (s.winFill < VAR_WINDOW) s.winFill++;

  float sd = computeStdDev(s);
  s.latestStdDev = sd;

  // Per-link detection
  const char *status = "CLEAR";

  if (sd >= STDDEV_TRIP) {
    s.tripCount++;
    if (s.tripCount >= TRIP_COUNT) {
      if (!s.tripped) {
        // New trip on this link
        s.tripped = true;
        s.tripStartTime = now;
        s.peakStdDev = sd;

        Serial.printf(">>> MOTION on %s link (stddev=%.1f dB)\n", senderId, sd);

        // Start or continue event
        if (!eventActive) {
          eventActive = true;
          eventStartTime = now;
          eventPeakCombined = 0;
          eventPeakCount = 0;
          eventClassified = false;
          firstTrippedIdx = idx;
          directionReported = false;
        }
      }
      // Update peak
      if (sd > s.peakStdDev) s.peakStdDev = sd;

      status = "MOTION";

      // Re-classify with updated data
      classifyEvent();
    } else {
      status = "ALERT";
    }
  } else {
    if (s.tripped) {
      Serial.printf(">>> CLEAR on %s link (stddev=%.1f dB)\n", senderId, sd);
      s.tripped = false;
      s.peakStdDev = 0;
    }
    s.tripCount = 0;

    // Check if all links are clear — end event
    bool anyTripped = false;
    for (int i = 0; i < NUM_SENDERS; i++) {
      if (senders[i].tripped) anyTripped = true;
    }
    if (!anyTripped && eventActive) {
      unsigned long duration = now - eventStartTime;
      if (directionReported) {
        int secondIdx = (firstTrippedIdx == 0) ? 1 : 0;
        Serial.printf(">>> EVENT ENDED: %s -> %s, %d person(s), duration %.1fs\n",
                      senders[firstTrippedIdx].id, senders[secondIdx].id,
                      eventPeakCount > 0 ? eventPeakCount : 1,
                      duration / 1000.0f);
      } else {
        Serial.printf(">>> EVENT ENDED: %s only, %d person(s), duration %.1fs\n",
                      senders[firstTrippedIdx].id,
                      eventPeakCount > 0 ? eventPeakCount : 1,
                      duration / 1000.0f);
      }
      Serial.println("----------------------------------------");
      eventActive = false;
    }
  }

  Serial.printf("DATA,%lu,%s,%u,%.1f,%.1f,%s\n",
                now, senderId, seq, rawRssi, sd, status);

  radio.startReceive();
}

void loop() {
  unsigned long now = millis();

  if (now - lastHop >= HOP_INTERVAL_MS) {
    int nextSlot = (currentSlot + 1) % NUM_SENDERS;
    switchFreq(nextSlot);
    lastHop = now;
  }

  if (rxFlag) {
    rxFlag = false;
    processPacket();
  }
}