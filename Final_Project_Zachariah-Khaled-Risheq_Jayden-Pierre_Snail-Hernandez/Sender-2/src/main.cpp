#include <Arduino.h>
#include <RadioLib.h>

#define SENDER_ID  "S1"

#define LORA_NSS    8
#define LORA_DIO1   14
#define LORA_RST    12
#define LORA_BUSY   13
#define LORA_SCK    9
#define LORA_MISO   11
#define LORA_MOSI   10

#define LORA_FREQ          915.0   // S2 on 916.0
#define LORA_BW            125.0
#define LORA_SF            7
#define LORA_CR            5
#define LORA_SYNC_WORD     0x12
#define LORA_POWER         2
#define LORA_PREAMBLE      8

#define TX_PERIOD_MS       100
#define PAYLOAD_LEN        16

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

uint32_t seq = 0;
unsigned long nextTx = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.printf("\n[SENDER %s] FDMA on %.1f MHz, %dms period\n",
                SENDER_ID, LORA_FREQ, TX_PERIOD_MS);

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

  int state = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR,
                          LORA_SYNC_WORD, LORA_POWER, LORA_PREAMBLE);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("radio.begin failed: %d\n", state);
    while (true) delay(1000);
  }

  radio.setCRC(true);
  nextTx = millis();
}

void loop() {
  unsigned long now = millis();
  if (now >= nextTx) {
    uint8_t payload[PAYLOAD_LEN];
    memset(payload, 'X', PAYLOAD_LEN);
    char header[PAYLOAD_LEN + 1];
    int n = snprintf(header, sizeof(header), "%s:%lu", SENDER_ID, (unsigned long)seq);
    if (n > PAYLOAD_LEN) n = PAYLOAD_LEN;
    memcpy(payload, header, n);

    radio.transmit(payload, PAYLOAD_LEN);
    seq++;
    nextTx += TX_PERIOD_MS;
    if (nextTx < now) nextTx = now + TX_PERIOD_MS;
  }
}