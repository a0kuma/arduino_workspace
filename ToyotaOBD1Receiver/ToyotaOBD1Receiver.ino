// Stripped Toyota OBD1 receiver for an Arduino Uno-compatible ATmega328P.
// Signal input: D2. Status output: CH340/USB Serial at 115200 baud.
//
// The packet decoder is intentionally based on hyperion11/toyota-obd-1 OBD.ino:
// long HIGH preamble, 8 ms bit cells, 4-bit ID, then bytes framed as
// LOW start, 8 data bits LSB first, and two HIGH stop bits.

#include <Arduino.h>

#define LED_PIN 13
#define ENGINE_DATA_PIN 2
#define MY_HIGH HIGH
#define MY_LOW LOW
#define TOYOTA_MAX_BYTES 24

#define OBD_INJ 1
#define OBD_IGN 2
#define OBD_IAC 3
#define OBD_RPM 4
#define OBD_MAP 5
#define OBD_ECT 6
#define OBD_TPS 7
#define OBD_SPD 8
#define OBD_OXSENS 9

volatile uint8_t ToyotaNumBytes = 0;
volatile uint8_t ToyotaID = 0;
volatile uint8_t ToyotaData[TOYOTA_MAX_BYTES];
volatile uint16_t ToyotaFailBit = 0;
volatile bool ToyotaFailPending = false;

void ChangeState();

float getOBDdata(byte OBDdataIDX) {
  float returnValue;
  switch (OBDdataIDX) {
    case 0:
      returnValue = ToyotaData[0];
      break;
    case OBD_INJ:
      returnValue = ToyotaData[OBD_INJ] * 0.125;
      break;
    case OBD_IGN:
      returnValue = ToyotaData[OBD_IGN] * 0.47 - 30;
      break;
    case OBD_IAC:
      returnValue = ToyotaData[OBD_IAC] * 0.39215;
      break;
    case OBD_RPM:
      returnValue = ToyotaData[OBD_RPM] * 25;
      break;
    case OBD_MAP:
      returnValue = ToyotaData[OBD_MAP] * 2;
      break;
    case OBD_ECT:
      if (ToyotaData[OBD_ECT] >= 243) {
        returnValue = ((float)(ToyotaData[OBD_ECT] - 243) * 9.8) + 122;
      } else if (ToyotaData[OBD_ECT] >= 237) {
        returnValue = ((float)(ToyotaData[OBD_ECT] - 237) * 3.83) + 99;
      } else if (ToyotaData[OBD_ECT] >= 228) {
        returnValue = ((float)(ToyotaData[OBD_ECT] - 228) * 2.11) + 80.0;
      } else if (ToyotaData[OBD_ECT] >= 210) {
        returnValue = ((float)(ToyotaData[OBD_ECT] - 210) * 1.11) + 60.0;
      } else if (ToyotaData[OBD_ECT] >= 180) {
        returnValue = ((float)(ToyotaData[OBD_ECT] - 180) * 0.67) + 40.0;
      } else if (ToyotaData[OBD_ECT] >= 135) {
        returnValue = ((float)(ToyotaData[OBD_ECT] - 135) * 0.44) + 20.0;
      } else if (ToyotaData[OBD_ECT] >= 82) {
        returnValue = ((float)(ToyotaData[OBD_ECT] - 82) * 0.38);
      } else if (ToyotaData[OBD_ECT] >= 39) {
        returnValue = ((float)(ToyotaData[OBD_ECT] - 39) * 0.47) - 20.0;
      } else if (ToyotaData[OBD_ECT] >= 15) {
        returnValue = ((float)(ToyotaData[OBD_ECT] - 15) * 0.83) - 40.0;
      } else {
        returnValue = ((float)(ToyotaData[OBD_ECT] - 15) * 2.0) - 60.0;
      }
      break;
    case OBD_TPS:
      returnValue = ToyotaData[OBD_TPS] / 1.8;
      break;
    case OBD_SPD:
      returnValue = ToyotaData[OBD_SPD];
      break;
    case OBD_OXSENS:
      returnValue = (float)ToyotaData[OBD_OXSENS] * 0.01953125;
      break;
    default:
      returnValue = 9999.99;
      break;
  }
  return returnValue;
}

static void printHexByte(uint8_t value) {
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

static void printDecodedPacket(uint8_t id, const uint8_t *data, uint8_t count) {
  Serial.print(F("PACKET id=0x"));
  Serial.print(id, HEX);
  Serial.print(F(" bytes="));
  Serial.print(count);
  Serial.print(F(" raw="));
  for (uint8_t i = 0; i < count; i++) {
    if (i > 0) {
      Serial.print(' ');
    }
    printHexByte(data[i]);
  }

  if (count > OBD_OXSENS) {
    Serial.print(F(" | INJ="));
    Serial.print(getOBDdata(OBD_INJ), 2);
    Serial.print(F("ms IGN="));
    Serial.print(getOBDdata(OBD_IGN), 1);
    Serial.print(F("deg IAC="));
    Serial.print(getOBDdata(OBD_IAC), 1);
    Serial.print(F("% RPM="));
    Serial.print(getOBDdata(OBD_RPM), 0);
    Serial.print(F(" MAP="));
    Serial.print(getOBDdata(OBD_MAP), 0);
    Serial.print(F(" ECT="));
    Serial.print(getOBDdata(OBD_ECT), 1);
    Serial.print(F("C TPS="));
    Serial.print(getOBDdata(OBD_TPS), 1);
    Serial.print(F("% SPD="));
    Serial.print(getOBDdata(OBD_SPD), 0);
    Serial.print(F("kmh O2="));
    Serial.print(getOBDdata(OBD_OXSENS), 2);
    Serial.print(F("V"));
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(ENGINE_DATA_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENGINE_DATA_PIN), ChangeState, CHANGE);

  Serial.println(F("READY toyota_obd1_receiver input=D2 serial=115200 bit_cell_ms=8"));
}

void loop() {
  uint8_t count = 0;
  uint8_t id = 0;
  uint8_t data[TOYOTA_MAX_BYTES];
  uint16_t failBit = 0;
  bool failPending = false;

  noInterrupts();
  if (ToyotaNumBytes > 0) {
    count = ToyotaNumBytes;
    id = ToyotaID;
    for (uint8_t i = 0; i < count; i++) {
      data[i] = ToyotaData[i];
    }
    ToyotaNumBytes = 0;
  }
  if (ToyotaFailPending) {
    failBit = ToyotaFailBit;
    failPending = true;
    ToyotaFailPending = false;
  }
  interrupts();

  if (count > 0) {
    printDecodedPacket(id, data, count);
  }
  if (failPending) {
    Serial.print(F("DECODE_FAIL bit="));
    Serial.println(failBit);
  }
}

void ChangeState() {
  static uint8_t ID, EData[TOYOTA_MAX_BYTES];
  static boolean InPacket = false;
  static unsigned long StartMS;
  static uint16_t BitCount;

  int state = digitalRead(ENGINE_DATA_PIN);
  digitalWrite(LED_PIN, state);

  if (InPacket == false) {
    if (state == MY_HIGH) {
      StartMS = millis();
    } else {
      if ((millis() - StartMS) > (15 * 8)) {
        StartMS = millis();
        InPacket = true;
        BitCount = 0;
      }
    }
  } else {
    uint16_t bits = ((millis() - StartMS) + 1) / 8;
    StartMS = millis();

    while (bits > 0) {
      if (BitCount < 4) {
        if (BitCount == 0) {
          ID = 0;
        }
        ID >>= 1;
        if (state == MY_LOW) {
          ID |= 0x08;
        }
      } else {
        uint16_t bitpos = (BitCount - 4) % 11;
        uint16_t bytepos = (BitCount - 4) / 11;

        if (bitpos == 0) {
          if ((BitCount > 4) && (state != MY_HIGH)) {
            ToyotaFailBit = BitCount;
            ToyotaFailPending = true;
            InPacket = false;
            break;
          }
        } else if (bitpos < 9) {
          EData[bytepos] >>= 1;
          if (state == MY_LOW) {
            EData[bytepos] |= 0x80;
          }
        } else {
          if (state != MY_LOW) {
            ToyotaFailBit = BitCount;
            ToyotaFailPending = true;
            InPacket = false;
            break;
          }
          if ((bitpos == 10) && ((bits > 1) || (bytepos == (TOYOTA_MAX_BYTES - 1)))) {
            ToyotaNumBytes = 0;
            ToyotaID = ID;
            for (uint16_t i = 0; i <= bytepos; i++) {
              ToyotaData[i] = EData[i];
            }
            ToyotaNumBytes = bytepos + 1;
            if (bits >= 16) {
              BitCount = 0;
            } else {
              ToyotaFailBit = BitCount;
              ToyotaFailPending = true;
              InPacket = false;
            }
            break;
          }
        }
      }
      ++BitCount;
      --bits;
    }
  }
}
