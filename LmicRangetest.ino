/*******************************************************************************
   Copyright (c) 2016 Thomas Telkamp, Matthijs Kooijman, Bas Peschier, Harmen Zijp

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

   In order to compile the following libraries need to be installed:
   - SparkFunHTU21D: https://github.com/sparkfun/SparkFun_HTU21D_Breakout_Arduino_Library
   - NeoGPS: https://github.com/SlashDevin/NeoGPS
   - Adafruit_SleepyDog: https://github.com/adafruit/Adafruit_SleepyDog
   - lmic: https://github.com/matthijskooijman/arduino-lmic
 *******************************************************************************/

// include external libraries
#include <SPI.h>
#include <SoftwareSerial.h>
#include <NMEAGPS.h>
#include <lmic.h>
#include <hal/hal.h>

// LoRaWAN NwkSKey, network session key
static u1_t NWKSKEY[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// LoRaWAN AppSKey, application session key
static u1_t APPSKEY[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// LoRaWAN end-device address (DevAddr)
static u4_t DEVADDR = 0x00000000;

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

// setup GPS module
byte const GPS_PIN = 8;
SoftwareSerial gpsSerial(GPS_PIN, GPS_PIN);
NMEAGPS gps;

// define various pins
byte const SW_GND_PIN = 20;
int const LORA_PORT = 10;

gps_fix gps_log[8];
uint8_t gps_log_next = 0;

#define lengthof(x) (sizeof(x) / sizeof(*(x)))

const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 3, 4},
};

enum class State {
  WAITING_FOR_GPS,
  WAITING_FOR_FIX,
  TRANSMITTING,
  WAITING_FOR_AIRTIME,
};

State state = State::WAITING_FOR_GPS;

void setState(State newState);
void setState(State newState) {
  state = newState;
  switch (newState) {
    case State::WAITING_FOR_GPS:
      Serial.println(F("WAITING_FOR_GPS"));
      break;
    case State::WAITING_FOR_FIX:
      Serial.println(F("WAITING_FOR_FIX"));
      break;
    case State::TRANSMITTING:
      Serial.println(F("TRANSMITTING"));
      break;
    case State::WAITING_FOR_AIRTIME:
      Serial.println(F("WAITING_FOR_AIRTIME"));
      break;
  }
}

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      setState(State::WAITING_FOR_AIRTIME);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void setup() {
  // Write OSCCAL from EEPROM
  uint8_t osccal_byte = eeprom_read_byte((uint8_t*)4);
  if (osccal_byte != 0xff) {
    OSCCAL = osccal_byte;
  }

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set up personalized activation
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);

  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // Use a fixed data rate of SF9 (not sure if tx power is actually
  // used). SF9 is the lowest datarate that (withing the TTN fair-usage-policy of 30 seconds of airtime
  // per day) allows us to send at least 4 packets every hour.
  LMIC_setDrTxpow(DR_SF9, 14);

  // Let LMIC compensate for +/- 1% clock error
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

  // setup switched ground and power down connected peripherals (GPS module)
  pinMode(SW_GND_PIN, OUTPUT);
  digitalWrite(SW_GND_PIN, HIGH);

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);
  Serial.println(F("Start"));

  // start communication to GPS
  gpsSerial.begin(9600);
}

void showState() {
  bool led;
  switch (state) {
    case State::WAITING_FOR_GPS:
      led = true;
      break;
    case State::WAITING_FOR_FIX:
      led = (millis() / 1024) % 2;
      break;
    case State::TRANSMITTING:
      led = (millis() / 64) % 2;
      break;
    case State::WAITING_FOR_AIRTIME:
      led = false
      break;
  }
  digitalWrite(LED_BUILTIN, led);
}

void loop() {
  showState();
  // LMIC loop
  os_runloop_once();
  // See if GPS data is ready
  if (gps.available(gpsSerial)) {
    gps_log[gps_log_next] = gps.read();
    bool have_fix = gps_log[gps_log_next].valid.location;
    if (state == State::WAITING_FOR_GPS)
      setState(State::WAITING_FOR_FIX);
    if (state == State::WAITING_FOR_FIX && have_fix)
      setState(State::WAITING_FOR_AIRTIME);

    if (have_fix && (LMIC.opmode & OP_TXRXPEND) == 0) {
      // Try sending the data. If TX is now pending, we'll be sending.
      // If not, we're waiting for duty cycle limits, so cancel
      // transmission to prevent sending an outdated GPS position later.
      queueData();

      if ((LMIC.opmode & OP_TXRXPEND) != 0) {
        // Packet is being sent now, advance gps log
        gps_log_next = (gps_log_next + 1) % lengthof(gps_log);
        dumpData();
        setState(State::TRANSMITTING);
      } else {
        // No airtime available yet, clear packet again
        LMIC_clrTxData();
      }
    }
  }
}

void dumpData() {
  for (uint8_t i = 0; i < lengthof(gps_log); ++i) {
    if (gps_log[i].valid.location) {
      Serial.print(i);
      Serial.print(F(": "));

      Serial.print(gps_log[i].latitudeL()/10000000.0, 6);
      Serial.print(F(", "));
      Serial.print(gps_log[i].longitudeL()/10000000.0, 6);
      if (i == gps_log_next) {
        Serial.println(F(" [most recent value]"));
      } else {
        Serial.println();
      }
    }
  }
}

void gpsPack(uint8_t *buf, int32_t data) {
  uint32_t scaled = int32_t((int64_t)data * 32768 / 10000000);
  buf[0] = scaled >> 16 & 0xFF;
  buf[1] = scaled >> 8 & 0xFF;
  buf[2] = scaled & 0xFF;
}

void queueData() {
  uint8_t data[(lengthof(gps_log)) * 6];
  uint8_t *ptr = data;

  for (uint8_t i = 0; i < lengthof(gps_log); ++i) {
    gps_fix *cur = &gps_log[(gps_log_next + i) % lengthof(gps_log)];
    if (cur->valid.location) {
      gpsPack(ptr, cur->latitudeL());
      ptr += 3;
      gpsPack(ptr, cur->longitudeL());
      ptr += 3;
    }
  }

  // Prepare upstream data transmission at the next possible time.
  LMIC_setTxData2(LORA_PORT, data, ptr - data, 0);
}

