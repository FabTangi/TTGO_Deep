/*******************************************************************************
 * Copyright (c) 2018 FabLab Lannion
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload containing GPS info, battery info
 * using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <WiFi.h>
#include "esp_sleep.h"
#include "keys.h"
#include <U8x8lib.h>
// the OLED used
// pins defined in TTGO variant, pins_arduino.h
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 22, /* data=*/ 21, /* reset=*/ OLED_RST);


// T-Beam specific hardware
#define BUILTIN_LED 25

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 60 /* Time ESP32 will go to sleep (in seconds) */


RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR u4_t RTC_seqnoUp = 0;
RTC_DATA_ATTR int statCount = 0;

const uint8_t vbatPin = 35;
float VBAT; // battery voltage from ESP32 ADC read
char s[32]; // used to sprintf for Serial output
uint8_t txBuffer[4]; //buffer used to send data
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = LMIC_UNUSED_PIN, 
  .dio = {26, 33, 32},
};


typedef struct state {
  uint16_t total_snd = 0; /**< total packets send */
  uint16_t total_rcv = 0; /**< total packets received (excluding ack) */
  uint8_t gps = 0; /**< nb gps sat */
  int8_t rssi = 0; /**< last RSSI received value */
  int8_t snr = 0; /**< last snr received value */
  uint8_t ant = 0; /**< number of last seen gateways */

} state_t;


state_t curState;

/** display current cnx state
 * @param state string to be displayed
 */
void displayState (const char* state) {

  u8x8.clearLine (2);
  u8x8.drawString (0,2,state);

  Serial.println(state);
}

/** display number of packets send & received
 */
void displayStats (state_t* st) {
  char l[17];
  uint8_t n = 4;
  
  Serial.println (l);
  /* packets */
  sprintf (l, "s: %5d r:%5d", st->total_snd, st->total_rcv);
  l[16] = 0;

  u8x8.clearLine (n);
  u8x8.drawString (0,n++, l);

  Serial.println (l);
  // RSSI & SNR
  sprintf (l, "RSSI%4d SNR%4d", st->rssi, st->snr);
  l[16] = 0;

  u8x8.clearLine (n);
  u8x8.drawString (0,n++, l);
  
  Serial.println (l);
  // gps & antenas
  sprintf (l, "Sat: %2d  GWs:%3d", st->gps, st->ant);
  l[16] = 0;

  u8x8.clearLine (n);
  u8x8.drawString (0,n++, l);

  Serial.println (l);

}

void onEvent (ev_t ev) {
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      displayState ("EV_SCAN_TIMEOUT");
      break;
    case EV_BEACON_FOUND:
      displayState ("EV_BEACON_FOUND");
      break;
    case EV_BEACON_MISSED:
      displayState("EV_BEACON_MISSED");
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      displayState ("EV_JOINING");
      break;
    case EV_JOINED:
      displayState ("EV_JOINED");
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      displayState("EV_JOIN_FAILED");
      break;
    case EV_REJOIN_FAILED:
      displayState("EV_REJOIN_FAILED");
      break;
    case EV_TXCOMPLETE:
      displayState("EV_TXCOMPLETE");
      digitalWrite(BUILTIN_LED, LOW);
        curState.total_snd++;
        curState.rssi = LMIC.rssi;
        curState.snr = LMIC.snr;
        Serial.println(curState.total_snd);
        Serial.println(curState.rssi);
        Serial.println(curState.snr);
        Serial.println(LMIC.txpow+"dBm);
      if (LMIC.txrxFlags & TXRX_ACK) {
        Serial.println(F("Received Ack"));
      }
      if (LMIC.dataLen) {
        sprintf(s, "Received %i bytes of payload", LMIC.dataLen);
        Serial.println(s);
        sprintf(s, "RSSI %d SNR %.1d", LMIC.rssi, LMIC.snr);
        Serial.println(s);
        curState.total_rcv++;
        Serial.println(curState.total_rcv);
      }
    displayStats (&curState);
     // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      // go into deep sleep for TX_interval
      //RTC_seqnoUp = LMIC.seqnoUp;
      /*esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
      //This 4 lines did not have influence if I remove them I get the same ->A verifier 
      esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
      esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
      esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
      esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
      esp_deep_sleep_start(); */
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

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    Serial.println(F("OP_TXRXPEND, not sending"));
  }
  else
  {
  
    // Battery Voltage
    VBAT = (float)(analogRead(vbatPin)) / 4095*2*3.3*1.1;
    /*
    The ADC value is a 12-bit number, so the maximum value is 4095 (counting from 0).
    To convert the ADC integer value to a real voltage you’ll need to divide it by the maximum value of 4095,
    then double it (note above that Adafruit halves the voltage), then multiply that by the reference voltage of the ESP32 which
    is 3.3V and then vinally, multiply that again by the ADC Reference Voltage of 1100mV.
    */
    Serial.println("Vbat = "); Serial.print(VBAT); Serial.println(" Volts"); 
    

      txBuffer[0] = highByte(round(VBAT*100));
      txBuffer[1] = lowByte(round(VBAT*100));
      txBuffer[2] = highByte(round(bootCount));
      txBuffer[3] = lowByte(round(bootCount));

      
      LMIC_setTxData2(1, txBuffer, sizeof(txBuffer), 0);
      Serial.println(F("Packet queued"));
      digitalWrite(BUILTIN_LED, HIGH);
      displayStats(&curState);
     
      //try again in 3 seconds
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(3), do_send);
  
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Deep Sleep Tester"));


  
  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  Serial.println("RTC_seqnoUp: " + String(RTC_seqnoUp));
  Serial.println("Stationary Counter: " + String(statCount));       

  //Print the wakeup reason for ESP32
  //print_wakeup_reason(); 

  //Turn off WiFi and Bluetooth
  WiFi.mode(WIFI_OFF);
  btStop();

  // init oled screen
  u8x8.begin();
  u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);    
  u8x8.clear();
  u8x8.inverse();
  u8x8.print("Deep_Sleep Test");
  u8x8.setFont(u8x8_font_chroma48medium8_r);  
  u8x8.noInverse();
  u8x8.setCursor(0,1);

  pinMode(vbatPin, INPUT);
  #ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
  #endif
  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

/*
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(LMIC.artKey)];
  uint8_t nwkskey[sizeof(LMIC.nwkKey)];
  memcpy_P(appskey, LMIC.artKey, sizeof(LMIC.artKey));
  memcpy_P(nwkskey, LMIC.nwkKey, sizeof(LMIC.nwkKey));
  LMIC_setSession (0x1, LMIC.devaddr, nwkskey, appskey);
  */
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
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

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  //LMIC_setDrTxpow(DR_SF11,14);
  LMIC_setDrTxpow(DR_SF9,14);

  // Start job (sending automatically starts OTAA too)
do_send(&sendjob); 
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);

}

void loop() {
    os_runloop_once();
}

void print_wakeup_reason() {

  digitalWrite(BUILTIN_LED, HIGH);
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
    case 1  : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case 2  : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case 3  : Serial.println("Wakeup caused by timer"); break;
    case 4  : Serial.println("Wakeup caused by touchpad"); break;
    case 5  : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

