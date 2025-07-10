/*
 *
 */

#define BUSWARE_TUL 1

#include <WiFi.h>
#include "busware.h"
#include "version.h"

#ifdef USE_IMPROV
  #include <ImprovWiFiLibrary.h>
  ImprovWiFi improvSerial(&Serial);
#endif

// main definitions:
#if defined(BUSWARE_EUL)

  #define MYNAME "EUL"

  #if defined(CONFIG_IDF_TARGET_ESP32C3)
    TCMTransceiver Transceiver(&Serial0, 3, 5);
  #elif defined(CONFIG_IDF_TARGET_ESP32S2)
    TCMTransceiver Transceiver(&Serial1, 21);
  #endif

#elif defined(BUSWARE_TUL)

  #define MYNAME "TUL"
  TPUARTTransceiver Transceiver(&Serial0);

#elif defined(BUSWARE_ZUL)

  #define MYNAME "ZUL"
  ZigbeeTransceiver Transceiver(&Serial0, 3, 2);

#elif defined(BUSWARE_CUN)

  #define MYNAME "CUN"
  CSMTransceiver Transceiver(&Serial0, 2);

#else

  #error "No matching gadget"

#endif

#define MAXBUF 1024
static uint16_t inByte;   // for reading from serial
byte smlMessage[MAXBUF];  // for storing the isolated message. 
static int led_state = LOW;
static uint32_t req_millis = 0;
static uint32_t prev_millis = 0;
static constexpr int32_t interval = 500;

void setup(void) {

  String UniqueName = String(MYNAME) + "-" + getBase32ID();

  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname( UniqueName.c_str() );

  Transceiver.begin();
  Serial.begin(19200);
  pinMode(LED_BUILTIN, OUTPUT);

  #ifdef USE_IMPROV
    improvSerial.setDeviceInfo(
      #if defined(CONFIG_IDF_TARGET_ESP32C3)
        ImprovTypes::ChipFamily::CF_ESP32_C3,
      #elif defined(CONFIG_IDF_TARGET_ESP32S2)
          ImprovTypes::ChipFamily::CF_ESP32_S2,
      #endif
      WiFi.getHostname(),
      VERSION_SHORT,
      MYNAME
    );
#endif

  Serial.print(WiFi.getHostname());
  Serial.print(" - init succeed - running: ");
  Serial.print(VERSION);
  Serial.print(" @ ");
  Serial.print(getCpuFrequencyMhz());
  Serial.println(" MHz");
}

void loop(void) {

  uint8_t i;
  uint16_t av;
  uint8_t sbuf[MAXBUF];
  uint32_t current_millis = millis();

  Transceiver.set_reset(false);
  av = Serial.available();
  if(0<av) {
    if(MAXBUF<av) {
      av = MAXBUF;
    }
    Serial.readBytes(sbuf, av);

    #ifdef USE_IMPROV
      if(!improvSerial.handleBuffer(sbuf, av))
    #endif  
        Transceiver.write( sbuf, av );
      digitalWrite(LED_BUILTIN, HIGH);
      prev_millis = current_millis;
    }

  av = Transceiver.available();
  if(0<av) {
    if(MAXBUF<av) av = MAXBUF;
    Transceiver.readBytes(sbuf, av);

    Serial.write( sbuf, av ) ;
    digitalWrite(LED_BUILTIN, HIGH);
    prev_millis = current_millis;
  }

  // LED blinking
  auto delta = (current_millis - prev_millis);
  if(interval<=delta) {
    prev_millis = current_millis;
    if (led_state == HIGH)
      led_state = LOW;
    else
      led_state = HIGH;
    digitalWrite(LED_BUILTIN, led_state);
  }
}

