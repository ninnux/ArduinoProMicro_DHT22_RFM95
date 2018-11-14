

#include <FlexiTimer2.h>
#include <Adafruit_SleepyDog.h>



/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
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
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
//#include <SparkFun_RHT03.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

//Libraries
#include <DHT.h>;

//Constants
#define DHTPIN 2     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino


#define VBATPIN A9
   



const int RHT03_DATA_PIN = 11;

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.

#include "config.h"
// make a config.h file with the three following lines
//static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
//static const u1_t PROGMEM DEVEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
//static const u1_t PROGMEM APPKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}


void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

//static uint8_t mydata[] = "il butirroplex,42 7";
uint8_t mydata[40];
static osjob_t sendjob;


//int timetosleep =0;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 300;

// Pin mapping

const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,   
    .rst = LMIC_UNUSED_PIN,
    .dio = {4, 5, 7},
};

//RHT03 rht;


void(* resetta)(void) = 0;

void read_dht_data(float* humidity, float* temperature,int debug)
{
    //delay(2000);
    //Read data and store it to variables hum and temp
    int i=0;
    int n=10;
    float temp=0;
    float hum=0;
    float tempsum=0;
    float humsum=0;
    for(i=0;i<n;i++){
    hum = dht.readHumidity();
    temp = dht.readTemperature();
    if(debug==1){
    Serial.print("Humidity: ");
    Serial.print(hum);
    Serial.print(" %, Temp: ");
    Serial.print(temp);
    Serial.print(" Celsius");
    Serial.println(" buttato");
    
    }
    }
    for(i=0;i<n;i++){
    humsum += dht.readHumidity();
    tempsum += dht.readTemperature();
    if(debug==1){
    Serial.print("Humidity: ");
    Serial.print(humsum);
    Serial.print(" %, Temp: ");
    Serial.print(tempsum);
    Serial.print(" Celsius");
    Serial.println(" preso per la media");
    }
    }
     *temperature=tempsum/n;
     *humidity=humsum/n;
    
}



void enterSleep(void)
{
//  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   /* EDIT: could also use SLEEP_MODE_PWR_DOWN for lowest power consumption. */
  sleep_enable();
  
  /* Now enter sleep mode. */
  sleep_mode();
  
  /* The program will continue from here after the WDT timeout*/
  sleep_disable(); /* First thing to do is disable sleep. */
  
  /* Re-enable the peripherals. */
  power_all_enable();

  //Watchdog.sleep(8000);
}

void enterSleepMinutes(int minutes)
{
  int times=0;
  times=int(minutes*60/8);
  for(int i=0;i<times;i++){
    enterSleep();
  }
  //timetosleep=0;
}


void wdt_setup(){
  MCUSR &= ~(1<<WDRF);
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  WDTCSR = 1<<WDP0 | 1<<WDP3; /* 8.0 seconds */
  WDTCSR |= _BV(WDIE);
}


void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
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

            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
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
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            //enterSleepMinutes(1);
            //resetta();
            //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(1), do_send);
            //timetosleep=1;
            //os_setCallback(&sendjob, do_send);
            
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

/*
void sensor_values(float *temp,float *hum){
  int i=0;
  const int n=10;
  float temparr[10];
  float humarr[10];
  float tempsum=0;
  float humsum=0;
//  for(i=0;i<n;i++){
//    int updateRet=rht.update();
//    if (updateRet == 1){
//      temparr[i]=rht.tempC();
//      humarr[i]=rht.humidity();
//    }
//  }
  for(i=0;i<n;i++){
    int updateRet = rht.update();
    if (updateRet == 1)
    {  
     temparr[i]=rht.tempC();
     humarr[i]=rht.humidity();
    }
  }
  for(i=0;i<n;i++){
    tempsum+=temparr[i];
    humsum+=humarr[i];
  }
  *temp=tempsum/n;
  *hum=tempsum/n;
}
*/
void do_send(osjob_t* j){

      float latestHumidity = 0;
      float latestTempC = 0;
      read_dht_data(&latestHumidity,&latestTempC,0);
      int lHi=latestHumidity*10;
      int lTi=latestTempC*10;
      //int lbat=measuredvbat*10;
      sprintf(mydata,"hum:%d,temp:%d",lHi,lTi);
     
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(9600);
    delay(5000);
    //radio.sleep();
    wdt_setup(); // DA CONTROLLARE
    //Serial.println("inizio");
    
    Serial.println(F("Starting"));


    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    // Start job (sending automatically starts OTAA too)
    
    do_send(&sendjob);
    //timetosleep =0;
    
}

void loop() {
   os_runloop_once();   
   
}
