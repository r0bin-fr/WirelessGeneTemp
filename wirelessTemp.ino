/*********************************************************************
 *********************************************
 *   ArtiGene Remote Sensor - for Gene Cafe  
 *********************************************
 * 
 * Features: Temperature, batt and orientation sensor + communication through BTLE
 * 
 * Code by M.Hameau
 * Board select in Arduino IDE: "ADAFRUIT BLUEFRUIT MICRO"
 * Specs: https://learn.adafruit.com/bluefruit-le-micro-atmega32u4-microcontroller-usb-bluetooth-le-in-one/overview
*********************************************************************/

#include <Arduino.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif
#include <SPI.h>

//******************************
// GLOBAL VARS 
//******************************
//DEBUG mode
//----------
//#define LOG_MESSAGES //uncomment for debug
#ifdef LOG_MESSAGES
  #define HWLOGGING Serial
#else
  #define HWLOGGING if (1) {} else Serial
#endif

//BLUEFRUIT PINOUT 
//------------------

//#define SDA2                2 //Accelerator ADXL345 SDA I2C pin
//#define SCL3                3 //Accelerator ADXL345 SCL I2C pin
//#define BLUEFRUIT_SPI_RST   4 //PIN4 = Bluefruit reset 
#define ONE_WIRE_BUS          5 //One Wire for MAXIM temperature sensor
//#define BLUEFRUIT_SPI_IRQ   7 //PIN7 = Bluefruit IRQ
//#define BLUEFRUIT_SPI_CS    8 //PIN8 = Bluefruit CS
#define VBATPIN               A9 //pin for battery voltage check
//#define BLUEFRUIT_SPI_SCK   //PIN SCK = Bluefuirt SCK
//#define BLUEFRUIT_SPI_MISO  //PIN MISO = Bluefruit MISO
//#define BLUEFRUIT_SPI_MOSI  //PIN MOSI = Bluefruit MOSI



//******************************
// SECTION BLUEFRUIT 
//******************************
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BLEGatt.h"
#include "BluefruitConfig.h"
#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
//#define MODE_LED_BEHAVIOUR          "MODE"
#define MODE_LED_BEHAVIOUR          "0" 
#define ENABLE_SERIAL 0

// Create the bluefruit object = hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


//Service GATT
Adafruit_BLEGatt gatt(ble);

// A small helper
void error(const __FlashStringHelper*err) {
  HWLOGGING.println(err);
  while (1);
}
// The BLE service information 
int32_t htsServiceId;
int32_t htsMeasureCharId;
// BLE Characteristics & services
int32_t temperatureCharacteristicMax;
int32_t temperatureCharacteristicMin;
int32_t temperatureCharacteristicCurr;
int32_t batteryCharacteristic;
int32_t accelXCharacteristics;
int32_t accelYCharacteristics;
int32_t accelZCharacteristics;
int32_t thermometerService;

// SETUP BLE
//******************************
void setupBLE(void){
  
  /* Initialise the module */
  HWLOGGING.print(F("Initialising the Bluefruit LE module: "));
  
  if ( !ble.begin(VERBOSE_MODE) ){
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  HWLOGGING.println( F("OK!") );

  if ( FACTORYRESET_ENABLE ){
    /* Perform a factory reset to make sure everything is in a known state */
    HWLOGGING.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      //error(F("Couldn't factory reset"));
      HWLOGGING.println(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
 // ble.echo(false);

  /* Print Bluefruit information */
  HWLOGGING.println("Requesting Bluefruit info:");
  ble.info();

  uint8_t thermometerServiceUUID = 0x1809; 
  thermometerService = gatt.addService(thermometerServiceUUID);
  /* Thermo characteristics, current temp */
  uint8_t thermometerCharacteristicUUID = 0x2221; 
  temperatureCharacteristicCurr = gatt.addCharacteristic(thermometerCharacteristicUUID,
                                                     GATT_CHARS_PROPERTIES_READ | GATT_CHARS_PROPERTIES_NOTIFY,
                                                     sizeof(float), sizeof(float), BLE_DATATYPE_BYTEARRAY);
  /* Thermo characteristics, min temp */
  uint8_t thermometerMinCharacteristicUUID = 0x2222; 
  temperatureCharacteristicMin = gatt.addCharacteristic(thermometerMinCharacteristicUUID,
                                                     GATT_CHARS_PROPERTIES_READ | GATT_CHARS_PROPERTIES_NOTIFY,
                                                     sizeof(float), sizeof(float), BLE_DATATYPE_BYTEARRAY);
  /* Thermo characteristics, max temp */
  uint8_t thermometerMaxCharacteristicUUID3 = 0x2223; 
  temperatureCharacteristicMax = gatt.addCharacteristic(thermometerMaxCharacteristicUUID3,
                                                     GATT_CHARS_PROPERTIES_READ | GATT_CHARS_PROPERTIES_NOTIFY,
                                                     sizeof(float), sizeof(float), BLE_DATATYPE_BYTEARRAY);

  /* Battery characteristic */
  uint8_t batteryCharacteristicUUID = 0x2224; 
  batteryCharacteristic = gatt.addCharacteristic(batteryCharacteristicUUID,
                                                     GATT_CHARS_PROPERTIES_READ | GATT_CHARS_PROPERTIES_NOTIFY,
                                                     sizeof(float), sizeof(float), BLE_DATATYPE_BYTEARRAY);

  /* Accel 3-axes characteristic */
  uint8_t accelCharacteristicUUID = 0x2225; 
  accelXCharacteristics = gatt.addCharacteristic(accelCharacteristicUUID,
                                                     GATT_CHARS_PROPERTIES_READ | GATT_CHARS_PROPERTIES_NOTIFY,
                                                     sizeof(float), sizeof(float), BLE_DATATYPE_BYTEARRAY);
  accelCharacteristicUUID = 0x2226; 
  accelYCharacteristics = gatt.addCharacteristic(accelCharacteristicUUID,
                                                     GATT_CHARS_PROPERTIES_READ | GATT_CHARS_PROPERTIES_NOTIFY,
                                                     sizeof(float), sizeof(float), BLE_DATATYPE_BYTEARRAY);
                                                     
  accelCharacteristicUUID = 0x2227; 
  accelZCharacteristics = gatt.addCharacteristic(accelCharacteristicUUID,
                                                     GATT_CHARS_PROPERTIES_READ | GATT_CHARS_PROPERTIES_NOTIFY,
                                                     sizeof(float), sizeof(float), BLE_DATATYPE_BYTEARRAY);


  /* Reset the device for the new service setting changes to take effect */
  HWLOGGING.print(F("Performing a SW reset (service changes require a reset): "));
  ble.reset();
  
//  HWLOGGING.println(F("Change power to -16dB to save power" ));
//  ble.sendCommandCheckOK("AT+BLEPOWERLEVEL=-16");

  HWLOGGING.println(F("Now, please use Adafruit Bluefruit LE app to connect in UART mode"));
  HWLOGGING.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    HWLOGGING.println(F("******************************"));
    HWLOGGING.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    HWLOGGING.println(F("******************************"));
  }

   ble.echo(false);
}

//******************************
// SECTION TEMPERATURE 
//******************************
#include <OneWire.h>
#include <DallasTemperature.h>

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

void setupMaxim(void){
  // locate devices on the bus
  HWLOGGING.print("Locating devices...");
  sensors.begin();
  HWLOGGING.print("Found ");
  HWLOGGING.print(sensors.getDeviceCount(), DEC);
  HWLOGGING.println(" devices.");

  // set the resolution to 9 bit
  sensors.setResolution(9);
  
  // report parasite power requirements
  HWLOGGING.print("Parasite power is: "); 
  if (sensors.isParasitePowerMode()) HWLOGGING.println("ON");
  else HWLOGGING.println("OFF");  
}

//get temperature from sensor
float getKTemp(){
  // Send the command to get temperatures
  sensors.requestTemperatures(); 
  //return value
  return sensors.getTempCByIndex(0);
}


union float_bytes {
  float value;
  uint8_t bytes[sizeof(float)];
};
static union float_bytes tCelsius = { .value = 0.0 };
static union float_bytes tCelsiusMin = { .value = 0.0 };
static union float_bytes tCelsiusMax = { .value = 0.0 };
static union float_bytes bPercent = { .value = 0.0 };
static union float_bytes aDegreeX = { .value = 0.0 };
static union float_bytes aDegreeY = { .value = 0.0 };
static union float_bytes aDegreeZ = { .value = 0.0 };
float Tnow,Tmin,Tmax,Tlast = 0;
float T_dif_now, T_dif_last = 0;
int TminLoopCount, TmaxLoopCount = 0;
//matt algo
float lastAxelZval = 0.0;
float mMinTemp = 0.0;
float mMaxTemp = 999.0;
int isCyclingDown = 0;

// algorithme de tri de temperature 
// compute mini and maxi values (Gene Cafe hack)
// algoritm from evquink/RoastGenie (thank you!)
void updateTempCounters(float newtemp)
{
  Tnow = newtemp;
  TminLoopCount = TminLoopCount+1;
  TmaxLoopCount = TmaxLoopCount+1;
  T_dif_now = Tnow - Tlast;

  if (T_dif_now >= 0.0 and T_dif_last < 0.0 and TminLoopCount > 1){  // this is a local minimum
    Tmin = Tlast  ;                                                  // best estimate of environmental$
    TminLoopCount = 0;                                               // reset loop counter
    //update BLE value
    if(tCelsiusMin.value != Tlast){
      tCelsiusMin.value = Tlast;
      gatt.setChar(temperatureCharacteristicMin, tCelsiusMin.bytes, sizeof(tCelsiusMin));
      HWLOGGING.print("Temp min: ");
      HWLOGGING.println(tCelsiusMin.value);
    }
  }
  if (T_dif_now <= 0.0 and T_dif_last > 0.0 and TmaxLoopCount > 1){  // this is a local maximum
    Tmax = Tlast;                                                    // best estimate of bean mass temp
    TmaxLoopCount = 0;                                               // reset loop counter
    //update BLE value
    if(tCelsiusMax.value != Tlast){
      tCelsiusMax.value = Tlast;
      gatt.setChar(temperatureCharacteristicMax, tCelsiusMax.bytes, sizeof(tCelsiusMax));
      HWLOGGING.print("Temp max: ");
      HWLOGGING.println(tCelsiusMin.value);
    }
  }
  Tlast = Tnow;
  T_dif_last = T_dif_now;
}

//algorithm with accelerometer
void updateTempCounters2(float newtemp)
{
  float az = aDegreeZ.value;

  //get max and min values
  if(newtemp < mMinTemp)
    mMinTemp = newtemp;
  if(newtemp > mMaxTemp)
    mMaxTemp = newtemp;
    
  //are we cycling down?
  if(az <= lastAxelZval){
    if(isCyclingDown == 0){
        //broadcast new values!
        aDegreeX.value = mMinTemp;
        aDegreeY.value = mMaxTemp;
        gatt.setChar(accelXCharacteristics, aDegreeX.bytes, sizeof(aDegreeX)); 
        gatt.setChar(accelYCharacteristics, aDegreeY.bytes, sizeof(aDegreeY));
        //reset vals
        mMinTemp = 999;
        mMaxTemp = 0;
    }
    isCyclingDown = 1;
  }
  //else, we are cycling up
  else{
    isCyclingDown = 0;
  }
  //backup values
  lastAxelZval = aDegreeZ.value;
}

//******************************
// SECTION ACCELEROMETRE
//******************************
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345); //I2C config
//data from sensor
sensors_event_t accelEvent;

void setupAccel(void) 
{
  HWLOGGING.println("ADXL345 Accelerometer Calibration"); 
  HWLOGGING.println("");
  
  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    HWLOGGING.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }
  HWLOGGING.println("ADXL345 found and ready!");
}

void getAccel(void)
{
    // Get a new sensor event       
    accel.getEvent(&accelEvent);
    HWLOGGING.print("X: "); HWLOGGING.print(accelEvent.acceleration.x); HWLOGGING.print("  ");
    HWLOGGING.print("Y: "); HWLOGGING.print(accelEvent.acceleration.y); HWLOGGING.print("  ");
    HWLOGGING.print("Z: "); HWLOGGING.print(accelEvent.acceleration.z); HWLOGGING.print("  ");HWLOGGING.println("m/s^2 ");
    //broadcast to Bluetooth
    aDegreeX.value = accelEvent.acceleration.x;
    aDegreeY.value = accelEvent.acceleration.y;
    aDegreeZ.value = accelEvent.acceleration.z;
    //gatt.setChar(accelXCharacteristics, aDegreeX.bytes, sizeof(aDegreeX)); //don't care for X value
    //gatt.setChar(accelYCharacteristics, aDegreeY.bytes, sizeof(aDegreeY));
    gatt.setChar(accelZCharacteristics, aDegreeZ.bytes, sizeof(aDegreeZ));
}


//******************************
// SECTION BATTERIE
//******************************
//vars pour VMAX et VMIN
#define BATT_VCHARGE 4.3
#define BATT_VMAX 4.10
#define BATT_VMIN 3.5

//var globale pour la derniere mesure (moyenne)
float lastBattVal = 4.2;
//Affiche le voltage de la batterie lu
float getBattVal() {
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  return measuredvbat;
}

//convertit le voltage en pourcentage
float getBattPercent() {
  int bperc = 0;
  //recupere le voltage et fait la moyenne
  float bvolt = getBattVal();
  bvolt = (bvolt + lastBattVal) / 2;
  HWLOGGING.print("bvolt=");HWLOGGING.println(bvolt);
  //HWLOGGING.print("lastBattVal=");HWLOGGING.println(lastBattVal);
  lastBattVal = bvolt;
  //if we are charging, return a special value
  if (bvolt > BATT_VCHARGE)
    return 200;
  //calcul du pourcentage
  bperc = (100 * (bvolt - BATT_VMIN)) / (BATT_VMAX - BATT_VMIN);
  //HWLOGGING.print("bperc=");HWLOGGING.println(bperc);
  if (bperc > 100)
    bperc = 100;
  if (bperc < 0)
    bperc = 0;
  //HWLOGGING.print("bperc final=");HWLOGGING.println(bperc);
  return float(bperc);
  //return bvolt;
}


//******************************
// SETUP
//******************************
void setup(void)
{ 
#ifdef LOG_MESSAGES
  while (!Serial);  // required for Flora & Micro
    delay(500);
#endif 

  HWLOGGING.begin(115200);
  HWLOGGING.println(F("Adafruit Bluefruit --- Temperature sensor by Matt"));
  HWLOGGING.println(F("-------------------------------------------------"));
  
  setupMaxim();
  setupBLE();
  setupAccel();
}

float tLastVal = 0;
float bLastVal = 0;

//******************************
// LOOP
//******************************
void loop(void)
{  
  tCelsius.value = getKTemp();
  bPercent.value = getBattPercent();

  //debug print
  HWLOGGING.print("Temperature for the device 1 (index 0) is: ");
  HWLOGGING.println(tCelsius.value);   
  HWLOGGING.print("Batt percent: ");
  HWLOGGING.println(bPercent.value);


  //getAcceleration
  getAccel();

  //avoid NAN numbers
  if(isnan(tCelsius.value))
    HWLOGGING.println("NAN val");
  else{
    if(tCelsius.value != tLastVal){
       updateTempCounters(tCelsius.value);
       updateTempCounters2(tCelsius.value);
       gatt.setChar(temperatureCharacteristicCurr, tCelsius.bytes, sizeof(tCelsius));
    }
    tLastVal= tCelsius.value;
  }

  //update batt value
  if(bPercent.value != bLastVal) 
    gatt.setChar(batteryCharacteristic, bPercent.bytes, sizeof(bPercent));
  bLastVal = bPercent.value;
  
  //delay a bit
  delay(200);
}

