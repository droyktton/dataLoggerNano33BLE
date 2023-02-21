///////////////// GPS ////////////////////////
/* 
 *  
 *  Esta libreria 

//#include <SoftwareSerial.h>

no es compatible con nano 33 ble

 pero Gracias a esto
 *  
https://forum.arduino.cc/t/arduino-nano-33-iot-neo-6m-gps-module/879016/7

logre hacer andar el serial port. 

*/


#include <Arduino_LSM9DS1.h>

// include GPS library
#include <TinyGPS.h>

/* This sample code demonstrates the normal use of a TinyGPS object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/

/* vistas en la web, pero no funcionan 
//SoftwareSerial ss(4, 3);
//UART ss(digitalPinToPinName(1), digitalPinToPinName(2), NC, NC);
*/

TinyGPS gps;


///////////////// SD ////////////////////////
// include the SD library:
#include <SPI.h>
#include <SD.h>

// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;

// change this to match your SD shield or module;
// Arduino Ethernet shield: pin 4
// Adafruit SD shields and modules: pin 10
// Sparkfun SD shield: pin 8
// MKRZero SD: SDCARD_SS_PIN
const int chipSelect = 4; // Nano 33 ble pin digital 4 <--> CS del SD module 


///////////////// BLE ////////////////////////
// Importante: el rango de BLE de la Nano 33 BLE Sense es muy corto
// mucho mas corto que el de la Wio Terminal
// https://forum.arduino.cc/t/ble-very-weak-signal/631751
#include <ArduinoBLE.h>
BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // create service
// create switch characteristic and allow remote device to read and write

//BLEByteCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite | BLENotify);

// Algunos detalles a tener en cuenta
// https://forum.arduino.cc/t/envio-de-string-mediante-libreria-arduino-ble-solucionado/659528/6
//https://forum.arduino.cc/t/ble-code-to-take-string-input-from-another-bluetooth-enabled-deice/643877
BLEStringCharacteristic  switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite | BLENotify, 20);

// variables y funciones usadas para la comunicacion BLE
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool sdWrite = false;
bool bleWrite=false;


const int ledPin = LED_BUILTIN; // pin to use for the LED



//////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(115200);  

  ////// GPS
  Serial1.begin(9600); // se usa para comunicacion serial con el GPS
  
  Serial.print("Simple TinyGPS library v. "); Serial.println(TinyGPS::library_version());
  Serial.println("by Mikal Hart");
  Serial.println();


  /////// SD
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");


  // Initialize IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU");
    while (1);
  }
  Serial.println("IMU initialized");
  
  ////// BLE
  // begin initialization
    if (!BLE.begin()) {
      Serial.println("starting Bluetooth® Low Energy module failed!");  
      while (1);
    }
  
    // set the local name peripheral advertises
    BLE.setLocalName("Tortuga Nano");
    // set the UUID for the service this peripheral advertises
    BLE.setAdvertisedService(ledService);
  
    // add the characteristic to the service
    ledService.addCharacteristic(switchCharacteristic);
  
    // add service
    BLE.addService(ledService);
  
    // assign event handlers for connected, disconnected to peripheral
    BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
    BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  
    // assign event handlers for characteristic
    switchCharacteristic.setEventHandler(BLEWritten, switchCharacteristicWritten);
    // set an initial value for the characteristic
    switchCharacteristic.setValue("0");

  
    // start advertising
    BLE.advertise();
  
    Serial.println(("Bluetooth® device active, waiting for connections..."));

    /*while(!deviceConnected) {
    BLE.poll();
    };*/

}

void loop()
{
    BLE.poll();
    bool newData = false;
   
    unsigned long chars;
    unsigned short sentences, failed;
  
    // make a string for assembling the data to log:
    String dataString = "";
  
    // aqui vamos a escribir las coordenadas en el SD
    File dataFile = SD.open("gpslog.txt", FILE_WRITE);
  
  
    // vars para IMU
    float x, y, z;
    float ax, ay, az, a2x, a2y, a2z;
    float gx, gy, gz, g2x, g2y, g2z;
    float mx, my, mz, m2x, m2y, m2z;
    int na,ng,nm;
    ax=ay=az=gx=gy=gz=mx=my=mz=0.0;
    a2x=a2y=a2z=g2x=g2y=g2z=m2x=m2y=m2z=0.0;
    na=ng=nm=0;

    int Nsec=10;
    // For Nsec seconds we parse GPS data and report some key values
    for (unsigned long start = millis(); millis() - start < 1000*Nsec;)
    {
      while (Serial1.available())
      {
        char c = Serial1.read();
        // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
        if (gps.encode(c)) // Did a new valid sentence come in?
          newData = true;
      }
      if (IMU.accelerationAvailable()) {
          IMU.readAcceleration(x, y, z);
          ax+=x;ay+=y;az+=z;
          a2x+=x*x;a2y+=y*y;a2z+=z*z;
          na++;  
      }        
      if (IMU.gyroscopeAvailable()) {
          IMU.readGyroscope(x, y, z);
          gx+=x;gy+=y;gz+=z;  
          g2x+=x*x;g2y+=y*y;g2z+=z*z;  
          ng++;  
      }        
      if (IMU.magneticFieldAvailable()) {
          IMU.readMagneticField(x, y, z);
          mx+=x;my+=y;mz+=z;  
          m2x+=x*x;m2y+=y*y;m2z+=z*z;  
          nm++;  
      }
    }
    
    dataString+= String(ax/na)+" "+String(ay/na)+" "+String(az/na)+" ";   
    dataString+= String(gx/ng)+" "+String(gy/ng)+" "+String(gz/ng)+" ";   
    dataString+= String(mx/nm)+" "+String(my/na)+" "+String(mz/na)+" ";   

    dataString+= String(a2x/na)+" "+String(a2y/na)+" "+String(a2z/na)+" ";   
    dataString+= String(g2x/ng)+" "+String(g2y/ng)+" "+String(g2z/ng)+" ";   
    dataString+= String(m2x/nm)+" "+String(m2y/na)+" "+String(m2z/na)+" ";   
  
   
  // solo si hay datos de gps
  if (newData)
  {

    unsigned long date,time,age;
    float flat, flon;
    long latitude,longitude;
    unsigned long fix_age;

    // date as ddmmyy, time as hhmmsscc, and age in milliseconds (UTC -> restar tres horas en ARG)
    gps.get_datetime(&date, &time, &age);    
    
    // lat/long in MILLIONTHs of a degree and age of fix in milliseconds
    // (note: versions 12 and earlier gave lat/long in 100,000ths of a degree.
    gps.get_position(&latitude, &longitude, &fix_age);
    //gps.f_get_position(&flat, &flon, &age);
    
    /*Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.println(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
    */

    // colecto los datos de GPS en un string para guardar en GPS
    // los campos son: LAT, LON, SAR, PREC, HDOP, DATE, TIME 
    
    //dataString+= String(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat); 
    dataString+= String(latitude == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : latitude); 
    dataString += ",";

    //dataString+= String(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon);
    dataString+= String(longitude == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : longitude);
    dataString+= ",";

    dataString+= String(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    dataString+= ",";

    dataString+= String(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
    dataString+= ",";

    dataString+= String(date == TinyGPS::GPS_INVALID_DATE ? 0.0 : int(date)); 
    dataString += ",";

    dataString+= String(time == TinyGPS::GPS_INVALID_TIME ? 0.0 : int(time)); 
    //dataString += ",";

    // escribir sobre SD
    // if the file is available, write to it:
    if (dataFile) {
      if(sdWrite) dataFile.println(dataString);
      dataFile.close();
    }
    // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening gpslog.txt, no pude imprimir");
    }

    //Serial.println(dataString);

    // notificar en BLE
    if(bleWrite)
    switchCharacteristic.writeValue(dataString.c_str());
  }

  Serial.println(dataString);

  gps.stats(&chars, &sentences, &failed);
  Serial.print(" CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);
  if (chars == 0)
    Serial.println("** No characters received from GPS: check wiring **");

}



void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
  deviceConnected = true;
  delay(2000);
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
  deviceConnected = false;
}

void switchCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  // central wrote new value to characteristic, update LED
  Serial.print("Characteristic event, written: ");

  if (switchCharacteristic.value()) {
    Serial.println("LED on");
    digitalWrite(ledPin, HIGH);
  } else {
    Serial.println("LED off");
    digitalWrite(ledPin, LOW);    
  }

  // SD 
  if (switchCharacteristic.value()=="sd") {
    if(sdWrite){
    switchCharacteristic.writeValue("SD write on");
    Serial.println("SD write on");
    }
    else{
    switchCharacteristic.writeValue("SD write off");    
    Serial.println("SD write off");
    }
  }  
  if (switchCharacteristic.value()=="sdon") {
    //Serial.print("eh, como andas?");
    sdWrite=true;
    switchCharacteristic.writeValue("SD write on");
    Serial.println("SD write on");
  }  
  if (switchCharacteristic.value()=="sdoff") {
    //Serial.print("eh, como andas?");
    sdWrite=false;
    switchCharacteristic.writeValue("SD write off");
    Serial.println("SD write off");
  }  

  // BLE 
  if (switchCharacteristic.value()=="ble") {
    if(bleWrite){
    switchCharacteristic.writeValue("BLE write on");
    Serial.println("BLE write on");
    }
    else{
    switchCharacteristic.writeValue("BLE write off");    
    Serial.println("BLE write off");
    }   
  }  
  if (switchCharacteristic.value()=="bleon") {
    //Serial.print("eh, como andas?");
    bleWrite=true;
    Serial.println("BLE write on");
    switchCharacteristic.writeValue("BLE write on");
  }  
  if (switchCharacteristic.value()=="bleoff") {
    //Serial.print("eh, como andas?");
    bleWrite=false;
    Serial.println("BLE write off");
    switchCharacteristic.writeValue("BLE write off");
  }  

}


/*
 * Cosas que encontre utiles
 * 
 * Para ver en google earth el csv
 * gawk -F ',' '{print $1/1000000,",",$2/1000000}' GPSLOG.TXT > caminata.csv
 * y luego convertirlo a kml con esto
 * https://www.convertcsv.com/csv-to-kml.htm
 * 
 * Observaciones:
 * El rango bluetooth de la nano 33 ble no es muy grande, hay que acercarse
 * 
 */
