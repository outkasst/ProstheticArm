/*
Poor mans version, uses BLE to send IMU DATA from XIAO to another XIAO
Sensor to Central
LED turns Red when connected to Central.
Indicates Battery Voltage(May need fix)

*/




#include <ArduinoBLE.h>
#include <LSM6DS3.h>  // XIAO and ARDUINO IOT33 Connect board.
//Arduino_LSMDS63 RP2040 Connect board
//#include <Arduino_LSM6DSOX.h>
#include "SensorFusion.h" //SF
//#include "simpleFusion.h" 
SF fusion;


#define BLE_BUFFER_SIZES             20
/* Device name which can be scene in BLE scanning software. */
#define BLE_DEVICE_NAME                "Arduino Nano 33 BLE Sense"
/* Local name which should pop up when scanning for BLE devices. */
#define BLE_LOCAL_NAME                "Sensors"

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A // XIAO and ARDUINO IOT33 Connect board.
//LSM6DSOXClass myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A // RP2040 Connect board.
float xAcc, yAcc, zAcc , oldxAcc , oldBatteryL , IMUxVal , vBat ; //  Define Variables
float xGyro, yGyro, zGyro;
float pitch, roll, yaw ;
float deltat;
float pitchFilteredOld = 0 ;

int pos = 25;    // variable to store the servo position
int Offset = 90 ; 
int FilteredXangle = 0 ;

//Battery Related settings
const double vRef = 3.3; // Assumes 3.3V regulator output is ADC reference voltage
const unsigned int numReadings = 1024; // 10-bit ADC readings 0-1023, so the factor is 1024
//REDIRECT_STDOUT_TO(Serial)
//


const char* deviceServiceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* deviceServiceCharacteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";
//BLEService gestureService(deviceServiceUuid); 
//BLEByteCharacteristic gestureCharacteristic(deviceServiceCharacteristicUuid, BLERead | BLEWrite);

BLEService BLExAcc("1100"); // Bluetooth® Low Energy  xACCService
//BLEService BLExAcc(deviceServiceUuid); // Bluetooth® Low Energy  xACCService

//BLEService ledService("LEDONOFFSERVICES"); // 
// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
//BLEByteCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
// Bluetooth® Low Energy Battery Level Characteristic
//BLEUnsignedCharCharacteristic xAccChar("2803", BLERead | BLENotify); // standard 16-bit characteristic UUID
//BLEByteCharacteristic xAccChar("2803", BLERead | BLENotify | BLEWrite ); // 
//BLEUnsignedCharCharacteristic xAccChar("2803", BLERead | BLENotify | BLEWrite ); // 
BLEFloatCharacteristic xAccChar("0007", BLERead | BLENotify | BLEBroadcast );
BLEFloatCharacteristic yAccChar("0008", BLERead | BLENotify | BLEBroadcast);
BLEFloatCharacteristic zAccChar("0009", BLERead | BLENotify | BLEBroadcast);

BLEFloatCharacteristic xGyroChar("0003", BLERead | BLENotify | BLEBroadcast );
BLEFloatCharacteristic yGyroChar("0004", BLERead | BLENotify | BLEBroadcast);
BLEFloatCharacteristic zGyroChar("0005", BLERead | BLENotify | BLEBroadcast);

BLEFloatCharacteristic batteryLevelChar("0006", BLERead | BLENotify | BLEBroadcast ); // standard 16-bit characteristic UUID

//BLEUnsignedCharCharacteristic yAccChar("0008", BLERead | BLENotify | BLEBroadcast);
//BLEUnsignedCharCharacteristic zAccChar("0009", BLERead | BLENotify | BLEBroadcast);
/*
BLECharacteristic yAccChar("0008", BLERead | BLENotify | BLEBroadcast, BLE_BUFFER_SIZES);
BLECharacteristic zAccChar("0009", BLERead | BLENotify | BLEBroadcast, BLE_BUFFER_SIZES);
*/

//BLEUnsignedCharCharacteristic humidityChar("2804", BLERead | BLENotify); // standard 16-bit characteristic UUID
// Bluetooth® Low Energy Battery Level Characteristic
//BLEUnsignedCharCharacteristic batteryLevelChar("2A19", BLERead | BLENotify | BLEBroadcast ); // standard 16-bit characteristic UUID
//BLEFloatCharacteristic batteryLevelChar("2A19", BLERead | BLENotify | BLEBroadcast ); // standard 16-bit characteristic UUID
// remote clients will be able to get notifications if this characteristic changes



int oldBatteryLevel = 0;  // last battery level reading from analog input
long previousMillis = 0;  // last time the dht value was checked, in ms
 
void setup()
{
  /*
  Serial.begin(115200);    // initialize serial communication
  while (!Serial);
  */
  
  pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected
  // begin initialization
  if (!BLE.begin()) 
  {
    Serial.println("starting BLE failed!");
    while (1);
  }



//Battery //
  pinMode(P0_14, OUTPUT);
  digitalWrite(P0_14, LOW);
  //

 
  BLE.setLocalName("IMUBatt");
//  BLE.setLocalName("BatteryMonitor");
  BLE.setAdvertisedService(BLExAcc); // add the service UUID
  BLExAcc.addCharacteristic(xAccChar); // add the characteristic IMU X Data
  BLExAcc.addCharacteristic(yAccChar); // add the characteristic IMU Y Data
  BLExAcc.addCharacteristic(zAccChar); // add the characteristic IMU Z Data
 
  BLExAcc.addCharacteristic(batteryLevelChar); // add the characteristic Battery Data

  BLExAcc.addCharacteristic(xGyroChar); // add the characteristic Gyro X Data
  BLExAcc.addCharacteristic(yGyroChar); // add the characteristic Gyro Y Data
  BLExAcc.addCharacteristic(zGyroChar); // add the characteristic Gyro Z Data
  

  
  BLE.addService(BLExAcc); // Add the service
//  BLE.addService(batteryLevelChar); // Add the service
  xAccChar.writeValue(0); // set initial value for this characteristic
  yAccChar.writeValue(0); // set initial value for this characteristic
  zAccChar.writeValue(0); // set initial value for this characteristic

  batteryLevelChar.writeValue(oldBatteryL);

  xGyroChar.writeValue(0); // set initial value for this characteristic
  yGyroChar.writeValue(0); // set initial value for this characteristic
  zGyroChar.writeValue(0); // set initial value for this characteristic

  
  // start advertising
  BLE.advertise();
  Serial.println("Bluetooth® device active, waiting for connections...");
  
 
  //IMU Start
  if (myIMU.begin() != 0) {
    Serial.println("IMU Device error");
    } else {
    Serial.println("IMU Device OK!");
    }
}
 
void loop() {
//Loop every 250ms, once it is connected No LOOP needed, once exited from Loop start again every 250ms
// listen for Bluetooth® Low Energy peripherals to connect:
   BLEDevice central = BLE.central();
 
  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());
    
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, LOW);
    
    // while the central is still connected to peripheral:
  while (central.connected()) {
        updatexAcc();
 // Check battery every minute
       long currentMillis = millis();
      // if 200ms have passed, check the battery level:
      if (currentMillis - previousMillis >= 1000) // Every second
      {
        previousMillis = currentMillis;
        updateBatteryLevel();
      //  delay(2000) ;
      }

    }

  }
 
 // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.print("Disconnected from central: ");
    Serial.println() ;
    delay(5000) ;
  }



void updatexAcc()
{
    xAcc = myIMU.readFloatAccelX() ;
    yAcc = myIMU.readFloatAccelY() ;
    zAcc = myIMU.readFloatAccelZ() ;
    xGyro = myIMU.readFloatGyroX() * DEG_TO_RAD ;
    yGyro = myIMU.readFloatGyroY() * DEG_TO_RAD ;
    zGyro = myIMU.readFloatGyroZ() * DEG_TO_RAD ;
/*
  //Accelerometer
    xAcc = myIMU.readFloatAccelX() ;
    yAcc = myIMU.readFloatAccelY() ;
    zAcc = myIMU.readFloatAccelZ() 
    Serial.print("Orientation: ");
    Serial.println(xAcc);
    Serial.println();
*/

//Following is Mahony filter for gyro inclusion//

/*
    deltat = fusion.deltatUpdate(); //this have to be done before calling the fusion update
   // fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);  //mahony is suggested if there isn't the mag and the mcu is slow
    fusion.MahonyUpdate(xGyro, yGyro, zGyro, xAcc, yAcc, zAcc, deltat);
    //    int FilteredXangle = ((int ((xAcc * 90) + 1)/5)*5)+ Offset  ; //rounding to 5

    pitch = fusion.getPitch();
    roll = fusion.getRoll();    //you could also use getRollRadians() ecc
    yaw = fusion.getYaw();

    float pitchFiltered = 0.4 * pitch + 0.6 * pitchFilteredOld; // low pass filter
//    Serial.print(pitchFiltered);
    pitchFilteredOld = pitchFiltered;
//  xAccChar.writeValue(xAcc);  // and update the temperature
//    xAccChar.writeValue(pitchFiltered);
*/

//We are NOt USING mahony for now.. but my custom filter
//  int FilteredXangle =  abs (Offset - ((int ((xAcc * 90) + 1)/5)*5))  ; //rounding to 5
  xAccChar.writeValue(xAcc);
  yAccChar.writeValue(yAcc) ;
  zAccChar.writeValue(zAcc) ;


  xGyroChar.writeValue(xGyro);
  yGyroChar.writeValue(yGyro) ;
  zGyroChar.writeValue(zGyro) ;
  
  
  Serial.print(xAcc) ;  //My filter
  Serial.print('\t');
//  Serial.print(pitchFiltered);  //Mahony filter
//  Serial.print('\t');
  Serial.print(yAcc) ;
  Serial.print('\t');
  Serial.print(zAcc) ;
  Serial.print('\t');
  Serial.print(xGyro) ;
  Serial.print('\t');
  Serial.print(yGyro) ;
  Serial.print('\t');
  Serial.print(zGyro) ;
  Serial.print('\t');
  Serial.print(vBat) ;
  Serial.println() ;
    
  //  oldxAcc = IMUxVal;
 //   oldxAcc = xAcc;           // save the level for next comparison
 //   Serial.println(xAcc);

 
 /*
    Serial.print(xAcc);
    Serial.print('\t');
    Serial.print(yAcc);
    Serial.print('\t');
    Serial.print(zAcc);
    Serial.println() ; 
*/
    
 //   Serial.println("I am in IMU Read Loop and Connected") ;
  //  delay(200) ;
    }


void updateBatteryLevel()
{
  
//
  unsigned int adcCount = analogRead(PIN_VBAT);
  double adcVoltage = (adcCount * vRef) / numReadings;
 // double vBat = adcVoltage*1510.0/510.0; // Voltage divider from Vbat to ADC
//  double vBat = adcVoltage*1565.0/510.0; // Voltage divider from Vbat to ADC
  vBat = adcVoltage*1565.0/510.0; // Voltage divider from Vbat to ADC
//    printf("adcCount = %3u = 0x%03X, adcVoltage = %.3fV, vBat = %.3f\n",
//             adcCount, adcCount, adcVoltage, vBat);
//
//  int battery = analogRead(A0);
//  int batteryLevel = map(battery, 0, 1023, 0, 100);

  float batteryLevel = ((vBat-1.4)/2.5)*100 ;
 // float BatteryD = (516 - battery) * 45 / 1023.0;
 // if (batteryLevel != oldBatteryLevel)    // if the battery level has changed
 // { 
//    Serial.print("Battery Level % is now: "); // print it
//    Serial.println(batteryLevel);
//    Serial.println("Battery Voltage") ;
//    Serial.println(vBat) ;
 //   Serial.println(vBat, 3) ;
 //   batteryLevelChar.writeValue(batteryLevel);  // and update the battery level characteristic
      batteryLevelChar.writeValue(vBat);
 //   delay(3000) ;
 //   oldBatteryLevel = batteryLevel;           // save the level for next comparison
 // }
}
