/*
 Retry, 8/23/2022 Version
 Advertiser version 7_8_922
*/

// For mbed compilers, the following allows us to use the
// standard printf() function.
//REDIRECT_STDOUT_TO(Serial)

#include <ArduinoBLE.h>
#include <Servo.h>

//############################################
//Various Filters
#include "MadgwickAHRS.h"
#include "SensorFusion.h" //SF
SF fusion;
#include "simpleFusion.h" 

//#############################################

int  xACCval ;  
float BatteryL ;

//#######################################
      float floatValueX;
      float floatValueY;
      float floatValueZ;
      float BatteryVoltage , vBat = 0 ;
      float floatValueGyroX;
      float floatValueGyroY;
      float floatValueGyroZ;
      float xGyro, yGyro, zGyro, pitchFiltered, deltat ; 
      float pitchFilteredOld= 0 ;
      float pitch, roll, yaw ;

//########################################
// Servo related
int servoPin1 = 12, servoPin2 = 11, servoPin3 = 10;
Servo servo1, servo2, servo3;
int pos = 25;    // variable to store the servo position
int Offset = 90 ; 
int FilteredXangle = 0 ;  
int FilteredYangle = 0 ;
int PitchAngle = 0 ;
int RollAngle = 0 ;
//

//Battery Related Settings
const double vRef = 3.3; // Assumes 3.3V regulator output is ADC reference voltage
const unsigned int numReadings = 1024; // 10-bit ADC readings 0-1023, so the factor is 1024
long previousMillis = 0 ;
unsigned int adcCount ;
double adcVoltage ;


//Filters 
/*
Madgwick filter;
const float sensorRate = 104.00;
*/
//


//BLE SERVICE for other devices to read Battery voltage, Still testing//
//#############################
BLEService BLECentral("1200") ;
//BLEFloatCharacteristic CxAccChar("0011" ,BLERead | BLENotify | BLEBroadcast) ;
BLEFloatCharacteristic CxAccChar("0011" , BLERead | BLENotify | BLEBroadcast) ;
// Bluetooth® Low Energy Battery Level Characteristic
BLEUnsignedCharCharacteristic batteryLevelChar("2A19", BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes


//#############################

void setup() {

        servo1.attach(servoPin1); 
        servo2.attach(servoPin2);
        servo3.attach(servoPin3);

        servo1.write(pos); //Init the servo1 angle to 0
//      servo2.write(0); //Init the servo2 angle to 0
//      servo3.write(0); //Init the servo2 angle to 0
  
//   Serial.begin(115200);
//  Serial.begin(9600);
 // while (!Serial);
  // configure the button pin as input
 // pinMode(buttonPin, INPUT);

//Battery Read , we need to set this up to read Battery
  pinMode(P0_14, OUTPUT);
  digitalWrite(P0_14, LOW);
// 

//##############################################################################
  // initialize the Bluetooth® Low Energy hardware
  BLE.begin();
  Serial.println("Bluetooth® Low Energy Central - LED control");
  // start scanning for peripherals
  BLE.scanForUuid("1100");
  Serial.println("Setup complete!");
  //delay(5000);

  //############### This is for communicating data to other devices from this device##########################
   BLE.setLocalName("Central") ;
   BLE.setAdvertisedService(BLECentral) ;
   BLECentral.addCharacteristic(CxAccChar) ;
   BLECentral.addCharacteristic(batteryLevelChar) ;
   BLE.addService(BLECentral) ; 
   CxAccChar.writeValue(10.01) ; //set initial value
   batteryLevelChar.writeValue(1.1) ; //Set initial Value
   BLE.advertise() ;
   //######################
}

void loop() {
      
  // check if a peripheral has been discovered
//  BLEDevice peripheral = BLE.available();

  BLEDevice peripheral = BLE.available();

  if (peripheral) {
    // discovered a peripheral, print out address, local name, and advertised service
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();


//Turn On LED

//
    // stop scanning, no more scan is required.
    BLE.stopScan();

while (peripheral) {
      GetupdatedIMUdata(peripheral) ;
    }


  }

  
    // peripheral disconnected, start scanning again
    BLE.scanForUuid("1100"); //start scanning again
  
}



void GetupdatedIMUdata(BLEDevice peripheral) {
  Serial.println("Connecting ...");

  if (peripheral.connect()) {
    Serial.println("Connected");
  } else {
    Serial.println("Failed to connect!");
    return;
  }

  // discover peripheral attributes
  Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes()) {
    Serial.println("Attributes discovered");
  } else {
    Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }

//#############################################################################
//   BLECharacteristic ledCharacteristic = peripheral.characteristic("0007");
//   BLECharacteristic ledCharacteristic = peripheral.characteristic("0007");
     BLECharacteristic IMUxValue = peripheral.characteristic("0007");
     BLECharacteristic IMUyValue = peripheral.characteristic("0008");
     BLECharacteristic IMUzValue = peripheral.characteristic("0009");
 
     BLECharacteristic batteryVoltageCHAR = peripheral.characteristic("0006"); 


     BLECharacteristic IMUxGyroValue = peripheral.characteristic("0003");
     BLECharacteristic IMUyGyroValue = peripheral.characteristic("0004");
     BLECharacteristic IMUzGyroValue = peripheral.characteristic("0005");
//#############################################################################


  if (!IMUxValue) {
    Serial.println("Peripheral does not have xAcc IMU Data!");
    peripheral.disconnect();
    return;
  } else if (!IMUxValue.canRead()) {
    Serial.println("Peripheral does not have a writable LED characteristic!");
    peripheral.disconnect();
    return;
  }
  if (!batteryVoltageCHAR) {
    Serial.println("Peripheral does not have BatteryLevel Data!");
    }


  while (peripheral.connected()) {
 
 //long currentMillis = millis() ;
 //
  //      adcCount = analogRead(PIN_VBAT) ;
 //     adcVoltage = (adcCount *vRef) / numReadings ;
 //     vBat = adcVoltage *1565.0/510.0 ;  //Voltage divider from Vbat to ADC


 
 /*
      float floatValueX;
      float floatValueY;
      float floatValueZ;
      float BatteryVoltage;

      */
      
      IMUxValue.readValue( &floatValueX, 4 );
      IMUyValue.readValue( &floatValueY, 4 );
      IMUzValue.readValue( &floatValueZ, 4 );
     
      batteryVoltageCHAR.readValue(&BatteryVoltage, 4) ;

    /*
      IMUxGyroValue.readValue( &floatValueGyroX, 4 ), IMUyGyroValue.readValue( &floatValueGyroY, 4 ) , IMUzGyroValue.readValue( &floatValueGyroZ, 4 );
      xGyro = floatValueGyroX * DEG_TO_RAD ;
      yGyro = floatValueGyroY * DEG_TO_RAD ;
      zGyro = floatValueGyroZ * DEG_TO_RAD ;
      */
      

/*
 *  xAcc = myIMU.readFloatAccelX() ;
    yAcc = myIMU.readFloatAccelY() ;
    zAcc = myIMU.readFloatAccelZ() ;
    xGyro = myIMU.readFloatGyroX() * DEG_TO_RAD ;
    yGyro = myIMU.readFloatGyroY() * DEG_TO_RAD ;
    zGyro = myIMU.readFloatGyroZ() * DEG_TO_RAD ;
 
 * 
 */

      
//IMUxGyroValue.readValue( &floatValueGyroX, 4 ), IMUyGyroValue.readValue( &floatValueGyroY, 4 ) , IMUzGyroValue.readValue( &floatValueGyroZ, 4 );

//Filter
   //   MahonyMadgwickFilter() ;
      FilteredXangle =  abs (Offset - ((int ((floatValueX * 90) + 1)/5)*5))  ; //rounding to 5
      PitchAngle  = (Offset +  ((int ((pitch + 1)/5)*5)))  ; //rounding to 5
      FilteredYangle = Offset - ((int ((floatValueY * 90) + 1)/5)*5) ;
      RollAngle = int ((roll+1)/5)*5 ;
//



       //Local Battery2
    
   //  if (currentMillis - previousMillis >= 1000 ) //Every second
 //    {
 //     previousMillis = currentMillis ;
  //    updateBatteryLevel2 () ;
 //    }



     
     CxAccChar.writeValue(FilteredXangle) ; //set initial value
//     batteryLevelChar.writeValue(BatteryLevel) ;
  /*
      Serial.print(floatValueX) ;
      Serial.print('\t') ;
      Serial.print(floatValueY) ;
      Serial.print('\t') ;
      Serial.print(floatValueZ) ;
      Serial.print('\t') ;
      */

//Servo control Routines 
      Control_Elbow() ;
      Control_Wrist() ;
      Control_Fingers() ;

      
      Serial.print(BatteryVoltage) ;
      Serial.print('\t') ;
      
      Serial.print(FilteredXangle) ;
      Serial.print('\t') ;
  /*
      Serial.print(PitchAngle) ;
      Serial.print('\t') ;           
      Serial.print(pitch) ;
      Serial.print('\t') ;
      */
      Serial.print(FilteredYangle) ;
      Serial.print('\t') ;
      /*
      Serial.print(RollAngle) ;
      Serial.print('\t') ;
      Serial.print(roll) ;
      Serial.print('\t') ;
      Serial.print(yaw) ; 
      */

      //print Local battery voltage
      Serial.print(vBat) ; 
      Serial.println() ;
            
  }

 
  Serial.println("Peripheral disconnected");
}

//###################################################################

void Control_Fingers () {
  }

void Control_Wrist() {
  }

void Control_Elbow () {

/*
//   float pitchFiltered = 0.1 * pitch + 0.9 * pitchFilteredOld; // low pass filter
 //   float rollFiltered = 0.1 * roll + 0.9 * rollFilteredOld ;  // low pass filter
 //   float YawFiltered = 0.1 * roll + 0.9 * YawFilteredOld ; // low pass filter
    
 //   Serial.println("pitch: " + String(pitchFiltered));
 //   pitchFilteredOld = pitchFiltered;
 //   rollFilteredOld = rollFiltered;
 //   YawFilteredOld = YawFiltered;
    Serial.print("Orientation: ");
    Serial.print ((int ((xAcc * 90) + 9)/10)*10);
    Serial.print(" ");
    Serial.print(yAcc);
    Serial.print(" ");
    Serial.println(zAcc);
    //xAcc, yAcc, zAcc;
//int round = ((grades[j] + 4)/5) * 5;     
//   for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo1.write((int ((xAcc * 90) + 9)/10)*10);    
  */
  
  //For Control Angle 270 Degree
  if (FilteredXangle > 135) {
 // FilteredXangle = 125 ;
  servo1.write(125); 
   delay(15);
  }
  else if ((0 < FilteredXangle ) && (FilteredXangle < 40)){
 // FilteredXangle = 0 ;
  servo1.write(25);
  delay(15); 
  }
  else if ((40 < FilteredXangle ) && (FilteredXangle < 135)){
   servo1.write(FilteredXangle);              // tell servo to go to position in variable 'pos'
   delay(15);
   }                      // waits 15 ms for the servo to reach the position
   
}






void MahonyMadgwickFilter() {

 //We are NOt USING mahony for now.. but my custom filter
 // FilteredXangle =  abs (Offset - ((int ((floatValueX * 90) + 1)/5)*5))  ; //rounding to 5
  //Following is Mahony filter for gyro inclusion//

    deltat = fusion.deltatUpdate(); //this have to be done before calling the fusion update
   // fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);  //mahony is suggested if there isn't the mag and the mcu is slow
    fusion.MahonyUpdate(xGyro, yGyro, zGyro, floatValueX, floatValueY, floatValueZ, deltat);  //floatValueGyroX
    //    int FilteredXangle = ((int ((xAcc * 90) + 1)/5)*5)+ Offset  ; //rounding to 5

    //fusion.MadgwickUpdate(xGyro, yGyro, zGyro, floatValueX, floatValueY, floatValueZ, mx, my, mz deltat);  //it is slower but more accurate

    pitch = fusion.getPitch();
    roll = fusion.getRoll();    //you could also use getRollRadians() ecc
    yaw = fusion.getYaw();

 //   float pitchFiltered = 0.4 * pitch + 0.6 * pitchFilteredOld; // low pass filter
//    Serial.print(pitchFiltered);
//    pitchFilteredOld = pitchFiltered;

//  xAccChar.writeValue(xAcc);  // and update the temperature
//    xAccChar.writeValue(pitchFiltered);


}


void updateBatteryLevel2()
{
  unsigned int adcCount = analogRead(PIN_VBAT);
  double adcVoltage = (adcCount * vRef) / numReadings;
  double vBat = adcVoltage*1565.0/510.0; // Voltage divider from Vbat to ADC
  
 // printf("adcCount = %3u = 0x%03X, adcVoltage = %.3fV, vBat = %.3f\n",
 //            adcCount, adcCount, adcVoltage, vBat);
  // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
//  delay(1000);
}

/*
void updateBatteryLevel() 
{
unsigned int adcCount = analogRead(PIN_VBAT) ;
double adcVoltage = (adcCount *vRef) / numReadings ;
vBat = adcVoltage *1565.0/510.0 ;  //Voltage divider from Vbat to ADC
float BatteryLevel = ((vBat-3.5)/0.65)*100 ;
batteryLevelChar.writeValue(BatteryLevel) ;
  }
 */
