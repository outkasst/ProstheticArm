#include <Servo.h>
#include <LSM6DS3.h>
#include <MadgwickAHRS.h>
#include <Math.h>
#include "SensorFusion.h" //SF
//#include "simpleFusion.h" 
SF fusion ;
// ================================================================

// ===                      INITIAL SETUP                       ===

// ================================================================
//Example setup by Sammy, please make it efficient and better organized.



//int servoPin1 = 12, servoPin2 = 11, servoPin3 = 10;
float xAcc, yAcc, zAcc;
float xGyro, yGyro, zGyro;
float pitch, roll, yaw ;
float deltat;
float pitchFilteredOld = 0 ;
int FilteredXangleCustom , FilteredYangleCustom , FilteredZangleCustom  ;

int pos = 25;    // variable to store the servo position
int Offset = 90 ; 

//Servo servo1 ; // , servo2, servo3;



//float pitchFilteredOld;
float rollFilteredOld;
float YawFilteredOld;

Madgwick filter;
const float sensorRate = 104.00;

int servoPin1 = 12, servoPin2 = 11, servoPin3 = 10;
Servo servo1, servo2, servo3;
//int pos = 0;    // variable to store the servo position


//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A // XIAO and ARDUINO IOT33 Connect board.

void setup() {

        servo1.attach(servoPin1); 
        servo2.attach(servoPin2);
        servo3.attach(servoPin3);

        servo1.write(0); //Init the servo1 angle to 0
        servo2.write(0); //Init the servo2 angle to 0
        servo3.write(0); //Init the servo2 angle to 0


  //Serial.begin(9600);
  /*
  while (!Serial);
  if(!IMU.begin())  {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

*/

 
  //IMU Start
  if (myIMU.begin() != 0) {
    Serial.println("IMU Device error");
    } else {
    Serial.println("IMU Device OK!");
    }

  filter.begin(sensorRate);
  Serial.println("Setup complete!");
  delay(10000);

}

//  Serial.print(yAcc);
//    Serial.print(" ");

void loop() {
updateIMU() ;
Control_Elbow_Servo() ; //FilteredXangleCustom value is used for now, basic

//Control_ElbowLock()
//Control_Elbow_twist_Servo()
//Control_Wrist_Servo()
//Control_All_Fingers_Grip_Servo()
//Control_Finger1_Servo()
//Control_Finger2_4_Servo()
//Control_Finger5_Servo()


delay(100) ;

}


void updateIMU()
{
    xAcc = myIMU.readFloatAccelX() ;
    yAcc = myIMU.readFloatAccelY() ;
    zAcc = myIMU.readFloatAccelZ() ;
    xGyro = myIMU.readFloatGyroX() * DEG_TO_RAD ;
    yGyro = myIMU.readFloatGyroY() * DEG_TO_RAD ;
    zGyro = myIMU.readFloatGyroZ() * DEG_TO_RAD ;






//RollPitchYaw calculation and Filtering
//Following is Mahony filter for gyro inclusion//

    deltat = fusion.deltatUpdate(); //this have to be done before calling the fusion update
   // fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);  //mahony is suggested if there isn't the mag and the mcu is slow
    fusion.MahonyUpdate(xGyro, yGyro, zGyro, xAcc, yAcc, zAcc, deltat);
   
   //My own Angle Calculation
     FilteredXangleCustom = ((int ((xAcc * 90) + 1)/5)*5)+ Offset  ; //rounding to 5
     FilteredYangleCustom = ((int ((yAcc * 90) + 1)/5)*5)+ Offset  ; //rounding to 5
     FilteredZangleCustom = ((int ((zAcc * 90) + 1)/5)*5)+ Offset  ; //rounding to 5

//Roll Pitch Yaw
    pitch = fusion.getPitch();
    roll = fusion.getRoll();    //you could also use getRollRadians() ecc
    yaw = fusion.getYaw();

    float pitchFiltered = 0.4 * pitch + 0.6 * pitchFilteredOld; // low pass filter
//    Serial.print(pitchFiltered);
    pitchFilteredOld = pitchFiltered;


Serial.print("RawX") ;
Serial.print('\t');
Serial.print(xAcc) ;  //My filter
Serial.print('\t');
Serial.print("RawY") ;
Serial.print(yAcc) ;
Serial.print('\t');
Serial.print("RawZ") ;
Serial.print(zAcc) ;
Serial.print('\t');
Serial.print("RawGyroX") ;
Serial.print(xGyro) ;
Serial.print('\t');
Serial.print("RawGyroY") ;
Serial.print(yGyro) ;
Serial.print('\t');
Serial.print("RawGyroz") ;
Serial.print(zGyro) ;
Serial.print('\t');

Serial.print("MahonyPitchX & LowpassFilter") ;
Serial.print('\t');
Serial.print(pitchFiltered);  //Mahony filter
Serial.print('\t');

Serial.print("My own AngleX") ;
Serial.print('\t');
Serial.print(FilteredXangleCustom);
Serial.print('\t');
Serial.print("My own AngleY") ;
Serial.print('\t');
Serial.print(FilteredYangleCustom);
Serial.print('\t');
Serial.print("My own AngleZ") ;
Serial.print('\t');
Serial.print(FilteredZangleCustom);
Serial.println() ;  


}


void Control_Elbow_Servo ()
{
/*
Basic : FilteredXangleCustom value is used for now,  Should take IMU, GESTURE, JOYSTICK, Pressure Sensor Data and Use Prioritzation value to define Elbow Angle, Code will be updated later.
Define Priority of data: FilteredYangleCustom is priority 1, all other sensor data priority is Zero. code will use Just FilteredYangleCustom data to define Elbow Angle
For Control Angle 270 Degree
*/
  if (FilteredXangleCustom > 135) {
  FilteredXangleCustom = 125 ;
   servo1.write(125); 
   Serial.print("Elbow ServoAngle") ;
   Serial.print('\t');
   Serial.print(FilteredXangleCustom);
   Serial.print('\t');
   delay(15);
  }
  else if ((0 < FilteredXangleCustom ) && (FilteredXangleCustom < 40)){
  FilteredXangleCustom = 25 ;
  servo1.write(25);
   Serial.print("Elbow ServoAngle") ;
   Serial.print('\t');
   Serial.print(FilteredXangleCustom);
   Serial.print('\t');
  delay(15); 
  }
  else if ((40 < FilteredXangleCustom ) && (FilteredXangleCustom < 135)){
   servo1.write(FilteredXangleCustom);              // tell servo to go to position in variable 'pos'
   Serial.print("Elbow ServoAngle") ;
   Serial.print('\t');
   Serial.print(FilteredXangleCustom);
   Serial.print('\t');
   delay(15);
   }                      // waits 15 ms for the servo to reach the position


}


/*
Serial.print('\t') ;
Serial.print("ELBOW Servo Angle") ;
Serial.print('\t') ;
Serial.print() ;


*/


/*
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
*/




/*

  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo1.write(pos);              // tell servo to go to position in variable 'pos'
    Serial.print("Orientation:+Positive ");
    Serial.print(" ");
    Serial.print(pos);
    Serial.print(" ");
    delay(2000);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    servo1.write(pos);              // tell servo to go to position in variable 'pos'
    Serial.print("Orientation:+Negative");
    Serial.print(" ");
    Serial.print(pos);
    Serial.print(" ");
    delay(2000);                       // waits 15 ms for the servo to reach the position
  }
delay(2000);
}
*/
