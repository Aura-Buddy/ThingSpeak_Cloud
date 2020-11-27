/*
  Device: UPRIGHT Furniture Monitoring System
  
  Creators: Levi Newton, Brandon Tiner, Aura Teasley
  
  Purpose: Alarm System using Thingspeak cloud to communicate motion detected
  from ESP32 SparkFun Thing to cloud to be picked up by another device (mobile app).
  User is allowed to input network name, network password, channel ID, channel Write Key
  through bluetooth connection wth mobile app rather than hardcoded.
  
  Hardware: ESP32 based boards
  
  ThingSpeak.h provided by MathWorks, INC
  Copyright 2018, The MathWorks, Inc.

  NOTICE: The bluetooth address is hardcoded into MAC address, this is directly related to name associated with
  bluetooth instance from esp32. Thus maintain current name for device communication to function as intended.
*/

#include "ThingSpeak.h"
#include "BluetoothSerial.h"
#include <WiFi.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

WiFiClient  client;
BluetoothSerial SerialBT;

// IMU Preprocessing:
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
LSM9DS1 imu;                                                                                          //Create an imu object of the LSM9DS1 class
#define LSM9DS1_M  0x1E                                                                               //Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG 0x6B                                                                               //Would be 0x6A if SDO_AG is LOW

String ssidString;                                                                                    //network name
String passString;                                                                                    //network password
String myChannelNumberString;                                                                         //channel number
String myWriteAPIKeyString;                                                                           //channel write key
String bluetoothstoreString = "";

char bluetoothChar;
char ssid[] = "";
char pass[] = "";
char myWriteAPIKey[17];
const char* myWriteAPIKeyPointer = myWriteAPIKey;

unsigned long myChannelNumberInt; 
int bluetoothInt;
int breakFlag = 0;
int deliminatorCount = 0;
int number;


//IMU variables and functions
unsigned long lastPrint = 0;
unsigned long startTime;
unsigned long prevTime = 0;
const unsigned int PRINT_RATE = 500;

float roll;
float pitch;
float yaw;
float roll_offset;
float pitch_offset;
float yaw_offset;


void printSensorReadings();                                                                           //this function can be removed if not printing to serial monitor
void getAngles();
void setupGyro();                                                                                     //enables gyro 
void setupAccel();                                                                                    //enables accelerometer
void setupMag();                                                                                      //enables magnetometer
uint16_t initLSM9DS1();

void setup() {
  //initialize Serial, Bluetooth, WiFi and ThingSpeak instances
  Serial.begin(115200);
  SerialBT.begin("UpRight Device");
  WiFi.mode(WIFI_STA);   
  ThingSpeak.begin(client);
  
  Serial.println("The device started, now you can pair it with bluetooth!");

  //getting strings from bluetooth connection
  while(!SerialBT.available())                                                                        //while bluetooth device is not connected
  {
    delay(2000);
    Serial.println("Waiting for bluetooth connection");
    while(SerialBT.available())                                                                       //when bluetooth device is connected
    {
      bluetoothChar = SerialBT.read();
      bluetoothInt = bluetoothChar;
      bluetoothstoreString = bluetoothstoreString + bluetoothChar;
      Serial.println(bluetoothChar);                                                                  //read serial input of from bluetooth feed
      
      //storing in substrings
      if(deliminatorCount == 0)                                                                       //no deliminators have been found, deliminator is '+' character
      {
        if(bluetoothInt != 43)                                                                        //only add to ssidstring if character is not deliminator
          ssidString = ssidString + bluetoothChar; 
        }
        
      if(deliminatorCount == 1)                                                                       //first deliminator has been found
      {
        if(bluetoothInt != 43)
          passString = passString + bluetoothChar;
        }

      if(deliminatorCount == 2)                                                                       //second deliminator has been found
      { 
        if(bluetoothInt != 43)
          myChannelNumberString = myChannelNumberString + bluetoothChar;
        }

      if(deliminatorCount == 3)                                                                       //third and last deliminator has been found
      {
        if(bluetoothInt != 35)                                                                        //only add to string if character is not '#' (terminating character for bluetooth read)
          myWriteAPIKeyString = myWriteAPIKeyString + bluetoothChar;
        }

      if(bluetoothInt == 43)                                                                          //ascii value for '+', chosen deliminator for substrings
      {
        deliminatorCount++;
        }
        
      if(bluetoothInt == 35)                                                                          //ascii value for '#' is 35
      {
        Serial.println("End of input");
        breakFlag = 1;
        break;
        }
      }
    if(breakFlag == 1)
    {
      Serial.println("Finished bluetooth read");
      SerialBT.end();                                                                                 //ending bluetooth communication
      break;
      }
    }

  //changing substrings into appropriate forms
                                                                                                      //ssidString to ssid char array
  char ssid[ssidString.length()-1];
  for(int i = 0; i < ssidString.length(); i++)
    ssid[i] = ssidString.charAt(i);
                                                                                                      //passString to pass char array
  char pass[passString.length()-1];
  for(int i = 0; i < passString.length(); i++)
    pass[i] = passString.charAt(i);

  //Connection to WiFi
  if(WiFi.status() != WL_CONNECTED){
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssidString);
    while(WiFi.status() != WL_CONNECTED){
      WiFi.begin(ssid, pass);                                                                         //Connect to WPA/WPA2 network
      Serial.print(".");
      delay(5000);     
    } 
    Serial.println("\nConnected.");
  }

  myChannelNumberInt = myChannelNumberString.toInt();                                                 //myChannelNumberString to myChannelNumberInt                         
  
  for(int i = 0; i < 16; i++)                                                                         //myWriteAPIKeyString to myWriteAPIKey char array
    myWriteAPIKey[i] = myWriteAPIKeyString.charAt(i);
  
  //IMU serial output to check sensor readings
  Wire.begin();
  
  Serial.println("Initializing the LSM9DS1");
  uint16_t status = initLSM9DS1();
  Serial.print("LSM9DS1 WHO_AM_I's returned: 0x");
  Serial.println(status, HEX);
  Serial.println("Should be 0x683D");
  Serial.println();
  
  startTime = millis();                                                                               //millis() returns runtime in miliseconds
}

void loop() {

  //IMU main loop
  
  if (imu.magAvailable() or imu.gyroAvailable() or imu.accelAvailable() )
  {
  getAngles(millis() - prevTime);
  prevTime = millis();
  }
  
  if (millis() < 1000) {                                                                              //wait 1 second after reset, then set the current angles as the offset. 
    roll_offset = roll;                                                                               //alert will need to be inactive for the first few seconds while the true angle is measured
    pitch_offset = pitch; 
    yaw_offset = yaw;
  }
  
  //Every PRINT_RATE milliseconds, print sensor data:
  if ((lastPrint + PRINT_RATE) < millis())
  {
    printSensorReadings();
    lastPrint = millis();
  }
  //IMU main loop ends
  
  delay(1000);
  
  if(abs(roll - roll_offset) || abs(pitch - pitch_offset))                                            //if roll or pitch != 0 send 1, else send 0. 1 indicates tilt alert
    number = 1;
  else
    number = 0;   
  
  int x = ThingSpeak.writeField(myChannelNumberInt, 1, number, myWriteAPIKeyPointer);
  if(x == 200)
    Serial.println("Channel update successful.");
  else
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  
  delay(20000);                                                                                       //Wait 20 seconds to update the channel again

}

// IMU Functions
void getAngles(long int dt)
{  
  imu.readAccel();
  imu.readMag();
  imu.readGyro();
  float alpha = 0.90;
  float Accel_X = imu.calcAccel(imu.ax);                                                              //units are G's
  float Accel_Y = imu.calcAccel(imu.ay);
  float Accel_Z = imu.calcAccel(imu.az); 
  float Gyro_X = imu.calcGyro(imu.gx)/57.3;                                                           //units are degrees / seconds, convert to radians
  float Gyro_Y = imu.calcGyro(imu.gy)/57.3; 
  float Gyro_Z = imu.calcGyro(imu.gz)/57.3;
  float Mag_X = imu.calcMag(imu.mx);                                                                  //units are Gauss
  float Mag_Y = imu.calcMag(imu.my);
  float Mag_Z = imu.calcMag(imu.mz);
  float Accel_Normalize = sqrt(Accel_X*Accel_X +Accel_Y*Accel_Y+Accel_Z*Accel_Z);

  Accel_X =Accel_X/Accel_Normalize;                                                                   //Normalize accelerometer values before passing to arctangent  
  Accel_Y =Accel_Y/Accel_Normalize;
  Accel_Z =Accel_Z/Accel_Normalize;

  float   Mag_Normalize = sqrt(Mag_X*Mag_X + Mag_Y*Mag_Y + Mag_Z*Mag_Z);

  Mag_X =Mag_X / Mag_Normalize;                                                                       //Normalize magnetometer values before passing to arctangent
  Mag_Y =Mag_Y / Mag_Normalize;
  Mag_Z =Mag_Z / Mag_Normalize;
 
  pitch = atan2 (Accel_Y ,( sqrt ((Accel_X * Accel_X) + (Accel_Z * Accel_Z))));                       //Euler angles using accelerometer
  roll = atan2(-Accel_X ,( sqrt((Accel_Y * Accel_Y) + (Accel_Z * Accel_Z))));

  float My = (Mag_Y * cos(roll)) + (Mag_Z * sin(roll));                                               //yaw from magnetometer (not possible with accelerometer, cant use Z axis 
  float Mx = (Mag_X * cos(pitch))+(Mag_Y * sin(roll)*sin(pitch)) - (Mag_Z * cos(roll) * sin(pitch));

  yaw =  atan2(My, Mx); 
  roll =  (alpha * (roll + (Gyro_X) * dt / 1000.0f) + (1 - alpha) * (Accel_X) ) * 57.3;               //complementary filter
  pitch = (alpha * (pitch + Gyro_Y * dt / 1000.0f) + (1 - alpha) * (Accel_Y) ) * 57.3;                //complementary filter
  yaw =   (alpha * (yaw + Gyro_Z * dt / 1000.0f) + (1 - alpha) * (Mag_Z) ) * 57.3;                    //complementary filter
}

void printSensorReadings()
{

Serial.print(" Roll angle : ");
Serial.println(roll - roll_offset);
Serial.print(" Pitch angle : ");
Serial.println(pitch - pitch_offset);
Serial.print(" Yaw angle : ");
Serial.println(yaw - yaw_offset);
Serial.println();
}

void setupGyro()
{
  imu.settings.gyro.enabled = true;                                                                   //Enable the gyro
                                                                                                      //[scale] sets the full-scale range of the gyroscope.
                                                                                                      //scale can be set to either 245, 500, or 2000
  imu.settings.gyro.scale = 245;                                                                      //Set scale to +/-245dps
                                                                                                      //[sampleRate] sets the output data rate (ODR) of the gyro
                                                                                                      //sampleRate can be set between 1-6
                                                                                                      //1 = 14.9    4 = 238
                                                                                                      //2 = 59.5    5 = 476
                                                                                                      //3 = 119     6 = 952
  imu.settings.gyro.sampleRate = 3;                                                                   //59.5Hz ODR
                                                                                                      //[bandwidth] can set the cutoff frequency of the gyro.
                                                                                                      //Allowed values: 0-3. Actual value of cutoff frequency
                                                                                                      //depends on the sample rate. (Datasheet section 7.12)
  imu.settings.gyro.bandwidth = 0;
                                                                                                      //[lowPowerEnable] turns low-power mode on or off.
  imu.settings.gyro.lowPowerEnable = false;                                                           //LP mode off
                                                                                                      //[HPFEnable] enables or disables the high-pass filter
  imu.settings.gyro.HPFEnable = true;                                                                 //HPF disabled
                                                                                                      //[HPFCutoff] sets the HPF cutoff frequency (if enabled)
                                                                                                      //Allowable values are 0-9. Value depends on ODR.
                                                                                                      //(Datasheet section 7.14)
  imu.settings.gyro.HPFCutoff = 1;                                                                    //HPF cutoff = 4Hz
                                                                                                      //[flipX], [flipY], and [flipZ] are booleans that can
                                                                                                      //automatically switch the positive/negative orientation
                                                                                                      //of the three gyro axes.
  imu.settings.gyro.flipX = false;                                                                    //Don't flip X
  imu.settings.gyro.flipY = false;                                                                    //Don't flip Y
  imu.settings.gyro.flipZ = false;                                                                    //Don't flip Z
}

void setupAccel()
{
                                                                                                      //[enabled] turns the acclerometer on or off.
  imu.settings.accel.enabled = true;                                                                  //Enable accelerometer
                                                                                                      //[enableX], [enableY], and [enableZ] can turn on or off
                                                                                                      //select axes of the acclerometer.
  imu.settings.accel.enableX = true;                                                                  //Enable X
  imu.settings.accel.enableY = true;                                                                  //Enable Y
  imu.settings.accel.enableZ = true;                                                                  //Enable Z
                                                                                                      //[scale] sets the full-scale range of the accelerometer.
                                                                                                      //accel scale can be 2, 4, 8, or 16
  imu.settings.accel.scale = 8;                                                                       //Set accel scale to +/-8g.
                                                                                                      //[sampleRate] sets the output data rate (ODR) of the
                                                                                                      //accelerometer. ONLY APPLICABLE WHEN THE GYROSCOPE IS
                                                                                                      //DISABLED! Otherwise accel sample rate = gyro sample rate.
                                                                                                      //accel sample rate can be 1-6
                                                                                                      //1 = 10 Hz    4 = 238 Hz
                                                                                                      //2 = 50 Hz    5 = 476 Hz
                                                                                                      //3 = 119 Hz   6 = 952 Hz
  imu.settings.accel.sampleRate = 1;                                                                  //Set accel to 10Hz.
                                                                                                      //[bandwidth] sets the anti-aliasing filter bandwidth.
                                                                                                      //Accel cutoff freqeuncy can be any value between -1 - 3. 
                                                                                                      //-1 = bandwidth determined by sample rate
                                                                                                      //0 = 408 Hz   2 = 105 Hz
                                                                                                      //1 = 211 Hz   3 = 50 Hz
  imu.settings.accel.bandwidth = 0;                                                                   //BW = 408Hz
                                                                                                      //[highResEnable] enables or disables high resolution 
                                                                                                      //mode for the acclerometer.
  imu.settings.accel.highResEnable = false;                                                           //Disable HR
                                                                                                      //[highResBandwidth] sets the LP cutoff frequency of
                                                                                                      //the accelerometer if it's in high-res mode.
                                                                                                      //can be any value between 0-3
                                                                                                      //LP cutoff is set to a factor of sample rate
                                                                                                      //0 = ODR/50    2 = ODR/9
                                                                                                      //1 = ODR/100   3 = ODR/400
  imu.settings.accel.highResBandwidth = 0;  
}

void setupMag()
{
                                                                                                      //[enabled] turns the magnetometer on or off.
  imu.settings.mag.enabled = true;                                                                    //Enable magnetometer
                                                                                                      //[scale] sets the full-scale range of the magnetometer
                                                                                                      //mag scale can be 4, 8, 12, or 16
  imu.settings.mag.scale = 12;                                                                        //Set mag scale to +/-12 Gs
                                                                                                      //[sampleRate] sets the output data rate (ODR) of the
                                                                                                      //magnetometer.
                                                                                                      //mag data rate can be 0-7:
                                                                                                      //0 = 0.625 Hz  4 = 10 Hz
                                                                                                      //1 = 1.25 Hz   5 = 20 Hz
                                                                                                      //2 = 2.5 Hz    6 = 40 Hz
                                                                                                      //3 = 5 Hz      7 = 80 Hz
  imu.settings.mag.sampleRate = 5;                                                                    //Set OD rate to 20Hz
                                                                                                      //[tempCompensationEnable] enables or disables 
                                                                                                      //temperature compensation of the magnetometer.
  imu.settings.mag.tempCompensationEnable = false;
                                                                                                      //[XYPerformance] sets the x and y-axis performance of the
                                                                                                      //magnetometer to either:
                                                                                                      //0 = Low power mode      2 = high performance
                                                                                                      //1 = medium performance  3 = ultra-high performance
  imu.settings.mag.XYPerformance = 3;                                                                 //Ultra-high perform.
                                                                                                      //[ZPerformance] does the same thing, but only for the z
  imu.settings.mag.ZPerformance = 3;                                                                  //Ultra-high perform.
                                                                                                      //[lowPowerEnable] enables or disables low power mode in
                                                                                                      //the magnetometer.
  imu.settings.mag.lowPowerEnable = false;
                                                                                                      //[operatingMode] sets the operating mode of the
                                                                                                      //magnetometer. operatingMode can be 0-2:
                                                                                                      //0 = continuous conversion
                                                                                                      //1 = single-conversion
                                                                                                      //2 = power down
  imu.settings.mag.operatingMode = 0;                                                                 //Continuous mode
}

uint16_t initLSM9DS1()
{
  setupGyro();                                                                                        //Set up gyroscope parameters
  setupAccel();                                                                                       //Set up accelerometer parameters
  setupMag();                                                                                         //Set up magnetometer parameters
  return imu.begin(LSM9DS1_AG, LSM9DS1_M, Wire);                                                      //for SPI use beginSPI()
}
