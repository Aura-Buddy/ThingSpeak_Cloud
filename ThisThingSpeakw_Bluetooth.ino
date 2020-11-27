  
/*
  Device: UPRIGHT Furniture Monitoring System
  
  Creator: Aura Teasley
  For: UpRights Device
  Team: Brandon Tiner, Levi Newton, Aura Teasley
  
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

#include "BluetoothSerial.h"
#include "ThingSpeak.h"
#include <WiFi.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//strings
String ssidString;                                                                  //network name
String passString;                                                                  //network password
String myChannelNumberString;                                                       //channel number
String myWriteAPIKeyString;                                                         //channel write key
String bluetoothstoreString = "";

//variables needed for functions

char bluetoothChar;
char ssid[] = "";
char pass[] = "";
char myWriteAPIKey[17];
const char* myWriteAPIKeyPointer = myWriteAPIKey;

unsigned long myChannelNumberInt;
int bluetoothInt;
int breakFlag = 0;
int number = 0;
int deliminatorCount = 0;

BluetoothSerial SerialBT;
WiFiClient  client;

void setup() {
  //initializing Serial, Bluetooth, WiFi and ThingSpeak instances
  Serial.begin(115200);
  SerialBT.begin("UpRight Device");
  WiFi.mode(WIFI_STA);   
  ThingSpeak.begin(client);

  Serial.println("The device started, now you can pair it with bluetooth!");

  //getting strings from bluetooth connection
  
  while(!SerialBT.available())                                                    //while bluetooth device is not connected
  {
    delay(2000);
    Serial.println("Waiting for bluetooth connection");
    while(SerialBT.available())                                                   //when bluetooth device is connected
    {
      bluetoothChar = SerialBT.read();
      bluetoothInt = bluetoothChar;
      bluetoothstoreString = bluetoothstoreString + bluetoothChar;
      Serial.println(bluetoothChar);                                              //read serial input of from bluetooth feed
      
      //storing in substrings
      if(deliminatorCount == 0)                                                   //no deliminators have been found, deliminator is '+' character
      {
        if(bluetoothInt != 43)                                                    //only add to ssidstring if character is not deliminator
          ssidString = ssidString + bluetoothChar; 
        }
        
      if(deliminatorCount == 1)                                                   //first deliminator has been found
      {
        if(bluetoothInt != 43)
          passString = passString + bluetoothChar;
        }

      if(deliminatorCount == 2)                                                   //second deliminator has been found
      { 
        if(bluetoothInt != 43)
          myChannelNumberString = myChannelNumberString + bluetoothChar;
        }

      if(deliminatorCount == 3)                                                   //third and last deliminator has been found
      {
        if(bluetoothInt != 35)                                                    //only add to string if character is not '#' (terminating character for bluetooth read)
          myWriteAPIKeyString = myWriteAPIKeyString + bluetoothChar;
        }

      if(bluetoothInt == 43)                                                      //ascii value for '+', chosen deliminator for substrings
      {
        deliminatorCount++;
        }
        
      if(bluetoothInt == 35)                                                      //ascii value for '#' is 35
      {
        Serial.println("End of input");
        breakFlag = 1;
        break;
        }
      }
    if(breakFlag == 1)
    {
      Serial.println("Finished bluetooth read");
      SerialBT.end();                                                             //ending bluetooth communication
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
      WiFi.begin(ssid, pass);                                                     // Connect to WPA/WPA2 network
      Serial.print(".");
      delay(5000);     
    } 
    Serial.println("\nConnected.");
  }

  myChannelNumberInt = myChannelNumberString.toInt();                           //myChannelNumberString to myChannelNumberInt                         
  
  for(int i = 0; i < 16; i++)                                                   //myWriteAPIKeyString to myWriteAPIKey char array
    myWriteAPIKey[i] = myWriteAPIKeyString.charAt(i);
  
}

void loop() {

  // Writing to ThingSpeak channel
  int x = ThingSpeak.writeField(myChannelNumberInt, 1, number, myWriteAPIKeyPointer);
  if(x == 200)
    Serial.println("Channel update successful.");
  else
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  
  delay(20000);                                                               // Wait 20 seconds to update the channel again
  
  //For DEMO, will change to after control system is installed
  if(number == 1)
    number = 0;
  else
    number = 1;
}
