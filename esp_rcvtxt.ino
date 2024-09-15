#include <SoftwareSerial.h>
#include "ThingSpeak.h" //install library for thing speak
#include <ESP8266WiFi.h>

#include <Wire.h>
#include <string.h>


char ssid[] = "Meenakshi";        // your network SSID (name) 
char pass[] = "8589019062";    // your network password
WiFiClient  client;

unsigned long myChannelNumber = 2499651;
const char * myWriteAPIKey = "6ADE4FF4PG10A1GM";

const byte RX = D6;
const byte TX = D7;
SoftwareSerial mySerial = SoftwareSerial(RX, TX);

String myStatus = "";
long lastUART = 0;
int val = 0;
void Read_Uart();    // UART STM
String LED1 = "OFF", LED2 = "OFF";
void setup()
{
  Serial.begin(115200);
  mySerial.begin(115200);

    if(WiFi.status() != WL_CONNECTED)
    {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    while(WiFi.status() != WL_CONNECTED)
    {
      WiFi.begin(ssid, pass);  // Connect to WPA/WPA2 network. Change this line if using open or WEP network
      Serial.print(".");
      delay(5000);     
    } 
    Serial.println("\nConnected.");
    }
  ThingSpeak.begin(client);  // Initialize ThingSpeak

  Wire.begin();

  Serial.println("UART Start");

  lastUART = millis();
}
void loop()
{
  Read_Uart();
  if (millis() - lastUART > 1000)
  {
    // mySerial.print("1ON2ON3OFF4");
    // Serial.println("Truyen : 1ON2ON3OFF4");
    lastUART = millis();
  }

  
  ThingSpeak.setField(1, val);
  ThingSpeak.setStatus(myStatus);
  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
}
void Read_Uart()
{
  String st = "";

  
  while (mySerial.available())
  {  
    char inChar = (char)mySerial.read();
    st +=  inChar;
     String inString = ""; 

    if (inChar == 'p')
    {
      Serial.println("Data : " + st);
      if(st.substring(0,2) == "0V")
    {
      val = 100;
    }

    else
    {
      inString = st.substring(8,14);
      val = inString.toInt();
    }
    Serial.print("Soc : ");
    Serial.println(val);
    
  //     break;
 
    }



  }

}