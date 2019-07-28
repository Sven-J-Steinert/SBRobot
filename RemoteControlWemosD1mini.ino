#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoNunchuk.h>

const char* ssid ="Robot_02";
const char* pass ="letmeaccessyourdata";

void initNetwork();

unsigned int localPort = 2000; // local port to listen for UDP packets

IPAddress ServerIP(192,168,4,255);
//IPAddress ServerIP(192,168,43,1);
//IPAddress ClientIP(192,168,4,2);
 
// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;
int AnalogY,AnalogX;
char packetBuffer[1];   //Where we get the UDP data
ArduinoNunchuk nunchuk = ArduinoNunchuk();

void setup()
{
  Serial.begin(115200);
    Serial.println();
 
  initNetwork();
  
  
  //Start UDP
  Serial.println("Starting UDP");
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(udp.localPort());

   nunchuk.init();//D1 = SDA D2 = SCL
}

void loop()
{
  nunchuk.update();
  AnalogX = map(analogRead(A0),3,1024,0,255);
  Serial.print(AnalogX, DEC);
  Serial.print(' ');
  AnalogY = map(nunchuk.analogX,3,252,0,255);
  Serial.print(AnalogY, DEC);
  Serial.print(' ');
 // Serial.print(' ');
 // Serial.print(nunchuk.analogY, DEC);
 // Serial.print(' ');
 // Serial.print(nunchuk.accelX, DEC);
 // Serial.print(' ');
 // Serial.print(nunchuk.accelY, DEC);
 // Serial.print(' ');
 // Serial.print(nunchuk.accelZ, DEC);
 // Serial.print(' ');
 // Serial.print(nunchuk.zButton, DEC);
 // Serial.print(' ');
 // Serial.println(nunchuk.cButton, DEC);

char sensorBuffer;
if ((AnalogX == 133) && (AnalogY == 128)){sensorBuffer = '0' ;}
if ((AnalogX == 133) && (AnalogY > 150)){sensorBuffer = '4' ;}
if ((AnalogX > 150) && (AnalogY == 128)){sensorBuffer = '1' ;}
if ((AnalogX == 133) && (AnalogY < 100)){sensorBuffer = '2' ;}
if ((AnalogX < 100) && (AnalogY == 128)){sensorBuffer = '3' ;}
if ((AnalogX > 180) && (AnalogY  > 180)){sensorBuffer = '8' ;}
if ((AnalogX > 180) && (AnalogY  < 70)){sensorBuffer = '5' ;}
if ((AnalogX < 70) && (AnalogY  < 70)){sensorBuffer = '6' ;}
if ((AnalogX < 70) && (AnalogY  > 180)){sensorBuffer = '7' ;}
//if ((nunchuk.cButton) ){sensorBuffer = 'c' ;}
//if ((nunchuk.zButton)){sensorBuffer = 'z' ;}

Serial.println(sensorBuffer);
 udp.beginPacket(ServerIP, 2000);
 udp.write(sensorBuffer);
 udp.endPacket();
 udp.flush();

delay(50);
}

void initNetwork()
{
  WiFi.softAP(ssid, pass);
  Serial.println(udp.begin(localPort));
  Serial.println(WiFi.localIP());
}
