#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>

#include <SD.h>

//#include <SoftwareSerial.h>
#include "TelemData.h"

/*
#define WIFI_SSID "UAV"
#define WIFI_PASS "A9A997B60FE11"
*/


#define WIFI_SSID "Ecospark_2.4GHz"
#define WIFI_PASS "Jasperkid1213"


//LAB
IPAddress server_ip(192, 168, 1, 37);
//192.168.43.152

//Phone
//IPAddress server_ip(192, 168, 43, 152);

//Home
//IPAddress server_ip(192, 168, 1, 103);
#define UDP_PORT 9000
#define CS_PIN  D8
#define SD_BUFFER_SIZE 1024

//struct telem_pack from TelemData.h
bool received,file_closed;
WiFiUDP UDP;
//SoftwareSerial(rxPin, txPin, inverse_logic)
//SoftwareSerial stmSerial(13, 15, false, 256);
struct telem_pack telem;

char test_buf[sizeof(telem)];
char buf[sizeof(struct telem_pack)];
char log_buf[SD_BUFFER_SIZE];
int char_index;
bool led_state, armed=true;
unsigned int sd_counter, end_counter;
unsigned long sd_time, udp_time, sd_send_time;

size_t offset_log;
char inChar;
File dataFile;
void setup() {
  Serial.begin(1000000);
  //stmSerial.begin(9600);
  // Begin WiFi
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
   
  // Connecting to WiFi...
  //Serial.print("Connecting to ");
  //Serial.print(WIFI_SSID);
  // Loop continuously while WiFi is not connected
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
 //   Serial.print(".");
  }


   //SD.begin(CS_PIN);
   //dataFile = SD.open("LOG.txt", FILE_WRITE);

 
   
  // Connected to WiFi
  //Serial.println();
  //Serial.print("Connected! IP address: ");
  //Serial.println(WiFi.localIP());
  //memcpy(test_buf, &telem , sizeof(telem));
  //sd_time = millis();
}

void loop() {
 /* if (millis() - sd_time > 30000) { 
      armed = false;
    }*/
  
  if(armed) {
      serialEvent();
  }
/*
  else if(!file_closed) {
    dataFile.close();
    file_closed = true; 
  }*/

  else {
    ToggleLED_1s();
  }

  //writeSD();
}
