#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>

#include <SD.h>

//#include <SoftwareSerial.h>
#include "TelemData.h"

#define WIFI_SSID "UAV"
#define WIFI_PASS "A9A997B60FE11"

IPAddress server_ip(192, 168, 1, 41);
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
bool led_state;
unsigned int sd_counter;
unsigned long sd_time, udp_time, sd_send_time;

File dataFile;
void setup() {
  Serial.begin(250000);
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


   SD.begin(CS_PIN);
   dataFile = SD.open("LOG.txt", FILE_WRITE);

 
   
  // Connected to WiFi
  //Serial.println();
  //Serial.print("Connected! IP address: ");
  //Serial.println(WiFi.localIP());
  //memcpy(test_buf, &telem , sizeof(telem));
  sd_time = millis();
}

void loop() {
  serialEvent();
  //writeSD();
}
