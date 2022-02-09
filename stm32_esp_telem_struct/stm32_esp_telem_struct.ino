#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <SoftwareSerial.h>
#include "TelemData.h"

#define WIFI_SSID "UAV"
#define WIFI_PASS "A9A997B60FE11"

IPAddress server_ip(192, 168, 1, 45);
#define UDP_PORT 9000

//struct telem_pack from TelemData.h
bool received;
WiFiUDP UDP;
SoftwareSerial stmSerial(13, 15, false, 256);
struct telem_pack telem;

char test_buf[sizeof(telem)];
char buf[sizeof(struct telem_pack)];
int char_index;

void setup() {

  Serial.begin(115200);
  stmSerial.begin(9600);
  // Begin WiFi
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
   
  // Connecting to WiFi...
  Serial.print("Connecting to ");
  Serial.print(WIFI_SSID);
  // Loop continuously while WiFi is not connected
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
    Serial.print(".");
  }
   
  // Connected to WiFi
  Serial.println();
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());
  memcpy(test_buf, &telem , sizeof(telem));

}

void loop() {
  serialEvent();

}
