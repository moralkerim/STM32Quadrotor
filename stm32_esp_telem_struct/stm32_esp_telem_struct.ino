#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>

#include <SD.h>
#include <PubSubClient.h>

//#include <SoftwareSerial.h>
#include "TelemData.h"


#define WIFI_SSID "UAV"
#define WIFI_PASS "A9A997B60FE11"

#define UAV2
/*
  #define WIFI_SSID "Moral"
  #define WIFI_PASS "Jasperkid1213"
*/

//LAB
IPAddress server_ip(192, 168, 1, 41);
//192.168.43.152

//Phone
//IPAddress server_ip(192, 168, 43, 152);

//Home
//IPAddress server_ip(192, 168, 1, 103);
#define UDP_PORT 9000
#define CS_PIN  D8
#define SD_BUFFER_SIZE 1024

//struct telem_pack from TelemData.h
bool received, file_closed;
bool send_mqtt;
WiFiUDP UDP;
//SoftwareSerial(rxPin, txPin, inverse_logic)
//SoftwareSerial stmSerial(13, 15, false, 256);
struct telem_pack telem;

char test_buf[sizeof(telem)];
char buf[sizeof(struct telem_pack)];
char log_buf[SD_BUFFER_SIZE];
int char_index;
bool led_state, armed = true;
unsigned int sd_counter, end_counter;
unsigned long sd_time, udp_time, sd_send_time;

size_t offset_log;

//File dataFile;

//MQTT
const char *mqtt_broker = "192.168.1.41"; // Enter your WiFi or Ethernet IP
const char *topic = "ch";
const char *topic_deb = "debug";
const int mqtt_port = 1883;
WiFiClient espClient;
PubSubClient client(espClient);

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

  //connecting to a mqtt broker
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);

  while (!client.connected()) {
#ifdef UAV1
    String client_id = "UAV-1";
#endif

#ifdef UAV2
    String client_id = "UAV-2";
#endif

    if (client.connect(client_id.c_str())) {
#ifdef UAV2
      client.subscribe(topic);
#endif

      //Serial.println("Public emqx mqtt broker connected");
    } else {
      //Serial.print("failed with state ");
      //Serial.print(client.state());
      delay(2000);
    }
  }

}

void loop() {
  /* if (millis() - sd_time > 30000) {
       armed = false;
     }*/

#ifdef UAV1
  if (armed) {
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
#endif
  client.loop();

  //writeSD();
}


void callback(char *topic, uint8_t *payload, unsigned int length) {
  //Serial.print("Message arrived in topic: ");
  // Serial.println(topic);
  // Serial.print("Message:");

  char rec_buf[length + 1];


  for (int i = 0; i < length; i++) {
    //Serial.print((char) payload[i]);
    rec_buf[i] = (char)payload[i];
  }
  //Serial.println();
  //Serial.println(rec_buf);
  //float roll_des = atof(rec_buf);
  char send_buf[sizeof(struct ch)];
  char ch1_buf[sizeof(uint16_t)];
  struct ch ch_deb;
  
  memcpy(send_buf, rec_buf, sizeof(struct ch));
  memcpy(&ch_deb, send_buf, sizeof(struct ch));

  Serial.write(send_buf, sizeof(struct ch));
  char deb_buf[10];

  itoa (ch_deb.ch1, deb_buf, 10 );
  memcpy(ch1_buf, &telem.ch.ch1, sizeof(uint16_t));
  client.publish(topic_deb, deb_buf);
  //Serial.println(ch_deb.ch1);
  //Serial.write(ch1_buf, sizeof(uint16_t));

  //  Serial.println();
  //  Serial.println(" - - - - - - - - - - - -");
}
