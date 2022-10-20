#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>

#include <SD.h>
#include <PubSubClient.h>

//#include <SoftwareSerial.h>
#include "TelemData.h"


#define WIFI_SSID "Ecospark_2.4GHz"
#define WIFI_PASS "Jasperkid1213"

#define UAV1
/*
  #define WIFI_SSID "Moral"
  #define WIFI_PASS "Jasperkid1213"

*/
//LAB
IPAddress server_ip(192, 168, 1, 37);

//Phone
//IPAddress server_ip(192, 168, 43, 152);

//Home
//IPAddress server_ip(192, 168, 1, 103);

#ifdef UAV1
#define UDP_PORT 9000
#endif

#ifdef UAV2
#define UDP_PORT 9001
#endif

#define CS_PIN  D8
#define SD_BUFFER_SIZE 1024

char inChar, inChar_;
//struct telem_pack from TelemData.h
bool received, file_closed;
bool send_mqtt;
WiFiUDP UDP;
//SoftwareSerial(rxPin, txPin, inverse_logic)
//SoftwareSerial stmSerial(13, 15, false, 256);
struct telem_pack telem;

char test_buf[sizeof(telem)];
char buf[1024];
char buf2send[1024];
char log_buf[SD_BUFFER_SIZE];
int char_index;
bool led_state, armed = true;
unsigned int sd_counter, end_counter;
unsigned long sd_time, udp_time, sd_send_time;

struct swarm_pack swarm_pack;

size_t offset_log;

//File dataFile;

//MQTT
const char *mqtt_broker = "192.168.1.37"; // Enter your WiFi or Ethernet IP
const char *topic_ch = "ch";
const char *topic_rem = "remote";
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
      client.subscribe(topic_ch);
      //client.subscribe(topic_rem);
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


/*

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

  if (!strcmp(topic, topic_ch)) {
    memcpy(&swarm_pack.pwm2.w1, rec_buf, sizeof(struct pwm));
    char send_buf[sizeof(struct swarm_pack)];
    memcpy(send_buf, &swarm_pack, sizeof(struct swarm_pack));
    Serial.write(send_buf, sizeof(struct swarm_pack));
    // Serial.println(swarm_pack.pwm2.w1);
  }

  else if (!strcmp(topic, topic_rem)) {
    // Serial.println(swarm_pack.ch.ch1);
    memcpy(&swarm_pack.ch.ch1, rec_buf, sizeof(struct ch));
  }




  //Serial.println(ch_deb.ch1);
  //Serial.write(ch1_buf, sizeof(uint16_t));

  //  Serial.println();
  //  Serial.println(" - - - - - - - - - - - -");
} */

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
  char send_buf[sizeof(struct pwm)];

  memcpy(send_buf, rec_buf, sizeof(struct pwm));

  Serial.write(send_buf, sizeof(struct pwm));

  //Serial.println(ch_deb.ch1);
  //Serial.write(ch1_buf, sizeof(uint16_t));

  //  Serial.println();
  //  Serial.println(" - - - - - - - - - - - -");
}
