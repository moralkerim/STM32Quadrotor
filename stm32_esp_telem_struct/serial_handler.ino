void ToggleLED() {
  switch (led_state) {
    case 0:
      digitalWrite(LED_BUILTIN, HIGH);
      led_state = 1;
      break;

    case 1:
      digitalWrite(LED_BUILTIN, LOW);
      led_state = 0;
      break;


  }
}

void ToggleLED_1s() {
  switch (led_state) {
    case 0:
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      led_state = 1;
      break;

    case 1:
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
      led_state = 0;
      break;


  }
}

void clear_buffer (unsigned int chunkSize)
{
  memcpy(log_buf, log_buf + chunkSize, sizeof(log_buf) - chunkSize);
}

void serialEvent() {



  while (Serial.available())
  {
    char inChar_ = inChar;
    inChar = (char)Serial.read();
    //client.publish(topic_deb, &inChar);
    if (inChar == 0x01 && inChar_ == 0x04) {  //Start char received


      memcpy(&telem, buf , sizeof(struct telem_pack));
      //memcpy(buf2send, buf , sizeof(struct telem_pack));
      sendUDP();

      char_index = 0;
      inChar_ = inChar;
      inChar = (char)Serial.read();
      ToggleLED();

    }



    buf[char_index] = inChar;
    char_index++;


  }

}

void sendUDP() {
//  if (millis() - udp_time > 20) {
//    udp_time = millis();
    ToggleLED();
    UDP.beginPacket(server_ip, UDP_PORT);
    UDP.write(buf, sizeof(struct telem_pack));
    UDP.endPacket();

//  }
  //
}
