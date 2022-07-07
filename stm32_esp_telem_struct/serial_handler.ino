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
  memcpy(log_buf, log_buf+chunkSize, sizeof(log_buf)-chunkSize);
}

void serialEvent() {



  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    if (inChar == '@') {
      end_counter++;
      if (end_counter < 3) {
        if (end_counter == 2) {
          char_index = 0;
          end_counter = 0;
          break;
        }
      }
    }

    else {
      end_counter = 0;
    }
    buf[char_index] = inChar;
    char_index++;
    //Serial.print(inChar);
    if (char_index == sizeof(struct telem_pack)) {
      //Serial.println();
      char_index = 0;

      offset_log += sizeof(struct telem_pack);
      memcpy(&telem, buf , sizeof(buf));
      // Serial.print("buf_size:"); Serial.println(sizeof(buf));
      //  Serial.print("telem_size:"); Serial.println(sizeof(struct telem_pack));
      //memcpy(log_buf + offset_log, buf, sizeof(buf));

      sendUDP();
      //writeSD();
    }
  }


}

void writeSD() {
  

      if (dataFile) {
        unsigned int chunkSize = dataFile.availableForWrite();

        // if(millis() - sd_send_time > 100) {
        //   sd_send_time = millis();
        if (chunkSize && offset_log >= chunkSize) {
          ToggleLED();
          //dataFile.print(telem.attitude.pitch);   dataFile.print("||");  dataFile.println(millis());
          unsigned int log_buf_size = offset_log - sizeof(struct telem_pack);
          dataFile.write(log_buf, chunkSize);
          offset_log = 0;
          clear_buffer(chunkSize);

        }
      }

      // }

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
