void ToggleLED() {
  switch(led_state) {
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

void clear_buffer (void)
{
  for (int i=0; i<SD_BUFFER_SIZE; i++) log_buf[i] = '\0';
}

void serialEvent() {
  


  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    buf[char_index] = inChar;
    char_index++;
    //Serial.print(inChar);
    if (char_index == sizeof(struct telem_pack)) {
        //Serial.println();
        char_index = 0; 
        memcpy(&telem, &buf , sizeof(buf));
       // Serial.print("buf_size:"); Serial.println(sizeof(buf));
      //  Serial.print("telem_size:"); Serial.println(sizeof(struct telem_pack));

        sendUDP();
        writeSD();

    }
  } 

  
}

void writeSD() {
  if(millis() - sd_time < 20000) {

  if(millis() - sd_send_time > 100) {
    sd_send_time = millis();
       if (dataFile) {
                ToggleLED();
                dataFile.print(telem.attitude.pitch);   dataFile.print("||");  dataFile.println(millis());  

          }
  }
           

  }

   else if(!file_closed) {
      
      file_closed = true;
      dataFile.close();

   }
           
}


void sendUDP() {
          if(millis() - udp_time > 100) {
            udp_time = millis();
  
            UDP.beginPacket(server_ip, UDP_PORT);
            UDP.write(buf,sizeof(struct telem_pack));
            UDP.endPacket();

         }
   //        
}
