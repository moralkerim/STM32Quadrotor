void serialEvent() {
  


  while (stmSerial.available())
  {
    char inChar = (char)stmSerial.read();
    buf[char_index] = inChar;
    char_index++;
    Serial.print(inChar);
    if (char_index == sizeof(struct telem_pack)) {
        Serial.println();
        char_index = 0; 

       // Serial.print("buf_size:"); Serial.println(sizeof(buf));
      //  Serial.print("telem_size:"); Serial.println(sizeof(struct telem_pack));

        UDP.beginPacket(server_ip, UDP_PORT);
        UDP.write(buf,sizeof(struct telem_pack));
        UDP.endPacket();

    }
  } 

  
}
