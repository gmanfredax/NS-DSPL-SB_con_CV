void printUsage() {
  Serial.println(F("Comandi disponibili:")); 
  Serial.println(F("SENSOR <ioPin>: Imposta ioPin come un SENSORE"));
  Serial.println(F("SENSORTOF <ioPin>: Imposta ioPin come un SENSORE ToF")); 
  Serial.println(F("MSENSOR <ioPin>: Imposta ioPin come un SENSORE MOMENTANEO"));  
  Serial.println(F("SENSORIR <ioPin>: Imposta ioPin come un SENSORE IR (beacon / LISSY)"));
  Serial.println(F("ABILITA <ioPin>: Abilita ioPin"));  
  Serial.println(F("DISABILITA <ioPin>: Disabilita ioPin"));    
  Serial.println(F("ADDRESS <ioPin> <dccAddress>: Imposta dccAddress per il pin ioPin"));
  Serial.println(F("STATUS: Stampa lo stato attuale"));
  Serial.println(F("RESET: Ripristina alle impostazioni di fabbrica"));
  Serial.println(F("REBOOT: Riavvia il sistema"));
  Serial.println(F("HELP: Stampa questo messaggio"));
  Serial.println();
}

void parseCmdLine() {

  // empty line, print usage
  if(cmdlinepos == 0) {
    printUsage();
    return;
  }

  // get the command 
  char *command = strtok(cmdline, " ");

  // SENSOR
  if(strcmp(command, "SENSOR") == 0) {

    int ioPin = atoi(strtok(NULL, " "));
    if(ioPin < 1 || ioPin > IOPINS) {
      Serial.print(F("Invalid ioPin: ")); 
      Serial.println(ioPin);
      Serial.println();
    }
    else {
      uint8_t pincv = ((ioPin-1) * 4) + 21;
      sensorInfo[ioPin - 1].deviceType = DEV_SENSOR;
      LocoNetSV.writeSVStorage(pincv, DEV_SENSOR);
      //saveConfigToEEPROM();
    }    
  }

  // SENSOR ToF
  else if(strcmp(command, "SENSORTOF") == 0) {

    int ioPin = atoi(strtok(NULL, " "));
    if(ioPin < 1 || ioPin > IOPINS) {
      Serial.print(F("Invalid ioPin: ")); 
      Serial.println(ioPin);
      Serial.println();
    }
    else {
      uint8_t pincv = ((ioPin-1) * 4) + 21;
      sensorInfo[ioPin - 1].deviceType = DEV_SENSORTOF;
      LocoNetSV.writeSVStorage(pincv, DEV_SENSORTOF);
      //saveConfigToEEPROM();
    }    
  }


  // MSENSOR
  else if(strcmp(command, "MSENSOR") == 0) {

    int ioPin = atoi(strtok(NULL, " "));
    if(ioPin < 1 || ioPin > IOPINS) {
     Serial.print("Invalid ioPin: "); 
      Serial.println(ioPin);
      Serial.println();
    }
    else {
      uint8_t pincv = ((ioPin-1) * 4) + 21;
      sensorInfo[ioPin - 1].deviceType = DEV_MSENSOR;
      LocoNetSV.writeSVStorage(pincv, DEV_MSENSOR);
      //saveConfigToEEPROM();
    }    
  }

  // SENSORIR (beacon IR / LISSY)
  else if(strcmp(command, "SENSORIR") == 0) {

    int ioPin = atoi(strtok(NULL, " "));
    if(ioPin < 1 || ioPin > IOPINS) {
      Serial.print(F("Invalid ioPin: ")); 
      Serial.println(ioPin);
      Serial.println();
    }
    else {
      uint8_t pincv = ((ioPin-1) * 4) + 21;
      sensorInfo[ioPin - 1].deviceType = DEV_SENSORIR;
      LocoNetSV.writeSVStorage(pincv, DEV_SENSORIR);
      //saveConfigToEEPROM();
    }    
  }


  // ABILITA
  else if(strcmp(command, "ABILITA") == 0) {

    int ioPin = atoi(strtok(NULL, " "));
    if(ioPin < 1 || ioPin > IOPINS) {
      Serial.print(F("Invalid ioPin: ")); 
      Serial.println(ioPin);
      Serial.println();
    }
    else {
      uint8_t pincv = ((ioPin-1) * 4) + 23;
      sensorInfo[ioPin - 1].isEnabled = true;
      LocoNetSV.writeSVStorage(pincv, 1);
      // servo1PLIV.attach(5);
      // servo2PLIV.attach(9);
      //saveConfigToEEPROM();
    }    
  }

  // DISABILITA
  else if(strcmp(command, "DISABILITA") == 0) {

    int ioPin = atoi(strtok(NULL, " "));
    if(ioPin < 1 || ioPin > IOPINS) {
      Serial.print(F("Invalid ioPin: ")); 
      Serial.println(ioPin);
      Serial.println();
    }
    else {
      uint8_t pincv = ((ioPin-1) * 4) + 23;
      sensorInfo[ioPin - 1].isEnabled = false;
      LocoNetSV.writeSVStorage(pincv, 0);
      // servo1PLIV.detach();
      // servo2PLIV.detach();
      //saveConfigToEEPROM();
    }    
  }

  // ADDRESS
  else if(strcmp(command, "ADDRESS") == 0) {

    int ioPin = atoi(strtok(NULL, " "));
    if(ioPin < 1 || ioPin > IOPINS) {
      Serial.print(F("Invalid ioPin: ")); 
      Serial.println(ioPin);
      Serial.println();
    } else {
      int address = atoi(strtok(NULL, " "));
      uint8_t adrLo = LocoNetSV.readSVStorage(SV_ADDR_CHANGE_ID_L) ;
      uint8_t adrHi = LocoNetSV.readSVStorage(SV_ADDR_CHANGE_ID_H) ;
      address = ((adrHi * 256 + adrLo) * 10) + address;
      if(address < 1 || address > 2048) {
        Serial.print(F("Invalid address: ")); 
        Serial.println(address);
        Serial.println();        
      } else {
        uint8_t pincv = ((ioPin-1) * 4) + 20;
        sensorInfo[ioPin - 1].dccAddress = address;
        //saveConfigToEEPROM();
        LocoNetSV.writeSVStorage(pincv, address);
      }
    }    
  }

  // STATUS
  else if(strcmp(command, "STATUS") == 0) printConfiguration();

  // RESET
  else if(strcmp(command, "RESET") == 0) {
    
    setFactoryDefault();
    //resetConfiguration();
    //saveConfigToEEPROM();
  }

  // REBOOT
  else if(strcmp(command, "REBOOT") == 0) softwareReset(WDTO_60MS);

  // HELP
  else if(strcmp(command, "HELP") == 0) printUsage();

  // Unknown
  else {
    Serial.print(F("Comando sconosciuto: '"));
    Serial.print(command);
    Serial.println("'");
  }
  
  cmdlinepos = 0;
}
