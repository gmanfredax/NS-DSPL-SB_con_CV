void printUsage() {
  Serial.println(F("Comandi disponibili:")); 
  Serial.println(F("SENSOR <ioPin>: Imposta ioPin come un SENSORE"));
  Serial.println(F("SENSORTOF <ioPin>: Imposta ioPin come un SENSORE ToF")); 
  Serial.println(F("SENSORIR <ioPin>: Imposta l'IR RX sull'ioPin (solo 8 ammesso)"));
  Serial.println(F("SENSORABS <ioPin>: Imposta ioPin come un SENSORE ASSORBIMENTO"));
  Serial.println(F("MSENSOR <ioPin>: Imposta ioPin come un SENSORE MOMENTANEO"));  
  Serial.println(F("ABILITA <ioPin>: Abilita ioPin"));  
  Serial.println(F("DISABILITA <ioPin>: Disabilita ioPin"));    
  Serial.println(F("ADDRESS <ioPin> <dccAddress>: Imposta dccAddress per il pin ioPin"));
  Serial.println(F("DEBUG <ON|OFF>: Abilita o disabilita i messaggi di debug su seriale"));
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
    }
  }

  // SENSOR IR
  else if(strcmp(command, "SENSORIR") == 0) {

    int ioPin = atoi(strtok(NULL, " "));
    if(ioPin < 1 || ioPin > IOPINS) {
      Serial.print(F("Invalid ioPin: "));
      Serial.println(ioPin);
      Serial.println();
    }
    else if (ioPin != IR_UNIT_IO) {
      Serial.println(F("Il sensore IR può essere configurato solo sul canale 8: comando ignorato."));
      Serial.println();
    }
    else {
      uint8_t pincv = ((ioPin-1) * 4) + 21;
      sensorInfo[ioPin - 1].deviceType = DEV_SENSORIR;
      LocoNetSV.writeSVStorage(pincv, DEV_SENSORIR);
    }
  }

  // SENSOR ASSORBIMENTO
  else if(strcmp(command, "SENSORABS") == 0) {

    int ioPin = atoi(strtok(NULL, " "));
    if(ioPin < 1 || ioPin > IOPINS) {
      Serial.print(F("Invalid ioPin: "));
      Serial.println(ioPin);
      Serial.println();
    }
    else {
      uint8_t pincv = ((ioPin-1) * 4) + 21;
      sensorInfo[ioPin - 1].deviceType = DEV_SENSORABS;
      LocoNetSV.writeSVStorage(pincv, DEV_SENSORABS);
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
        LocoNetSV.writeSVStorage(pincv, address);
      }
    }    
  }

  // DEBUG
  else if(strcmp(command, "DEBUG") == 0) {

    char *state = strtok(NULL, " ");
    if (state == NULL) {
      Serial.println(F("Uso: DEBUG ON|OFF"));
    } else {
      if(strcmp(state, "ON") == 0 || strcmp(state, "1") == 0) {
        debugEnabled = true;
      } else if(strcmp(state, "OFF") == 0 || strcmp(state, "0") == 0) {
        debugEnabled = false;
      } else {
        Serial.println(F("Valore non valido. Usa ON oppure OFF."));
        cmdlinepos = 0;
        return;
      }
      LocoNetSV.writeSVStorage(SV_ADDR_DEBUG_FLAG, debugEnabled ? 1 : 0);
      Serial.print(F("Debug seriale: "));
      Serial.println(debugEnabled ? F("ABILITATO") : F("DISABILITATO"));
    }
  }

  // STATUS
  else if(strcmp(command, "STATUS") == 0) printConfiguration();

  // RESET
  else if(strcmp(command, "RESET") == 0) {    
    setFactoryDefault();
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
