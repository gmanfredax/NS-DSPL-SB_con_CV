void softwareReset( uint8_t prescaller) {
  // start watchdog with the provided prescaller
  wdt_enable( prescaller);
  // wait for the prescaller time to expire
  // without sending the reset signal by using
  // the wdt_reset() method
  while(1) {}
}

// restore the configuration from EEPROM
void readConfigFromStorage() {

  int currentAddress = 20;
  for(int i = 0; i < pinCount; i++) {

    sensorInfo[i].dccAddress = LocoNetSV.readSVStorage(currentAddress);
    sensorInfo[i].deviceType = LocoNetSV.readSVStorage(currentAddress+1);
    sensorInfo[i].sensdistance = LocoNetSV.readSVStorage(currentAddress+2);
    sensorInfo[i].isEnabled = LocoNetSV.readSVStorage(currentAddress+3);
    if (sensorInfo[i].deviceType == DEV_SENSORIR && i != IR_UNIT_INDEX) {
      Serial.print(F("I/O "));
      Serial.print(i + 1);
      Serial.println(F(" non può essere configurato come sensore IR: forzatura su DISABILITATO"));
      sensorInfo[i].deviceType = DEV_SENSOR;
      sensorInfo[i].isEnabled = 0;
      LocoNetSV.writeSVStorage(currentAddress+1, DEV_SENSOR);
      LocoNetSV.writeSVStorage(currentAddress+3, 0);
    }
    currentAddress += sizeof(SensorInfo);
  }

  Serial.println(("- Configurazione caricata dalla EEPROM"));
}

void notifySVChanged(uint16_t Offset) {

  if (Offset >= SV_ADDR_SENSOR1_LNADR and Offset <= SV_ADDR_SENSOR8_SENSDISTANCE) {
    //sensorsNumber = LocoNetSV.readSVStorage(SV_ADDR_NUM_SENSORS) ;
    Serial.print("CV "); Serial.print(Offset); Serial.print(" aggiornata. Nuovo valore: "); Serial.println(LocoNetSV.readSVStorage(Offset));
    readConfigFromStorage();
    softwareReset(WDTO_60MS);
  } else if (Offset == SV_ADDR_CHANGE_ID_L) {
    Serial.print("CV "); Serial.print(Offset); Serial.print(" aggiornata. Nuovo valore: "); Serial.println(LocoNetSV.readSVStorage(Offset));
    changeSensorAddress();
    readConfigFromStorage();
    softwareReset(WDTO_60MS);
  }
}

void changeSensorAddress() {
  uint8_t adrLo = LocoNetSV.readSVStorage(SV_ADDR_CHANGE_ID_L) ;
  uint8_t adrHi = LocoNetSV.readSVStorage(SV_ADDR_CHANGE_ID_H) ;
  uint8_t address = ((adrHi * 256 + adrLo) * 10);

  EEPROM.put(SV_ADDR_SENSOR1_LNADR-2, address + 1);
  EEPROM.put(SV_ADDR_SENSOR2_LNADR-2, address + 2);
  EEPROM.put(SV_ADDR_SENSOR3_LNADR-2, address + 3);
  EEPROM.put(SV_ADDR_SENSOR4_LNADR-2, address + 4);
  EEPROM.put(SV_ADDR_SENSOR5_LNADR-2, address + 5);
  EEPROM.put(SV_ADDR_SENSOR6_LNADR-2, address + 6);
  EEPROM.put(SV_ADDR_SENSOR7_LNADR-2, address + 7);
  EEPROM.put(SV_ADDR_SENSOR8_LNADR-2, address + 8);
}

void setFactoryDefault() {

  EEPROM.put(SV_ADDR_NODE_ID_L-2, VALUE_NODE_ID_L) ;
  EEPROM.put(SV_ADDR_NODE_ID_H-2, VALUE_NODE_ID_H) ;

  EEPROM.put(SV_ADDR_SERIAL_NUMBER_L-2, VALUE_SERIAL_NUMBER_L) ;
  EEPROM.put(SV_ADDR_SERIAL_NUMBER_H-2, VALUE_SERIAL_NUMBER_H) ;

  EEPROM.put(SV_ADDR_MANUFACTURER_ID-2, VALUE_MANUFACTURER_ID) ;
  EEPROM.put(SV_ADDR_DEVELOPER_ID-2, VALUE_DEVELOPER_ID) ;
  
  EEPROM.put(SV_ADDR_PRODUCT_ID_L-2, VALUE_PRODUCT_ID_L) ;
  EEPROM.put(SV_ADDR_PRODUCT_ID_H-2, VALUE_PRODUCT_ID_H) ;
  
  //EEPROM.put(SV_ADDR_NUM_SENSORS, VALUE_NUM_SENSORS) ;
  
  EEPROM.put(SV_ADDR_ADDR_SENSORS_L-2, VALUE_ADDR_SENSORS_L) ;
  EEPROM.put(SV_ADDR_ADDR_SENSORS_H-2, VALUE_ADDR_SENSORS_H) ;

  EEPROM.put(SV_ADDR_CHANGE_ID_L-2, VALUE_CHANGE_ID_L) ;
  EEPROM.put(SV_ADDR_CHANGE_ID_H-2, VALUE_CHANGE_ID_H) ;

  EEPROM.put(SV_ADDR_AUXILIARY-2, VALUE_AUXILIARY) ;

  uint8_t adrLo = LocoNetSV.readSVStorage(SV_ADDR_CHANGE_ID_L) ;
  uint8_t adrHi = LocoNetSV.readSVStorage(SV_ADDR_CHANGE_ID_H) ;
  uint8_t address = ((adrHi * 256 + adrLo) * 10);

  EEPROM.put(SV_ADDR_SENSOR1_LNADR-2, address + 1);
  EEPROM.put(SV_ADDR_SENSOR1_TYPE-2, DEV_SENSOR);
  EEPROM.put(SV_ADDR_SENSOR1_SENSDISTANCE-2, 50);
  EEPROM.put(SV_ADDR_SENSOR1_ENABLED-2, 1);
  EEPROM.put(SV_ADDR_SENSOR2_LNADR-2, address + 2);
  EEPROM.put(SV_ADDR_SENSOR2_TYPE-2, DEV_SENSOR);
  EEPROM.put(SV_ADDR_SENSOR2_SENSDISTANCE-2, 50);
  EEPROM.put(SV_ADDR_SENSOR2_ENABLED-2, 1);
  EEPROM.put(SV_ADDR_SENSOR3_LNADR-2, address + 3);
  EEPROM.put(SV_ADDR_SENSOR3_TYPE-2, DEV_SENSOR);
  EEPROM.put(SV_ADDR_SENSOR3_SENSDISTANCE-2, 50);
  EEPROM.put(SV_ADDR_SENSOR3_ENABLED-2, 1);
  EEPROM.put(SV_ADDR_SENSOR4_LNADR-2, address + 4);
  EEPROM.put(SV_ADDR_SENSOR4_TYPE-2, DEV_SENSOR);
  EEPROM.put(SV_ADDR_SENSOR4_SENSDISTANCE-2, 50);
  EEPROM.put(SV_ADDR_SENSOR4_ENABLED-2, 1);
  EEPROM.put(SV_ADDR_SENSOR5_LNADR-2, address + 5);
  EEPROM.put(SV_ADDR_SENSOR5_TYPE-2, DEV_SENSOR);
  EEPROM.put(SV_ADDR_SENSOR5_SENSDISTANCE-2, 50);
  EEPROM.put(SV_ADDR_SENSOR5_ENABLED-2, 1);
  EEPROM.put(SV_ADDR_SENSOR6_LNADR-2, address + 6);
  EEPROM.put(SV_ADDR_SENSOR6_TYPE-2, DEV_SENSOR);
  EEPROM.put(SV_ADDR_SENSOR6_SENSDISTANCE-2, 50);
  EEPROM.put(SV_ADDR_SENSOR6_ENABLED-2, 1);
  EEPROM.put(SV_ADDR_SENSOR7_LNADR-2, address + 7);
  EEPROM.put(SV_ADDR_SENSOR7_TYPE-2, DEV_SENSOR);
  EEPROM.put(SV_ADDR_SENSOR7_SENSDISTANCE-2, 50);
  EEPROM.put(SV_ADDR_SENSOR7_ENABLED-2, 1);
  EEPROM.put(SV_ADDR_SENSOR8_LNADR-2, address + 8);
  EEPROM.put(SV_ADDR_SENSOR8_TYPE-2, DEV_SENSOR);
  EEPROM.put(SV_ADDR_SENSOR8_SENSDISTANCE-2, 50);
  EEPROM.put(SV_ADDR_SENSOR8_ENABLED-2, 1);

  Serial.println("Reset Completato. Riavviare per applicare le modifiche!");

}

// print the current configuration
void printConfiguration() {

  Serial.println(("Configurazione corrente:"));
  Serial.println();

  for(int i = 0; i < pinCount; i++) {

    Serial.print(("I/O ")); Serial.print(i + 1); Serial.print("\t");
    Serial.print(("Addr: ")); Serial.print(sensorInfo[i].dccAddress); Serial.print("  ");
    Serial.print(("Type: ")); Serial.print(("INPUT")); Serial.print("  ");
    Serial.print(("Device: ")); 
    if(sensorInfo[i].deviceType == DEV_SENSOR) Serial.print(("SENSORE")); 
    else if(sensorInfo[i].deviceType == DEV_SENSORTOF) Serial.print(("SENSORE TOF"));
    else if(sensorInfo[i].deviceType == DEV_SENSORIR) Serial.print(("SENSORE IR RX"));    
    else if(sensorInfo[i].deviceType == DEV_MSENSOR) Serial.print("MSENSOR"); 
    Serial.print("  ");
    Serial.print(("State: ")); sensorOn[i] ? Serial.print(("OFF")) : Serial.print(("ON")); Serial.print("  ");
    Serial.print(("ENABLED: ")); sensorInfo[i].isEnabled ? Serial.print(("YES")) : Serial.print(("NO")); Serial.print("\n");
  }
}
