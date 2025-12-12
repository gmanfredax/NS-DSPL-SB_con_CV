// NS-DSPL-SB v1.3
// Decoder DCC per 8 sensori (TOF e HALL EFFECT)
// Configurazione tramite CV
// NeXtorSystem 2025
// Gabriele Manfreda
//
// Ultimo Aggiornamento: 23/03/2025
// Ultima modifica: Introduzione supporto sensori effetto HALL
#include <avr/wdt.h>
#include <VL53L0X.h>
#include <LocoNet.h>
#include <Wire.h>
#include <EEPROM.h>
#define DECODE_NEC
#include <IRremote.hpp>
/*#include <Arduino_FreeRTOS.h>

void TaskBlink1( void *pvParameters );*/

#define DEBUG 1

//#define VERSION         "1.0"
#define IOPINS          8
//#define CONFIGVALID     111
#define CMDLINESIZE     100
#define DEV_TURNOUT     0
#define DEV_SENSOR      1
#define DEV_SENSORTOF   2
#define DEV_MSENSOR     3
#define DEV_SENSORIR    4

#define ON_DISTANCE     50
#define THRESHOLD       5

#define STATUSLED       13

const uint8_t xshut_pins[] = {4,5,7,9,10,14,15,16}; //{4,5,7,9,10,14,15,16};
// xshut_pins[0] -> pin X1 per sensore TOF
// xshut_pins[1] -> pin X2 per sensore TOF
// xshut_pins[2] -> pin X3 per sensore effetto HALL / TOF
// xshut_pins[3] -> pin X4 per sensore TOF
// xshut_pins[4] -> pin X5 per sensore TOF
// xshut_pins[5] -> pin X6 per sensore effetto HALL / TOF
// xshut_pins[6] -> pin X7 per sensore effetto HALL / TOF
// xshut_pins[7] -> pin X8 per sensore effetto HALL / TOF
const uint8_t pinCount = 8;

const uint8_t IR_UNIT_IO = IOPINS; // l'ultimo I/O (8) è l'unico ammesso per IR RX
const uint8_t IR_UNIT_INDEX = IR_UNIT_IO - 1;
const uint8_t IR_RECV_PIN = xshut_pins[IR_UNIT_INDEX];

#define OPC_LISSY_REP 0xE4
uint32_t lastIrTime = 0;
uint16_t lastLocoId = 0;
const uint16_t IR_RETRIGGER_MS = 500;

/******* Inizio Dichiarazione Indirizzo e Valori di default delle CV *******/

// const uint16_t SV_ADDR_SW_VERSION = 2 ;       
const uint8_t VALUE_SW_VERSION = 1 ;
const char VALUE_SW_REV[] = "2-beta1" ;      

// const uint16_t SV_ADDR_NODE_ID_L = 3 ;  
// const uint16_t SV_ADDR_NODE_ID_H = 4 ;   
const uint8_t VALUE_NODE_ID_L = 0 ;  // default address = 0
const uint8_t VALUE_NODE_ID_H = 0 ;  // address = SV_ADDR_NODE_ID_H * 256 + SV_ADDR_NODE_ID_L

//  SV_ADDR_SERIAL_NUMBER_L = 5,
//  SV_ADDR_SERIAL_NUMBER_H = 6,
const uint8_t VALUE_SERIAL_NUMBER_L = 1 ;  // default value = 1
const uint8_t VALUE_SERIAL_NUMBER_H = 0 ;  // address = SV_ADDR_NODE_ID_H * 256 + SV_ADDR_NODE_ID_L

// const uint16_t SV_ADDR_USER_BASE = 7 ;  

const uint8_t SV_ADDR_MANUFACTURER_ID = 8 ;
const uint8_t VALUE_MANUFACTURER_ID = 13 ;
const uint8_t SV_ADDR_DEVELOPER_ID = 9 ;
const uint8_t VALUE_DEVELOPER_ID = 21 ;
const uint8_t SV_ADDR_PRODUCT_ID_L = 10 ;
const uint8_t VALUE_PRODUCT_ID_L = 1 ;
const uint8_t SV_ADDR_PRODUCT_ID_H = 11 ;
const uint8_t VALUE_PRODUCT_ID_H = 0 ;

const uint8_t SV_ADDR_NUM_SENSORS = 12 ;  // Indirizzo EEPROM della variabile del numero di sensori abilitati
const uint8_t VALUE_NUM_SENSORS = 4 ;     // Numero di Sensori abilitati

const uint16_t SV_ADDR_ADDR_SENSORS_L = 13 ;   
const uint16_t SV_ADDR_ADDR_SENSORS_H = 14 ;  
const uint8_t VALUE_ADDR_SENSORS_L = 1 ;  // Sensors default address = 257
const uint8_t VALUE_ADDR_SENSORS_H = 1 ;  // address = SV_ADDR_ADDR_SENSORS_H * 256 + SV_ADDR_ADDR_SENSORS_L

const uint8_t SV_ADDR_CHANGE_ID_L = 15 ;  // default address = 1
const uint8_t SV_ADDR_CHANGE_ID_H = 16 ;  // address = SV_ADDR_NODE_ID_H * 256 + SV_ADDR_NODE_ID_L
const uint8_t VALUE_CHANGE_ID_L = 0 ;  // default address = 1
const uint8_t VALUE_CHANGE_ID_H = 0 ;  // address = SV_ADDR_NODE_ID_H * 256 + SV_ADDR_NODE_ID_L

const uint8_t SV_ADDR_AUXILIARY = 17 ;
const uint8_t VALUE_AUXILIARY = 7 ;

const uint8_t SV_ADDR_SENSOR1_LNADR = 20 ;
const uint8_t SV_ADDR_SENSOR1_TYPE = 21 ;
const uint8_t SV_ADDR_SENSOR1_ENABLED = 23 ;

const uint8_t SV_ADDR_SENSOR2_LNADR = 24 ;
const uint8_t SV_ADDR_SENSOR2_TYPE = 25 ;
const uint8_t SV_ADDR_SENSOR2_ENABLED = 27 ;

const uint8_t SV_ADDR_SENSOR3_LNADR = 28 ;
const uint8_t SV_ADDR_SENSOR3_TYPE = 29 ;
const uint8_t SV_ADDR_SENSOR3_ENABLED = 31 ;

const uint8_t SV_ADDR_SENSOR4_LNADR = 32 ;
const uint8_t SV_ADDR_SENSOR4_TYPE = 33 ;
const uint8_t SV_ADDR_SENSOR4_ENABLED = 35 ;

const uint8_t SV_ADDR_SENSOR5_LNADR = 36 ;
const uint8_t SV_ADDR_SENSOR5_TYPE = 37 ;
const uint8_t SV_ADDR_SENSOR5_ENABLED = 39 ;

const uint8_t SV_ADDR_SENSOR6_LNADR = 40 ;
const uint8_t SV_ADDR_SENSOR6_TYPE = 41 ;
const uint8_t SV_ADDR_SENSOR6_ENABLED = 43 ;

const uint8_t SV_ADDR_SENSOR7_LNADR = 44 ;
const uint8_t SV_ADDR_SENSOR7_TYPE = 45 ;
const uint8_t SV_ADDR_SENSOR7_ENABLED = 47 ;

const uint8_t SV_ADDR_SENSOR8_LNADR = 48 ;
const uint8_t SV_ADDR_SENSOR8_TYPE = 49 ;
const uint8_t SV_ADDR_SENSOR8_ENABLED = 51 ;

const uint8_t SV_ADDR_SENSOR1_SENSDISTANCE = 22 ;
const uint8_t SV_ADDR_SENSOR2_SENSDISTANCE = 26 ;
const uint8_t SV_ADDR_SENSOR3_SENSDISTANCE = 30 ;
const uint8_t SV_ADDR_SENSOR4_SENSDISTANCE = 34 ;
const uint8_t SV_ADDR_SENSOR5_SENSDISTANCE = 38 ;
const uint8_t SV_ADDR_SENSOR6_SENSDISTANCE = 42 ;
const uint8_t SV_ADDR_SENSOR7_SENSDISTANCE = 46 ;
const uint8_t SV_ADDR_SENSOR8_SENSDISTANCE = 50 ;

// address we will assign if dual sensor is present
//#define LOX1_ADDRESS     0x30
//#define LOX2_ADDRESS     0x31

#define LNET_TX_PIN          8

typedef struct {
  uint8_t dccAddress;  //int
  uint8_t deviceType;   //int
  uint8_t sensdistance;   //int
  uint8_t isEnabled;
} SensorInfo;

// Variabili Globali
VL53L0X sensor[pinCount];
int hallSensorSTDVAL = 543;

LocoNetSystemVariableClass LocoNetSV;
lnMsg *LnPacket;
SV_STATUS   svStatus = SV_OK;
boolean     deferredProcessingNeeded = false;

SensorInfo sensorInfo[pinCount];
bool sensorOn[pinCount] = {false,false,false,false,false,false,false,false};
char cmdline[CMDLINESIZE];
bool failedStart = false;
uint8_t cmdlinepos = 0;
uint8_t blinkled = 0;
unsigned long prevmillis;
unsigned long currentmillis;
unsigned long interval = 1500; //tempo per ogni interrogazione dei sensori

void setup() {
  delay(2500);

  uint8_t adrLo = LocoNetSV.readSVStorage(SV_ADDR_CHANGE_ID_L) ;
  uint8_t adrHi = LocoNetSV.readSVStorage(SV_ADDR_CHANGE_ID_H) ;
  uint8_t loconetADR = ((adrHi * 256 + adrLo) * 10);

  Serial.begin(115200);
  Serial.print(("NS-SB v"));
  Serial.print(VALUE_SW_VERSION);
  Serial.print(".");
  Serial.println(VALUE_SW_REV);
  Serial.print("Data Revisione: ");
  Serial.println(__DATE__);
  Serial.print("Indirizzo Loconet: ");
  Serial.println(loconetADR);
  Serial.println();

  Wire.begin();

  //xTaskCreate(TaskBlink1, "task1", 128, NULL, 1, NULL);

  pinMode(STATUSLED, OUTPUT);  

  // check if the configuration in the EEPROM is valid
  uint8_t auxiliary = LocoNetSV.readSVStorage(SV_ADDR_AUXILIARY) ;
  if (auxiliary != VALUE_AUXILIARY) {
    setFactoryDefault();
  } else {
    readConfigFromStorage();
    digitalWrite(STATUSLED, HIGH);
  }

  LocoNet.init();
  uint16_t productId = VALUE_PRODUCT_ID_L + 256 * VALUE_PRODUCT_ID_H ;
  LocoNetSV.init(VALUE_MANUFACTURER_ID, VALUE_DEVELOPER_ID, productId, VALUE_SW_VERSION);
  Serial.println("- Bus LocoNet avviato correttamente");

  initLocalVariables() ;

  //Inizializzo i pin legati alle CV

  for (uint8_t i = 0; i < pinCount; i++) {
    // Se il pin è configurato come sensore TOF ed è attivo
    if (sensorInfo[i].deviceType == DEV_SENSORTOF and sensorInfo[i].isEnabled) {
      pinMode(xshut_pins[i], OUTPUT);
      digitalWrite(xshut_pins[i], LOW);
    }
    // Se il pin è configurato come sensore ed è attivo
    if (sensorInfo[i].deviceType == DEV_SENSOR and sensorInfo[i].isEnabled) {
      pinMode(xshut_pins[i], INPUT);
    }
    // Se il pin è configurato come sensore IR ed è attivo
    if (sensorInfo[i].deviceType == DEV_SENSORIR and sensorInfo[i].isEnabled) {
      pinMode(xshut_pins[i], INPUT);
    }
  }

  // Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < pinCount; i++)
  {    
    // Stop driving this sensor's XSHUT low. This should allow the carrier
    // board to pull it high. (We do NOT want to drive XSHUT high since it is
    // not level shifted.) Then wait a bit for the sensor to start up.
    if (sensorInfo[i].deviceType == DEV_SENSORTOF and sensorInfo[i].isEnabled) {
      digitalWrite(xshut_pins[i], HIGH);
      delay(10);
      if (!sensor[i].init()) {
        Serial.print("!!! Impossibile inizializzare il sensore #");
        Serial.print(i+1);
        Serial.println(" !!!");
        failedStart = true;
      }
      delay(10);
      sensor[i].setAddress(0x2A + i);
      sensor[i].setTimeout(500);
      sensor[i].setMeasurementTimingBudget(200000);
    }
  }

  Serial.println();
  if (failedStart) Serial.println(("Avvio completato con errori"));
  else Serial.println(("Avvio completato con successo!"));
  Serial.println();
  
  //vTaskStartScheduler();

  // IR receiver
  if (sensorInfo[IR_UNIT_INDEX].deviceType == DEV_SENSORIR && sensorInfo[IR_UNIT_INDEX].isEnabled) {
    pinMode(IR_RECV_PIN, INPUT);
    IrReceiver.begin(IR_RECV_PIN, ENABLE_LED_FEEDBACK);  // LED feedback opzionale
    Serial.print(F("IRreceiver pronto su I/O "));
    Serial.println(IR_UNIT_IO);
  } else {
    Serial.print(F("IRreceiver non attivato: I/O "));
    Serial.print(IR_UNIT_IO);
    Serial.println(F(" non configurato come SENSORIR abilitato"));
  }
}

void loop() {

  //Dcc.process();

  LnPacket = LocoNet.receive();
  if (LnPacket) {
    svStatus = LocoNetSV.processMessage(LnPacket);
    
    deferredProcessingNeeded = (svStatus == SV_DEFERRED_PROCESSING_NEEDED);

    if (deferredProcessingNeeded) {
      int counter = 11 ;
      do {
        counter-- ;
        deferredProcessingNeeded = (LocoNetSV.doDeferredProcessing() != SV_OK) ;
       
      } while ((deferredProcessingNeeded) && (counter > 0)) ;
    }
   
    LocoNet.processSwitchSensorMessage(LnPacket);
  }
 
  currentmillis = millis ();
  if (currentmillis - prevmillis > interval) { // time to check for changes on inputs
    if (failedStart) LocoNet.reportSensor(999, 1);
    else checkSensors();
  } 

  // Controllo beacon IR loco → LISSY su LocoNet
  checkIRBeacons();

  if (Serial.available() > 0) {

    int inByte = Serial.read();
    Serial.write(inByte);

    if (inByte == '\n') {
      cmdline[cmdlinepos] = '\0';
      parseCmdLine();
    }
    else {
      cmdline[cmdlinepos] = inByte;
      cmdlinepos++;
      if (cmdlinepos == CMDLINESIZE) cmdlinepos = 0;
    }
  }
}

void initLocalVariables() {

  uint8_t changeLo = LocoNetSV.readSVStorage(SV_ADDR_CHANGE_ID_L) ;
  uint8_t changeHi = LocoNetSV.readSVStorage(SV_ADDR_CHANGE_ID_H) ;
  LocoNetSV.writeSVStorage(SV_ADDR_NODE_ID_H, changeHi) ;
  LocoNetSV.writeSVStorage(SV_ADDR_NODE_ID_L, changeLo) ;
  
}

void checkSensors() {
  for (int i = 0; i < pinCount; i++) {
    uint16_t distance;
    String sensor_name;
    // Se il sensore è di tipo TOF
    if (sensorInfo[i].deviceType == DEV_SENSORTOF and sensorInfo[i].isEnabled) {
      distance = sensor[i].readRangeSingleMillimeters();
      sensor_name = "VL53L0X #";
      if (distance < (sensorInfo[i].sensdistance + 40) - THRESHOLD && !sensorOn[i]) {
        LocoNet.reportSensor(sensorInfo[i].dccAddress, 1);
        sensorOn[i] = true;
        #ifdef DEBUG
        Serial.print(sensor_name);
        Serial.print(i+1);
        Serial.print(" Sent sensor state - ");
        Serial.print("address: "); Serial.print(sensorInfo[i].dccAddress);
        Serial.print(", state: "); Serial.print("ON");
        Serial.print(" - Distanza: "); Serial.println(distance);
        #endif
      }
      else if (distance > (sensorInfo[i].sensdistance + 40) + THRESHOLD && sensorOn[i]) {
        LocoNet.reportSensor(sensorInfo[i].dccAddress, 0);
        sensorOn[i] = false;
        #ifdef DEBUG
        Serial.print(sensor_name);
        Serial.print(i+1);
        Serial.print(" Sent sensor state - ");
        Serial.print("address: "); Serial.print(sensorInfo[i].dccAddress);
        Serial.print(", state: "); Serial.print("OFF");
        Serial.print(" - Distanza: "); Serial.println(distance);
        #endif
      }
    }
    // Se il sensore è di tipo HALL
    if (sensorInfo[i].deviceType == DEV_SENSOR and sensorInfo[i].isEnabled){
      sensor_name = "HALLSENSOR #";
      int analog_value = analogRead(xshut_pins[i]);
      Serial.print("Analog Value HALL Sensor #");
      Serial.print(i+1);
      Serial.print(": "); Serial.println(analog_value);
      if (analog_value >= (hallSensorSTDVAL - 30) && analog_value <= (hallSensorSTDVAL + 30)) {
        LocoNet.reportSensor(sensorInfo[i].dccAddress, 0);
        #ifdef DEBUG
        Serial.print(sensor_name);
        Serial.print(i+1);
        Serial.print(" Sent sensor state - ");
        Serial.print("address: "); Serial.print(sensorInfo[i].dccAddress);
        Serial.print(", Treno: "); Serial.println("NON PRESENTE");
        #endif
      }
      else if (analog_value < (hallSensorSTDVAL - 30)) {//sensorInfo[i].sensdistance)) {
        LocoNet.reportSensor(sensorInfo[i].dccAddress, 1);
        #ifdef DEBUG
        Serial.print(sensor_name);
        Serial.print(i+1);
        Serial.print(" Sent sensor state - ");
        Serial.print("address: "); Serial.print(sensorInfo[i].dccAddress);
        Serial.print(", Treno: "); Serial.println("Bachmann 4-6-2");
        #endif
      }
      else if (analog_value > (hallSensorSTDVAL + 30)) {//sensorInfo[i].sensdistance)) {
        LocoNet.reportSensor(sensorInfo[i].dccAddress, 2);
        #ifdef DEBUG
        Serial.print(sensor_name);
        Serial.print(i+1);
        Serial.print(" Sent sensor state - ");
        Serial.print("address: "); Serial.print(sensorInfo[i].dccAddress);
        Serial.print(", Treno: "); Serial.println("Spectrum 2-6-0 Bumble Bee");
        #endif
      }
    }
  }
}

/*void TaskBlink1(void *pvParameters) {

  pinMode(STATUSLED, OUTPUT);   // GREEN LED

  while(blinkled) {
    digitalWrite(STATUSLED, LOW);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    digitalWrite(STATUSLED, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS);    
  }

}*/

// Invia un messaggio tipo LISSY a Rocrail: unit = sensore (indirizzo LocoNet),
// locoAddr = indirizzo DCC della loco
void sendLissyReport(uint16_t unit, uint16_t locoAddr) {
  lnMsg tx;

  // Formato semplificato OPC_LISSY_REP:
  // data[0] = 0xE4 (opcode)
  // data[1] = 0x08 (count)
  // data[2] = 0x00 (Arg1 fisso)
  // data[3] = high unit (senza direction per ora)
  // data[4] = low unit
  // data[5] = high addr
  // data[6] = low addr
  // data[7] = checksum XOR

  tx.data[0] = OPC_LISSY_REP;
  tx.data[1] = 0x08;
  tx.data[2] = 0x00;

  uint16_t unit12 = unit & 0x0FFF; // 12 bit
  uint8_t unitHi = (unit12 >> 8) & 0x0F;
  uint8_t unitLo = unit12 & 0xFF;

  tx.data[3] = unitHi; // bit direzione = 0 per ora
  tx.data[4] = unitLo;

  tx.data[5] = (locoAddr >> 8) & 0xFF;
  tx.data[6] = locoAddr & 0xFF;

  uint8_t chk = 0;
  for (uint8_t i = 0; i <= 6; i++) {
    chk ^= tx.data[i];
  }
  tx.data[7] = chk;

  // Invia su LocoNet
  LN_STATUS st = LocoNet.send(&tx);
#ifdef DEBUG
  Serial.print(F("LISSY REP unit="));
  Serial.print(unit);
  Serial.print(F(" loco="));
  Serial.print(locoAddr);
  Serial.print(F(" status="));
  Serial.println(st);
#endif
}

void checkIRBeacons() {
  if (!IrReceiver.decode()) {
    return;
  }

  auto &d = IrReceiver.decodedIRData;

#ifdef DEBUG
  Serial.print(F("IR recv: proto="));
  Serial.print(d.protocol);
  Serial.print(F(" addr="));
  Serial.print(d.address, HEX);
  Serial.print(F(" cmd="));
  Serial.println(d.command, HEX);
#endif

  // Filtra un minimo: accettiamo solo NEC, ma se il tuo tx usa altro puoi allentare
  // if (d.protocol != NEC) {   // se ti crea problemi, commenta questa riga
  //   IrReceiver.resume();
  //   return;
  // }

  // Loco ID = command (indirizzo corto DCC da 1..127)
  uint16_t locoId = d.command;

  // Piccolo debounce temporale per non spammare
  uint32_t now = millis();
  if (locoId == 0) {
    IrReceiver.resume();
    return;
  }
  if ((locoId == lastLocoId) && (now - lastIrTime < IR_RETRIGGER_MS)) {
    IrReceiver.resume();
    return;
  }
  lastLocoId = locoId;
  lastIrTime = now;

  // Scegli a quale "unit address" associare questo IR.
  // Esempio semplice: prendi il primo sensore abilitato di tipo TOF come "punto LISSY"
  int irIndex = IR_UNIT_INDEX;
  if (!(sensorInfo[irIndex].isEnabled && sensorInfo[irIndex].deviceType == DEV_SENSORIR)) {
#ifdef DEBUG
    Serial.println(F("Sensore IR non configurato o non abilitato su I/O 8 -> scarto"));
#endif
    IrReceiver.resume();
    return;
  }

  uint16_t unitAddr = sensorInfo[irIndex].dccAddress;

  sendLissyReport(unitAddr, locoId);

  IrReceiver.resume();
}
