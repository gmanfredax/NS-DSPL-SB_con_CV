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
#include <util/atomic.h>
#include <avr/interrupt.h>
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
#define DEV_SENSORIR    4   // sensore IR (beacon / LISSY via IRremote)

#define ON_DISTANCE     50
#define THRESHOLD       5

#define STATUSLED       13

// ---- IR multi-receiver (NEC) configurabile via CV ----
// Imposta uno o più canali come DEV_SENSORIR e abilitali: ciascun canale userà il proprio pin xshut_pins[i]
// e invierà un LISSY_REP con unitAddr = sensorInfo[i].dccAddress

#define OPC_LISSY_REP 0xE4
const uint16_t IR_RETRIGGER_MS = 500;

#define IR_PORT_B 0
#define IR_PORT_C 1
#define IR_PORT_D 2

#define IR_STATE_IDLE        0
#define IR_STATE_LEAD_MARK   1
#define IR_STATE_LEAD_SPACE  2
#define IR_STATE_BIT_MARK    3
#define IR_STATE_BIT_SPACE   4
#define IR_STATE_STOP_MARK   5

typedef struct {
  uint8_t pin;        // Arduino pin (xshut_pins[i])
  uint8_t portId;     // IR_PORT_*
  uint8_t bitMask;    // bitmask nel port
  volatile uint8_t active;

  volatile uint8_t lastLevel;     // 0/1
  volatile uint32_t lastEdgeUs;   // micros() all'ultimo edge
  volatile uint8_t state;
  volatile uint8_t bitIndex;
  volatile uint32_t data;

  volatile uint32_t frameData;
  volatile uint8_t frameReady;
} IrNecDecoder;

static IrNecDecoder g_irDec[IOPINS];
static volatile uint8_t g_irPortMaskB = 0;
static volatile uint8_t g_irPortMaskC = 0;
static volatile uint8_t g_irPortMaskD = 0;
static volatile uint8_t g_lastPortB = 0;
static volatile uint8_t g_lastPortC = 0;
static volatile uint8_t g_lastPortD = 0;

static uint32_t g_irLastMs[IOPINS];
static uint16_t g_irLastLoco[IOPINS];

// prototipi
void initIrReceivers();
void processIRBeacons();
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
      pinMode(xshut_pins[i], INPUT_PULLUP);
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
        //failedStart = true;
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

    initIrReceivers();
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
  processIRBeacons();

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

// --- NEC decoder helpers (multi-pin) ---
static inline bool irMatch(uint32_t dur, uint32_t target) {
  // tolleranza ~25%
  uint32_t tol = target / 4;
  return (dur > (target - tol)) && (dur < (target + tol));
}

static inline void irReset(uint8_t idx) {
  g_irDec[idx].state = IR_STATE_IDLE;
  g_irDec[idx].bitIndex = 0;
  g_irDec[idx].data = 0;
}

static inline void irOnEdge(uint8_t idx, uint8_t level, uint32_t nowUs) {
  IrNecDecoder &d = g_irDec[idx];
  if (!d.active) return;

  uint8_t prev = d.lastLevel;
  uint32_t dur = nowUs - d.lastEdgeUs;

  d.lastEdgeUs = nowUs;
  d.lastLevel = level;

  // idle HIGH, mark = LOW (tipico ricevitore demodulato)
  if (prev == 1 && level == 0) { // FALLING: inizio MARK, finisce uno SPACE
    if (d.state == IR_STATE_IDLE) {
      d.state = IR_STATE_LEAD_MARK;
      d.bitIndex = 0;
      d.data = 0;
      return;
    }

    if (d.state == IR_STATE_LEAD_SPACE) {
      if (irMatch(dur, 4500)) {        // start space
        d.state = IR_STATE_BIT_MARK;   // ora siamo all'inizio del mark del bit0
        return;
      } else if (irMatch(dur, 2250)) { // repeat code -> ignoriamo
        irReset(idx);
        return;
      } else {
        irReset(idx);
        return;
      }
    }

    if (d.state == IR_STATE_BIT_SPACE) {
      uint8_t bit;
      if (irMatch(dur, 560)) bit = 0;
      else if (irMatch(dur, 1690)) bit = 1;
      else { irReset(idx); return; }

      d.data |= ((uint32_t)bit << d.bitIndex);
      d.bitIndex++;

      if (d.bitIndex >= 32) d.state = IR_STATE_STOP_MARK;
      else d.state = IR_STATE_BIT_MARK;

      return;
    }
  }

  if (prev == 0 && level == 1) { // RISING: fine MARK, inizio SPACE
    if (d.state == IR_STATE_LEAD_MARK) {
      if (irMatch(dur, 9000)) d.state = IR_STATE_LEAD_SPACE;
      else irReset(idx);
      return;
    }

    if (d.state == IR_STATE_BIT_MARK) {
      if (!irMatch(dur, 560)) { irReset(idx); return; }
      d.state = IR_STATE_BIT_SPACE;
      return;
    }

    if (d.state == IR_STATE_STOP_MARK) {
      if (irMatch(dur, 560)) {
        if (!d.frameReady) {
          d.frameData = d.data;
          d.frameReady = 1;
        }
      }
      irReset(idx);
      return;
    }
  }
}

static inline void irHandlePort(uint8_t portId, uint8_t portVal, uint8_t changedMask, uint32_t nowUs) {
  if (!changedMask) return;

  for (uint8_t i = 0; i < pinCount; i++) {
    if (!g_irDec[i].active) continue;
    if (g_irDec[i].portId != portId) continue;

    uint8_t m = g_irDec[i].bitMask;
    if (changedMask & m) {
      uint8_t level = (portVal & m) ? 1 : 0;
      irOnEdge(i, level, nowUs);
    }
  }
}

// ISR PCINT port B (D8..D13)
ISR(PCINT0_vect) {
  uint32_t nowUs = micros();
  uint8_t v = PINB;
  uint8_t changed = (v ^ g_lastPortB) & g_irPortMaskB;
  g_lastPortB = v;
  irHandlePort(IR_PORT_B, v, changed, nowUs);
}

// ISR PCINT port C (A0..A5)
ISR(PCINT1_vect) {
  uint32_t nowUs = micros();
  uint8_t v = PINC;
  uint8_t changed = (v ^ g_lastPortC) & g_irPortMaskC;
  g_lastPortC = v;
  irHandlePort(IR_PORT_C, v, changed, nowUs);
}

// ISR PCINT port D (D0..D7)
ISR(PCINT2_vect) {
  uint32_t nowUs = micros();
  uint8_t v = PIND;
  uint8_t changed = (v ^ g_lastPortD) & g_irPortMaskD;
  g_lastPortD = v;
  irHandlePort(IR_PORT_D, v, changed, nowUs);
}

void initIrReceivers() {
  // reset
  for (uint8_t i = 0; i < pinCount; i++) {
    g_irDec[i].active = 0;
    g_irDec[i].frameReady = 0;
    g_irLastMs[i] = 0;
    g_irLastLoco[i] = 0;
  }

  noInterrupts();

  g_irPortMaskB = 0;
  g_irPortMaskC = 0;
  g_irPortMaskD = 0;

  // configura canali IR (uno per ciascun I/O configurato come DEV_SENSORIR)
  for (uint8_t i = 0; i < pinCount; i++) {
    if (!(sensorInfo[i].isEnabled && sensorInfo[i].deviceType == DEV_SENSORIR)) continue;

    uint8_t pin = xshut_pins[i];

    g_irDec[i].active = 1;
    g_irDec[i].pin = pin;
    g_irDec[i].state = IR_STATE_IDLE;
    g_irDec[i].bitIndex = 0;
    g_irDec[i].data = 0;
    g_irDec[i].frameReady = 0;
    g_irDec[i].frameData = 0;

    // mappa pin -> port + maschera
    if (pin <= 7) { // D0..D7 -> PORTD / PCMSK2
      g_irDec[i].portId = IR_PORT_D;
      g_irDec[i].bitMask = (1 << pin);
      g_irPortMaskD |= (1 << pin);
      PCICR |= (1 << PCIE2);
      PCMSK2 |= (1 << pin);
    } else if (pin <= 13) { // D8..D13 -> PORTB / PCMSK0
      g_irDec[i].portId = IR_PORT_B;
      g_irDec[i].bitMask = (1 << (pin - 8));
      g_irPortMaskB |= (1 << (pin - 8));
      PCICR |= (1 << PCIE0);
      PCMSK0 |= (1 << (pin - 8));
    } else if (pin <= 19) { // A0..A5 -> PORTC / PCMSK1
      g_irDec[i].portId = IR_PORT_C;
      g_irDec[i].bitMask = (1 << (pin - 14));
      g_irPortMaskC |= (1 << (pin - 14));
      PCICR |= (1 << PCIE1);
      PCMSK1 |= (1 << (pin - 14));
    } else {
      // pin non supportato su ATmega328p -> disabilita questo canale
      g_irDec[i].active = 0;
      continue;
    }
  }

  // snapshot livelli iniziali (per calcolare i change)
  g_lastPortB = PINB;
  g_lastPortC = PINC;
  g_lastPortD = PIND;

  // inizializza livelli/tempi per ogni canale attivo
  uint32_t nowUs = micros();
  for (uint8_t i = 0; i < pinCount; i++) {
    if (!g_irDec[i].active) continue;

    uint8_t level = 1;
    if (g_irDec[i].portId == IR_PORT_B) level = (g_lastPortB & g_irDec[i].bitMask) ? 1 : 0;
    else if (g_irDec[i].portId == IR_PORT_C) level = (g_lastPortC & g_irDec[i].bitMask) ? 1 : 0;
    else if (g_irDec[i].portId == IR_PORT_D) level = (g_lastPortD & g_irDec[i].bitMask) ? 1 : 0;

    g_irDec[i].lastLevel = level;
    g_irDec[i].lastEdgeUs = nowUs;
  }

  interrupts();

#ifdef DEBUG
  Serial.println(F("IR multi-receiver:"));
  for (uint8_t i = 0; i < pinCount; i++) {
    if (!(sensorInfo[i].isEnabled && sensorInfo[i].deviceType == DEV_SENSORIR)) continue;
    Serial.print(F(" - IO "));
    Serial.print(i + 1);
    Serial.print(F(" pin "));
    Serial.print(xshut_pins[i]);
    Serial.print(F(" unitAddr "));
    Serial.println(sensorInfo[i].dccAddress);
  }
#endif
}

void processIRBeacons() {
  for (uint8_t i = 0; i < pinCount; i++) {
    if (!g_irDec[i].active) continue;

    uint8_t ready = 0;
    uint32_t frame = 0;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      ready = g_irDec[i].frameReady;
      if (ready) {
        frame = g_irDec[i].frameData;
        g_irDec[i].frameReady = 0;
      }
    }

    if (!ready) continue;

    uint8_t addr    = (frame >> 0)  & 0xFF;
    uint8_t addrInv = (frame >> 8)  & 0xFF;
    uint8_t cmd     = (frame >> 16) & 0xFF;
    uint8_t cmdInv  = (frame >> 24) & 0xFF;

    // Validazione NEC (byte + complement)
    if (((addr ^ addrInv) != 0xFF) || ((cmd ^ cmdInv) != 0xFF)) {
#ifdef DEBUG
      Serial.print(F("IR drop (NEC checksum) IO "));
      Serial.println(i + 1);
#endif
      continue;
    }

    uint16_t locoId = cmd;
    if (locoId == 0) continue;

    uint32_t nowMs = millis();
    if ((locoId == g_irLastLoco[i]) && (nowMs - g_irLastMs[i] < IR_RETRIGGER_MS)) {
      continue;
    }

    g_irLastLoco[i] = locoId;
    g_irLastMs[i] = nowMs;

    uint16_t unitAddr = sensorInfo[i].dccAddress;
    sendLissyReport(unitAddr, locoId);

#ifdef DEBUG
    Serial.print(F("IR OK: IO "));
    Serial.print(i + 1);
    Serial.print(F(" loco "));
    Serial.print(locoId);
    Serial.print(F(" -> unit "));
    Serial.println(unitAddr);
#endif
  }
}

