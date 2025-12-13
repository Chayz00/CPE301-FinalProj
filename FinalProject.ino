#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>
#include <RTClib.h>         
#include <DHT.h>            
#include <LiquidCrystal.h>  
#include <Stepper.h>        

// Pin assignments 

// LEDs  
const uint8_t PIN_LED_DISABLED = 22; // Yellow -> PA0
const uint8_t PIN_LED_IDLE     = 23; // Green  -> PA1
const uint8_t PIN_LED_ERROR    = 24; // Red    -> PA2
const uint8_t PIN_LED_RUNNING  = 25; // Blue   -> PA3

// Buttons
const uint8_t PIN_BTN_START = 2;   // Start -> PE4 (INT4)
const uint8_t PIN_BTN_STOP  = 30;  // Stop  -> PC7
const uint8_t PIN_BTN_RESET = 31;  // Reset -> PC6

// Fan control (D6 -> PH3)
const uint8_t PIN_FAN = 6;

// DHT11
const uint8_t PIN_DHT = 7;
#define DHTTYPE DHT11

// Water sensor (ADC channel 0 -> A0)
const uint8_t WATER_ADC_CHANNEL = 0;

// Pot for stepper control (A1 -> ADC channel 1)
const uint8_t PIN_VENT_POT = A1;

// Stepper pins  IN1–IN4
const uint8_t PIN_STEP_IN1 = 32;
const uint8_t PIN_STEP_IN2 = 33;
const uint8_t PIN_STEP_IN3 = 34;
const uint8_t PIN_STEP_IN4 = 35;

//GPIO

// LEDs on PORTA bits 0–3
#define LED_DDR        DDRA
#define LED_PORT       PORTA
#define LED_DISABLED_BIT PA0    // pin 22
#define LED_IDLE_BIT     PA1    // pin 23
#define LED_ERROR_BIT    PA2    // pin 24
#define LED_RUNNING_BIT  PA3    // pin 25

// Fan on PORTH bit 3 (D6)
#define FAN_DDR   DDRH
#define FAN_PORT  PORTH
#define FAN_BIT   PH3

// Stop & Reset buttons on PORTC bits 7 & 6
#define BUTTON_DDR       DDRC
#define BUTTON_PORT      PORTC
#define BUTTON_PIN_REG   PINC
#define BUTTON_STOP_BIT  PC7      // pin 30
#define BUTTON_RESET_BIT PC6      // pin 31

// Start button PORTE (INT4 - pin 2 = PE4)
#define BUTTON_START_DDR  DDRE
#define BUTTON_START_PORT PORTE
#define BUTTON_START_BIT  PE4

// LCD: RS, EN, D4, D5, D6, D7
LiquidCrystal lcd(8, 9, 10, 11, 12, 13);
DHT dht(PIN_DHT, DHTTYPE);
RTC_DS3231 rtc;

// Stepper setup
const int STEPS_PER_REV   = 2048;
const int VENT_MAX_STEPS  = STEPS_PER_REV / 2; // 1024
const int VENT_STEP_CHUNK = 8;
const unsigned long VENT_UPDATE_MS = 50UL;
Stepper ventStepper(STEPS_PER_REV, PIN_STEP_IN1, PIN_STEP_IN3, PIN_STEP_IN2, PIN_STEP_IN4);
long ventCurrentSteps = 0; 
unsigned long lastVentMillis = 0;

// ADC 
volatile unsigned char *my_ADMUX    = (unsigned char*) 0x7C;
volatile unsigned char *my_ADCSRB   = (unsigned char*) 0x7B;
volatile unsigned char *my_ADCSRA   = (unsigned char*) 0x7A;
volatile unsigned int  *my_ADC_DATA = (unsigned int*)  0x78;

void adc_init() {
  *my_ADCSRA = (1 << 7)        // ADEN: enable ADC
             | (0 << 5)        // ADATE: auto trigger off
             | (0 << 3)        // ADIE: interrupt off
             | (1 << 2) | (1 << 1) | (1 << 0); // prescaler 128

  *my_ADCSRB = 0x00;           // free running, MUX5=0
  *my_ADMUX  = (0 << 7)        // REFS1
             | (1 << 6)        // REFS0 -> AVCC
             | (0 << 5)        // right adjust
             | 0x00;           // start on channel 0
}

unsigned int adc_read(uint8_t adc_channel_num) {
  adc_channel_num &= 0x07;
  *my_ADMUX = (*my_ADMUX & 0xE0) | (adc_channel_num & 0x1F);
  *my_ADCSRB &= ~(1 << 3);   // MUX5=0

  *my_ADCSRA |= (1 << 6);    // ADSC: start

  while ((*my_ADCSRA & (1 << 6)) != 0) { } // wait

  return *my_ADC_DATA;
}

// State machine

enum SystemState {
  STATE_DISABLED,
  STATE_IDLE,
  STATE_RUNNING,
  STATE_ERROR
};

volatile SystemState systemState = STATE_DISABLED;
volatile bool startRequested = false;

float currentTempC    = NAN;
float currentHumidity = NAN;
unsigned int waterAdc = 0;
bool waterIsLow       = false;

// thresholds
const unsigned int WATER_ADC_THRESHOLD = 200; 
const float TEMP_ON_C  = 26.0; // fan on
const float TEMP_OFF_C = 25.0; // fan off

// timing
const unsigned long SENSOR_PERIOD_MS = 1000UL;
const unsigned long LCD_PERIOD_MS    = 60000UL;
const unsigned long STOP_DEBOUNCE_MS = 50UL;
unsigned long lastSensorMillis = 0;
unsigned long lastLcdMillis    = 0;
unsigned long lastStopChangeMs = 0;
int           lastStopLevel    = HIGH;
bool          stopLatched      = false;

// RTC 
bool rtcOK = false;

// UART0
#define RDA 0x80
#define TBE 0x20

volatile unsigned char *uartStatusA  = (unsigned char *)0x00C0;
volatile unsigned char *uartControlB = (unsigned char *)0x00C1;
volatile unsigned char *uartControlC = (unsigned char *)0x00C2;
volatile unsigned int  *uartBaud     = (unsigned int  *)0x00C4;
volatile unsigned char *uartData     = (unsigned char *)0x00C6;

void uartInit(int baud);
void uartPutChar(unsigned char c);
void uartPrintStr(const char *s);
void uartPrintNewline(void);
void uartPrintUint(unsigned int value);
void uartPrint2Digit(uint8_t v);

// Prototypes
void initGPIO();
void updateLeds();
void updateFanOutput();
void updateLcd();
void sampleSensorsAndUpdateState();
void handleButtons();
void setState(SystemState newState);
const char* stateName(SystemState s);
void initRTC();
void logWithTimestampPrefix();
void logStateChange(SystemState fromState, SystemState toState);
void lcdPrint2Digit(int v);
void updateVentFromPot();

// ISR: Start button
void onStartPressed() {
  startRequested = true;  // falling edge = press
}

// Setup

void setup() {
  uartInit(9600);
  initGPIO();
  adc_init();
  dht.begin();
  lcd.begin(16, 2);
  ventStepper.setSpeed(10); 
  initRTC();
  systemState = STATE_DISABLED;
  updateLeds();
  updateFanOutput();
  updateLcd();  
  logWithTimestampPrefix();
  uartPrintStr("System booted. State=DISABLED");
  uartPrintNewline();
}

// Main loop
void loop() {
  // start button from ISR
  if (startRequested) {
    noInterrupts();
    bool localStart = startRequested;
    startRequested = false;
    interrupts();

    if (localStart && systemState == STATE_DISABLED) {
      setState(STATE_IDLE);
    }
  }

  handleButtons();

  unsigned long now = millis();

  // monitor sensors when enabled
  if (systemState != STATE_DISABLED &&
      (now - lastSensorMillis) >= SENSOR_PERIOD_MS) {
    lastSensorMillis = now;
    sampleSensorsAndUpdateState();
  }

  if ((now - lastLcdMillis) >= LCD_PERIOD_MS) {
    lastLcdMillis = now;
    updateLcd();
  }

  // Stepper update in all except DISABLED
  if (systemState != STATE_DISABLED && (now - lastVentMillis) >= VENT_UPDATE_MS) {
    lastVentMillis = now;
    updateVentFromPot();
  }
}

// GPIO

void initGPIO() {
  // LEDs (PA0–PA3)
  LED_DDR |= (1 << LED_DISABLED_BIT) | (1 << LED_IDLE_BIT)  | (1 << LED_ERROR_BIT) | (1 << LED_RUNNING_BIT);

  LED_PORT &= ~((1 << LED_DISABLED_BIT) | (1 << LED_IDLE_BIT)  | (1 << LED_ERROR_BIT) | (1 << LED_RUNNING_BIT));

  // Fan (PH3)
  FAN_DDR  |= (1 << FAN_BIT);
  FAN_PORT &= ~(1 << FAN_BIT);   // fan off 

  // Stop + Reset buttons (PC7, PC6) with pull-ups
  BUTTON_DDR  &= ~((1 << BUTTON_STOP_BIT) | (1 << BUTTON_RESET_BIT));
  BUTTON_PORT |=  (1 << BUTTON_STOP_BIT) | (1 << BUTTON_RESET_BIT);

  // Start button (PE4) with pull-up & external interrupt INT4
  BUTTON_START_DDR  &= ~(1 << BUTTON_START_BIT);
  BUTTON_START_PORT |=  (1 << BUTTON_START_BIT);

  attachInterrupt(digitalPinToInterrupt(PIN_BTN_START),
    onStartPressed, FALLING);
}  

// LEDs & Fan
void updateLeds() {
  // all off
  LED_PORT &= ~((1 << LED_DISABLED_BIT) | (1 << LED_IDLE_BIT) | (1 << LED_ERROR_BIT)  | (1 << LED_RUNNING_BIT));

  switch (systemState) {
    case STATE_DISABLED:
      LED_PORT |= (1 << LED_DISABLED_BIT);
      break;
    case STATE_IDLE:
      LED_PORT |= (1 << LED_IDLE_BIT);
      break;
    case STATE_ERROR:
      LED_PORT |= (1 << LED_ERROR_BIT);
      break;
    case STATE_RUNNING:
      LED_PORT |= (1 << LED_RUNNING_BIT);
      break;
  }
}

void updateFanOutput() {
  if (systemState == STATE_RUNNING) {
    FAN_PORT |=  (1 << FAN_BIT);   // fan ON
  } else {
    FAN_PORT &= ~(1 << FAN_BIT);   // fan OFF
  }
}

// States
void setState(SystemState newState) {
  if (systemState != newState) {
    SystemState oldState = systemState;
    systemState = newState;
    updateLeds();
    updateFanOutput();
    updateLcd();
    logStateChange(oldState, newState);
  }
}

// Buttons

void handleButtons() {
  unsigned long now = millis();

  // read stop & reset PORTC 
  int stopLevel  = (BUTTON_PIN_REG & (1 << BUTTON_STOP_BIT))  ? HIGH : LOW;
  int resetLevel = (BUTTON_PIN_REG & (1 << BUTTON_RESET_BIT)) ? HIGH : LOW;

  // Stop with debounce
  if (stopLevel != lastStopLevel) {
    lastStopLevel = stopLevel;
    lastStopChangeMs = now;
  }
  if ((now - lastStopChangeMs) > STOP_DEBOUNCE_MS) {
    if (stopLevel == LOW && !stopLatched) {
      setState(STATE_DISABLED);
      stopLatched = true;
    } else if (stopLevel == HIGH) {
      stopLatched = false;
    }
  }

  // RESET only in ERROR
  if (resetLevel == LOW && systemState == STATE_ERROR) {
    if (!waterIsLow) {
      setState(STATE_IDLE);
    }
  }
}

// Sensors + transitions 

void sampleSensorsAndUpdateState() {
  // water sensor with 16 sample average 
  unsigned long sum = 0;
  for (uint8_t i = 0; i < 16; ++i) {
    sum += adc_read(WATER_ADC_CHANNEL);
  }
  waterAdc = (unsigned int)(sum >> 4);
  waterIsLow = (waterAdc < WATER_ADC_THRESHOLD);

  if (waterIsLow) {
    if (systemState != STATE_ERROR) {
      setState(STATE_ERROR);
    }
  } else {
    // DHT
    float t = dht.readTemperature();
    float h = dht.readHumidity();

    if (!isnan(t) && !isnan(h)) {
      currentTempC    = (int)t;
      currentHumidity = (int)h;
    }

    if (!isnan(currentTempC)) {
      if (systemState == STATE_IDLE && currentTempC >= TEMP_ON_C) {
        setState(STATE_RUNNING);
      } else if (systemState == STATE_RUNNING && currentTempC <= TEMP_OFF_C) {
        setState(STATE_IDLE);
      }
    }
  }

  // status log with RTC 
  logWithTimestampPrefix();
  uartPrintStr("Status: ");
  uartPrintStr(stateName(systemState));
  uartPrintStr(" | T=");
  if (isnan(currentTempC)) {
    uartPrintStr("--C");
  } else {
    uartPrintUint((unsigned int)currentTempC);
    uartPrintStr("C");
  }
  uartPrintStr(" H=");
  if (isnan(currentHumidity)) {
    uartPrintStr("--%");
  } else {
    uartPrintUint((unsigned int)currentHumidity);
    uartPrintStr("%");
  }
  uartPrintStr(" | waterADC=");
  uartPrintUint(waterAdc);
  uartPrintStr(" | low=");
  uartPutChar(waterIsLow ? '1' : '0');
  uartPrintNewline();
}

// helper
void lcdPrint2Digit(int v) {
  if (v < 0) v = 0;
  if (v > 99) v = 99;
  if (v < 10) {
    lcd.print('0');
    lcd.print(v);
  } else {
    lcd.print(v);
  }
}

// LCD 
void updateLcd() {
  lcd.clear();

  // Line 1
  lcd.setCursor(0, 0);
  // state name
  switch (systemState) {
    case STATE_DISABLED:
      lcd.print("DISABLED");   
      break;
    case STATE_IDLE:
      lcd.print("IDLE    ");   
      break;
    case STATE_RUNNING:
      lcd.print("RUNNING ");   
      break;
    case STATE_ERROR:
      lcd.print("ERROR   ");    
      break;
  }

  lcd.print('T');
  if (isnan(currentTempC) || systemState == STATE_DISABLED) {
    lcd.print("--");
  } else {
    lcdPrint2Digit((int)currentTempC);
  }

  lcd.print(' ');
  lcd.print('H');
  if (isnan(currentHumidity) || systemState == STATE_DISABLED) {
    lcd.print("--");
  } else {
    lcdPrint2Digit((int)currentHumidity);
  }

  // line 2
  lcd.setCursor(0, 1);
  switch (systemState) {
    case STATE_DISABLED:
      lcd.print("Cooler OFF Start");   
      break;

    case STATE_IDLE:
      if (waterIsLow) {
        lcd.print("Low water ->ERR ");
      } else {
        lcd.print("Ready,monitoring"); 
      }
      break;

    case STATE_RUNNING:
      lcd.print("Fan ON, cooling ");  
      break;

    case STATE_ERROR:
      if (waterIsLow) {
        lcd.print("Low water! Reset"); 
      } else {
        lcd.print("Error! Press Rst"); 
      }
      break;
  }
} 

//Stepper
void updateVentFromPot() {
  int pot = (int)adc_read(1);   // A1 = channel 1

  long target = map(pot, 0, 1023, 0, VENT_MAX_STEPS);
  long delta = target - ventCurrentSteps;
  if (delta == 0) return;

  long stepsToMove = delta;
  if (stepsToMove > VENT_STEP_CHUNK)  stepsToMove = VENT_STEP_CHUNK;
  if (stepsToMove < -VENT_STEP_CHUNK) stepsToMove = -VENT_STEP_CHUNK;

  ventStepper.step((int)stepsToMove);
  ventCurrentSteps += stepsToMove;
}

// RTC helpers 
const char* stateName(SystemState s) {
  switch (s) {
    case STATE_DISABLED: return "DISABLED";
    case STATE_IDLE:     return "IDLE";
    case STATE_RUNNING:  return "RUNNING";
    case STATE_ERROR:    return "ERROR";
    default:             return "?";
  }
}

void initRTC() {
  Wire.begin();
  if (!rtc.begin()) {
    rtcOK = false;
    uartPrintStr("RTC init failed (no module?)");
    uartPrintNewline();
    return;
  }
  rtcOK = true;
  uartPrintStr("RTC initialized.");
  uartPrintNewline();
}

void logWithTimestampPrefix() {
  if (!rtcOK) {
    uartPrintStr("[no-RTC] ");
    return;
  }

  DateTime now = rtc.now();
  uartPutChar('[');
  uartPrintUint(now.year());
  uartPutChar('-');
  uartPrint2Digit(now.month());
  uartPutChar('-');
  uartPrint2Digit(now.day());
  uartPutChar(' ');
  uartPrint2Digit(now.hour());
  uartPutChar(':');
  uartPrint2Digit(now.minute());
  uartPutChar(':');
  uartPrint2Digit(now.second());
  uartPutChar(']');
  uartPutChar(' ');
}

void logStateChange(SystemState fromState, SystemState toState) {
  logWithTimestampPrefix();
  uartPrintStr("State change: ");
  uartPrintStr(stateName(fromState));
  uartPrintStr(" -> ");
  uartPrintStr(stateName(toState));
  uartPrintNewline();
}

//UART
void uartInit(int baud)
{
  unsigned long FCPU = 16000000UL;
  unsigned int tbaud = (unsigned int)(FCPU / 16UL / baud - 1UL);

  *uartStatusA  = 0x20;   // clear flags
  *uartControlB = 0x18;   // RXEN0 | TXEN0
  *uartControlC = 0x06;   // 8 data, 1 stop, no parity
  *uartBaud     = tbaud;
}

void uartPutChar(unsigned char c)
{
  while ((*uartStatusA & TBE) == 0) {
  }
  *uartData = c;
}

void uartPrintStr(const char *s)
{
  while (*s) {
    uartPutChar(*s++);
  }
}

void uartPrintNewline(void)
{
  uartPutChar('\r');
  uartPutChar('\n');
}

void uartPrintUint(unsigned int x)
{
  char buf[5];
  int i = 0;

  if (x == 0) {
    uartPutChar('0');
    return;
  }

  while (x > 0 && i < 5) {
    unsigned int q = x / 10;
    buf[i++] = '0' + (x - q * 10);
    x = q;
  }
  while (--i >= 0) {
    uartPutChar(buf[i]);
  }
}

void uartPrint2Digit(uint8_t v)
{
  if (v < 10) {
    uartPutChar('0');
  }
  uartPrintUint(v);
}
