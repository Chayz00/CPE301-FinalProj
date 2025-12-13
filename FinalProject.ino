#include <avr/io.h>
#include <avr/interrupt.h>
#include <DHT.h>   

//System state
typedef enum {
  STATE_DISABLED,
  STATE_IDLE,
  STATE_RUNNING,
  STATE_ERROR
} SystemState;

volatile SystemState systemState = STATE_DISABLED;
volatile uint8_t    startPending = 0;   

// LED pins (22–25 = PA0–PA3)
#define LED_DDR DDRA
#define LED_PORT PORTA

// Yellow= disabled, Green = idle, Red= error,Blue = running
#define LED_DISABLED_BIT PA0    // pin 22= Yellow
#define LED_IDLE_BIT     PA1    // pin 23 = Green
#define LED_ERROR_BIT    PA2    // pin 24 = Red
#define LED_RUNNING_BIT  PA3    // pin 25 = Blue

//Buttons 
#define BUTTON_DDR       DDRC
#define BUTTON_PORT      PORTC
#define BUTTON_PIN_REG   PINC
#define BUTTON_STOP_BIT  PC7      // pin 30
#define BUTTON_RESET_BIT PC6      // pin 31

// Start PORTE (INT4 - pin 2)
#define BUTTON_START_BIT PE4     // pin 2

// Water sensor (ADC channel 0)
static const unsigned char WATER_CHANNEL = 0;    // A0
static unsigned int  WATER_THRESHOLD = 200;  

volatile unsigned char *adcMux   = (unsigned char*) 0x7C;
volatile unsigned char *adcCtrlB = (unsigned char*) 0x7B;
volatile unsigned char *adcCtrlA = (unsigned char*) 0x7A;
volatile unsigned int  *adcData  = (unsigned int  *) 0x78;

void initAdc(void);
unsigned int readAdc(unsigned char channel);
uint8_t isWaterLow(void);

//  DHT11 (temp/humidity)
#define DHT_PIN   7
#define DHT_TYPE  DHT11
DHT dht(DHT_PIN, DHT_TYPE);

int     currentTempC    = 0;
int     currentHumidity = 0;
uint8_t hasValidDht     = 0;

void updateDhtIfNeeded(void);

// Temp transition tresholds
const uint8_t TEMP_ON_THRESHOLD  = 26;  // go to RUNNING when >= 26
const uint8_t TEMP_OFF_THRESHOLD = 24;  // go back to IDLE when <= 24

//UART0
#define RDA 0x80
#define TBE 0x20

volatile unsigned char *uartStatusA  = (unsigned char *)0x00C0;
volatile unsigned char *uartControlB = (unsigned char *)0x00C1;
volatile unsigned char *uartControlC = (unsigned char *)0x00C2;
volatile unsigned int  *uartBaud     = (unsigned int  *)0x00C4;
volatile unsigned char *uartData     = (unsigned char *)0x00C6;

void uartInit(int baud);
unsigned char uartKbHit(void);
unsigned char uartGetChar(void);
void uartPutChar(unsigned char c);
void uartPrintStr(const char *s);
void uartPrintNewline(void);
void uartPrintUint(unsigned int value);
const char* stateToString(SystemState s);
void logStateChange(SystemState oldState, SystemState newState);
void logStatusIfNeeded(void);

// Prototypes
void setupPins(void);
void updateLeds(void);
void handleButtonsSensorsAndState(void);
void setState(SystemState newState);

// Start button callback
void onStartButton()
{
  startPending = 1;
}

// Setup
void setup()
{
  setupPins();
  initAdc();
  uartInit(9600);
  dht.begin();  

  attachInterrupt(digitalPinToInterrupt(2), onStartButton, FALLING);

  systemState = STATE_DISABLED;
  updateLeds();
  uartPrintStr("System booted. State=DISABLED");
  uartPrintNewline();
}

// Loop
void loop()
{
  handleButtonsSensorsAndState();

  // No monitoring/logging in DISABLED
  if (systemState != STATE_DISABLED) {
    updateDhtIfNeeded();
    logStatusIfNeeded();   
  }
}

// GPIO
void setupPins(void)
{
  // LED 
  LED_DDR |= (1 << LED_DISABLED_BIT) |
             (1 << LED_IDLE_BIT)     |
             (1 << LED_ERROR_BIT)    |
             (1 << LED_RUNNING_BIT);

  LED_PORT &= ~((1 << LED_DISABLED_BIT) |
                (1 << LED_IDLE_BIT)     |
                (1 << LED_ERROR_BIT)    |
                (1 << LED_RUNNING_BIT));

  // Stop & Reset button
  BUTTON_DDR  &= ~((1 << BUTTON_STOP_BIT) | (1 << BUTTON_RESET_BIT)); // inputs
  BUTTON_PORT |=  (1 << BUTTON_STOP_BIT) | (1 << BUTTON_RESET_BIT);   // pull-ups

  // Start button 
  DDRE  &= ~(1 << BUTTON_START_BIT);
  PORTE |=  (1 << BUTTON_START_BIT);

  
}

//LED update 
void updateLeds(void)
{
  // All off
  LED_PORT &= ~((1 << LED_DISABLED_BIT) | (1 << LED_IDLE_BIT)  | (1 << LED_ERROR_BIT) | (1 << LED_RUNNING_BIT));

  switch (systemState) {
    case STATE_DISABLED:
      LED_PORT |= (1 << LED_DISABLED_BIT);  // Yellow
      break;
    case STATE_IDLE:
      LED_PORT |= (1 << LED_IDLE_BIT);      // Green
      break;
    case STATE_ERROR:
      LED_PORT |= (1 << LED_ERROR_BIT);     // Red
      break;
    case STATE_RUNNING:
      LED_PORT |= (1 << LED_RUNNING_BIT);   // Blue
      break;
  }
}

// State Change
void setState(SystemState newState)
{
  if (systemState != newState) {
    SystemState oldState = systemState;
    systemState = newState;
    updateLeds();
    logStateChange(oldState, newState);
  }
}

// Buttons + water sensor + state 
void handleButtonsSensorsAndState(void)
{
  // Buttons
  uint8_t buttonPins   = BUTTON_PIN_REG;
  uint8_t stopPressed  = !(buttonPins & (1 << BUTTON_STOP_BIT));   // active low
  uint8_t resetPressed = !(buttonPins & (1 << BUTTON_RESET_BIT));  // active low

  // Start 
  if (startPending) {
    startPending = 0;

    if (systemState == STATE_DISABLED) {
      setState(STATE_IDLE);
    }
  }

  // Stop
  if (stopPressed) {
    setState(STATE_DISABLED);
    return;   
  }

  // Reset
  if (resetPressed && systemState == STATE_ERROR) {
    if (!isWaterLow()) {
      setState(STATE_IDLE);
    }
  }

  // IDLE/RUNNING (ERROR when water is low)
  if (systemState == STATE_IDLE || systemState == STATE_RUNNING) {
    if (isWaterLow()) {
      setState(STATE_ERROR);
      return;   
    }
  }
 //Check for valid 
  if (!hasValidDht) {
    return;
  }
 // Check water level 
  if (!isWaterLow()) {
    if (systemState == STATE_IDLE && currentTempC >= TEMP_ON_THRESHOLD) {
      setState(STATE_RUNNING);
    } else if (systemState == STATE_RUNNING && currentTempC <= TEMP_OFF_THRESHOLD) {
      setState(STATE_IDLE);
    }
  }
}

//ADC - Lab 8 
void initAdc(void) 
{
  // Setup A register
  *adcCtrlA = (1 << 7) | (0 << 5) | (0 << 3) | (1 << 2) | (1 << 1) | (1 << 0); 
  // Setup B register
  *adcCtrlB = (0 << 3) | (0 << 2) | (0 << 1) | (0 << 0); 

  // Setup MUX register
  *adcMux  = (0 << 7)  | (1 << 6) | (0 << 5)  | (0x00);                 
}

unsigned int readAdc(unsigned char channel)
{
  channel &= 0x07;   // only 0–7

  *adcMux = (*adcMux & 0xE0) | (channel & 0x1F);
  *adcCtrlB &= ~(1 << 3); 
  *adcCtrlA |= (1 << 6);
  while ((*adcCtrlA & (1 << 6)) != 0) {

  }

  unsigned int value = *adcData;
  return value;
}

// Water sensor helper 
uint8_t isWaterLow(void)
{
  unsigned int reading = readAdc(WATER_CHANNEL);
  if (reading <= WATER_THRESHOLD) {
    return 1;   // water low 
  } else {
    return 0;   // water OK 
  }
}

//  DHT11 helper 
void updateDhtIfNeeded(void)
{
  // No monitoring of temp in DISABLED
  if (systemState == STATE_DISABLED) {
    return;
  }

  static unsigned long lastReadMs = 0;
  unsigned long now = millis();

  if (now - lastReadMs < 2000) {
    return;
  }
  lastReadMs = now;

  float h = dht.readHumidity();
  float t = dht.readTemperature(); // Celsius

  if (isnan(h) || isnan(t)) {
    hasValidDht = 0;
    return;
  }

  currentHumidity = (int)h;
  currentTempC    = (int)t;
  hasValidDht     = 1;
}

//  UART helpers
void uartInit(int baud)
{
  unsigned long FCPU = 16000000UL;
  unsigned int tbaud = (unsigned int)(FCPU / 16UL / baud - 1UL);

  *uartStatusA  = 0x20;   
  *uartControlB = 0x18;  
  *uartControlC = 0x06;  
  *uartBaud     = tbaud;
}

unsigned char uartKbHit(void)
{
  return *uartStatusA & RDA;
}

unsigned char uartGetChar(void)
{
  return *uartData;
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

const char* stateToString(SystemState s)
{
  switch (s) {
    case STATE_DISABLED: return "DISABLED";
    case STATE_IDLE:     return "IDLE";
    case STATE_RUNNING:  return "RUNNING";
    case STATE_ERROR:    return "ERROR";
    default:             return "UNKNOWN";
  }
}

void logStateChange(SystemState oldState, SystemState newState)
{
  uartPrintStr("State change: ");
  uartPrintStr(stateToString(oldState));
  uartPrintStr(" -> ");
  uartPrintStr(stateToString(newState));
  uartPrintNewline();
}

void logStatusIfNeeded(void)
{
 // No monitoring/logging in DISABLED
  if (systemState == STATE_DISABLED) {
    return;
  }

  static unsigned long lastLogMs = 0;
  unsigned long now = millis();

  if (now - lastLogMs >= 1000) {   // every second
    lastLogMs = now;
    unsigned int reading = readAdc(WATER_CHANNEL);

    uartPrintStr("Status: ");
    uartPrintStr(stateToString(systemState));
    uartPrintStr(" | ");

    if (hasValidDht) {
      uartPrintStr("T=");
      uartPrintUint((unsigned int)currentTempC);
      uartPrintStr("C H=");
      uartPrintUint((unsigned int)currentHumidity);
      uartPrintStr("% | ");
    } else {
      uartPrintStr("T=-- H=-- | ");
    }

    uartPrintStr("waterADC=");
    uartPrintUint(reading);
    uartPrintStr(" | low=");
    uartPutChar(isWaterLow() ? '1' : '0');
    uartPrintNewline();
  }
}
