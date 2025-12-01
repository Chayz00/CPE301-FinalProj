#include <avr/io.h>
#include <avr/interrupt.h>

// States
typedef enum {
  STATE_DISABLED,
  STATE_IDLE,
  STATE_RUNNING,
  STATE_ERROR
} SystemState;

volatile SystemState systemState      = STATE_DISABLED;
volatile uint8_t     startPending     = 0;   


#define LED_DDR       DDRA
#define LED_PORT      PORTA

#define LED_DISABLED_BIT  PA0   // pin 22 - Yellow
#define LED_IDLE_BIT      PA1   // pin 23 -Green
#define LED_ERROR_BIT     PA2   // pin 24 -Red
#define LED_RUNNING_BIT   PA3   // pin 25 - Blue
#define BUTTON_DDR       DDRC
#define BUTTON_PORT      PORTC
#define BUTTON_PIN_REG   PINC
#define BUTTON_STOP_BIT  PC7      
#define BUTTON_RESET_BIT PC6      
#define BUTTON_START_BIT PE4      

// Water sensor 
static const unsigned char WATER_CHANNEL   = 0;    
static unsigned int        WATER_THRESHOLD = 200;  

volatile unsigned char *adcMux    = (unsigned char*) 0x7C; 
volatile unsigned char *adcCtrlB  = (unsigned char*) 0x7B; 
volatile unsigned char *adcCtrlA  = (unsigned char*) 0x7A; 
volatile unsigned int  *adcData   = (unsigned int  *) 0x78; 

void initAdc(void);
unsigned int readAdc(unsigned char channel);
uint8_t isWaterLow(void);

void setupPins(void);
void updateLeds(void);
void handleButtonsSensorsAndState(void);
void setState(SystemState newState);


void onStartButton()
{
  
  startPending = 1;
}

void setup()
{
  setupPins();
  initAdc();   
  attachInterrupt(digitalPinToInterrupt(2), onStartButton, FALLING);
  systemState = STATE_DISABLED;
  updateLeds();
}

void loop()
{
  handleButtonsSensorsAndState();
}
void setupPins(void)
{
  
  LED_DDR |= (1 << LED_DISABLED_BIT) |
             (1 << LED_IDLE_BIT)     |
             (1 << LED_ERROR_BIT)    |
             (1 << LED_RUNNING_BIT);

  // All LEDs off 
  LED_PORT &= ~((1 << LED_DISABLED_BIT) |
                (1 << LED_IDLE_BIT)     |
                (1 << LED_ERROR_BIT)    |
                (1 << LED_RUNNING_BIT));

  
  BUTTON_DDR  &= ~((1 << BUTTON_STOP_BIT) | (1 << BUTTON_RESET_BIT)); // inputs
  BUTTON_PORT |=  (1 << BUTTON_STOP_BIT) | (1 << BUTTON_RESET_BIT);   // pull-ups

  
  DDRE  &= ~(1 << BUTTON_START_BIT);
  PORTE |=  (1 << BUTTON_START_BIT);
}

// LED
void updateLeds(void)
{
  // All off
  LED_PORT &= ~((1 << LED_DISABLED_BIT) |
                (1 << LED_IDLE_BIT)     |
                (1 << LED_ERROR_BIT)    |
                (1 << LED_RUNNING_BIT));

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

// State change
void setState(SystemState newState)
{
  if (systemState != newState) {
    systemState = newState;
    updateLeds();
  }
}


void handleButtonsSensorsAndState(void)
{
  // Buttons
  uint8_t buttonPins   = BUTTON_PIN_REG;
  uint8_t stopPressed  = !(buttonPins & (1 << BUTTON_STOP_BIT));   
  uint8_t resetPressed = !(buttonPins & (1 << BUTTON_RESET_BIT));  

  
  if (startPending) {
    startPending = 0;

    if (systemState == STATE_DISABLED) {
      setState(STATE_IDLE);
    }
   
  }

  
  if (stopPressed) {
    setState(STATE_DISABLED);
    return;   
  }

  /
  if (resetPressed && systemState == STATE_ERROR) {
    
    if (!isWaterLow()) {
      setState(STATE_IDLE);
    }
  }

  
  if (systemState == STATE_IDLE || systemState == STATE_RUNNING) {
    if (isWaterLow()) {
      setState(STATE_ERROR);
    }
  }

 
}



void initAdc(void) 
{
 
  *adcCtrlA = (1 << 7)               
            | (0 << 5)               
            | (0 << 3)               
            | (1 << 2) | (1 << 1) | (1 << 0); 


  *adcCtrlB = (0 << 3)               
            | (0 << 2) | (0 << 1) | (0 << 0); 

  
  *adcMux  = (0 << 7)                
           | (1 << 6)                
           | (0 << 5)             
           | (0x00);                 
}

unsigned int readAdc(unsigned char channel)
{
  channel &= 0x07;   

  
  *adcMux = (*adcMux & 0xE0) | (channel & 0x1F);

  
  *adcCtrlB &= ~(1 << 3); 

  
  *adcCtrlA |= (1 << 6);


  while ((*adcCtrlA & (1 << 6)) != 0) {
    
  }
  unsigned int value = *adcData;
  return value;
}

uint8_t isWaterLow(void)
{
  unsigned int reading = readAdc(WATER_CHANNEL);


  if (reading <= WATER_THRESHOLD) {
    return 1;   
  } else {
    return 0;  
  }
}
