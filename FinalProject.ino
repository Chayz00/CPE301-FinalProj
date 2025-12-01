#include <avr/io.h>
#include <avr/interrupt.h>

// States 
typedef enum {
  STATE_DISABLED,
  STATE_IDLE,
  STATE_RUNNING,
  STATE_ERROR
} SystemState;

volatile SystemState systemState = STATE_DISABLED;
volatile uint8_t     startPending = 0;   


#define LED_DDR       DDRA
#define LED_PORT      PORTA
#define LED_DISABLED_BIT  PA0   // pin 22 - Yellow
#define LED_IDLE_BIT      PA1   // pin 23 - Green
#define LED_ERROR_BIT     PA2   // pin 24 - Red
#define LED_RUNNING_BIT   PA3   // pin 25 - Blue



#define BUTTON_DDR       DDRC
#define BUTTON_PORT      PORTC
#define BUTTON_PIN_REG   PINC
#define BUTTON_STOP_BIT  PC7      
#define BUTTON_RESET_BIT PC6      


#define BUTTON_START_BIT PE4      


void setupPins(void);
void updateLeds(void);
void handleButtonsAndState(void);
void setState(SystemState newState);


void onStartButton()
{
 
  startPending = 1;
}

void setup()
{
  setupPins();
  attachInterrupt(digitalPinToInterrupt(2), onStartButton, FALLING);
  systemState = STATE_DISABLED;
  updateLeds();
}
void loop()
{
  handleButtonsAndState();
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

  
  BUTTON_DDR  &= ~((1 << BUTTON_STOP_BIT) | (1 << BUTTON_RESET_BIT)); 
  BUTTON_PORT |=  (1 << BUTTON_STOP_BIT) | (1 << BUTTON_RESET_BIT);   

  
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


void handleButtonsAndState(void)
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

  // Stop -> DISABLED
  if (stopPressed) {
    setState(STATE_DISABLED);
    return;   
  }

  // Reset 
  if (resetPressed && systemState == STATE_ERROR) {
    setState(STATE_IDLE);
  }

 
}

