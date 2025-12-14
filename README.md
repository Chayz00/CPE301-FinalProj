CPE301-FinalProj
Group#: 43
Members: Chayton Zuniga & Inshallah Baca 

Project Overview: 
  This project implments a swamp cooler controller using an Adruino AtMega2560. It monitors the temperature, humidity, and water level. Using these readings and user input it controls a fan, a set of state status LEDs, a LCD, and a stepper motor controled via a potentiometer. We use a RTC to timestamp the status log and state transitions. The water level sensor is read using a direct register level ADC. 

  The controller uses a four states: 
          Disabled - System off, LED on, no monitoring, no fan, no stepper motor. 
          Idle - System ready, monitoring on, stepper on, fan off. 
          Error - Low water fan forced off, forced idle off. 
          Running - Fan on system cooling. 

  Buttons are used for state transitions as well as the water & temperature/humidity sensors: 
      Start: External interrupt (pin 2) 
      Stop: with software debounce
      Reset: Recovers from Error when water level fixed
      Water level: ADC 
      Temperature/humidity: DTH11 

Hardware and Pin assignment:
Microcontroller: Arduino AtMega2560

Inputs:
Start button: D2 (INT4, INPUT_PULLUP, falling-edge interrupt)
Stop button: D30 (INPUT_PULLUP, debounced)
Reset button: D31 (INPUT_PULLUP)
DHT11 temperature/humidity: D7
Vent potentiometer: A1, uses analogRead() only for optional vent/stepper control
Water-level sensor: A0 (WATER_ADC_CHANNEL = 0), read via direct ADC registers                          (my_ADMUX, my_ADCSRA, my_ADCSRB, my_ADC_DATA)



Outputs
Status LEDs
Disabled (yellow): D22
Idle (green): D23
Error (red): D24
Running (blue): D25
Fan control: D6 -> PN2222 transistor -> fan (ON only in RUNNING)
LCD 16×2: LiquidCrystal lcd(8, 9, 10, 11, 12, 13)
Vent stepper: IN1–IN4 on D32, D33, D34, D35
RTC DS3231: used to prefix serial logs.


States: STATE_DISABLED, STATE_IDLE, STATE_RUNNING, STATE_ERROR

Transitions:
Start button: DISABLED → IDLE
Stop button: any state → DISABLED (fan OFF, monitoring OFF)
Water low (waterAdc < 200): IDLE/RUNNING -> ERROR
Reset button: ERROR -> IDLE if water level OK
Temperature (DHT11, °C): currentTempC >= 26.0 -> STATE_RUNNING, fan ON
                         currentTempC <= 24.0 ->  STATE_IDLE, fan OFF


Build & Run Instructions:
1) Install libraries: RTClib, DHT sensor library (plus built-in LiquidCrystal and Stepper).
2)Open the .ino in Arduino IDE, select Arduino Mega 2560 and correct Port.
3) Wire hardware according to schematic in report.
4) Upload and open Serial Monitor at 9600 baud.

Operation demo:
1) Power on -> DISABLED (yellow LED, “Cooler OFF Start”).
2) Press Start -> IDLE, monitoring enabled.
3) Raise temp >= 26C (water OK) ->  STATE_RUNNING, fan ON.
4) Cool to <= 25C -> back to IDLE, fan OFF.
5) Lower water below threshold -> ERROR, fan OFf. Fix water and press Reset to         return to IDLE.
6) Adjust pot on A1 to move vent stepper while system is active.
