/*
Optimized for:
ATtiny85
8Mhz
Program via Arduino:
1. To program the ATtiny85 we need to first set Arduino Uno in ISP mode. Connect your Arduino Uno to the PC. Open Arduino IDE and open the ArduinoISP example file (File -> Examples -> ArduinoISP) and upload it.
Disable all the wiring...
2. Wire up ATtiny
3. Select ATtiny85, 8MHz internal -> burn Bootloader
*/
/**********************************************************************/
/* INCLUDES                                                           */
/**********************************************************************/
#include <avr/interrupt.h>     
#include "tone.h"
#include "servo.h"

/**********************************************************************/
/* DEFINES                                                            */
/**********************************************************************/
#define PIN_POTI                3 /* A3 */
#define PIN_START               4 /* A2 */
#define PIN_SERVO               1 /* D1 */
#define PIN_LED                 0 /* D0 */
#define PIN_SPEAKER             2 /* A1 */

#define MAX_TEA_TIME            600000  /* 780 000 = 13 min ; 600 000 ms = 10 min */
#define WARNING_STILL_ON_AFTER  50000   /* go to sleep after this time (in ms 30 000 ms = 30 s) */

#define WAKE_UP_BLINK           3       /* wake up */
#define WAKE_UP_FREQUENCY       200     /* in ms */ 

#define TEA_START_BLINK         3   
#define TEA_START_FREQUENCY     300

#define DURING_BREWING_FADE_PERIODE 5000.0

#define END_BREWING_SHAKING     3       /* number of shaking the teabag */
#define END_BREWING_SHAKE_DISTANCE 10   /* in degree */
#define END_BREWING_SHAKE_POSITIONING_TIME 300 /* in ms */
#define SERVO_POSITIONING_TIME  1600    /* in ms */
#define TEA_FINISHED_BLINK      5

enum TeaStateEnum { TEA_POWER_ON, TEA_START_PRESSED, TEA_BREWING, TEA_FINISHED, TEA_WARNING };

/**********************************************************************/
/* GLOBAL VARIABLES                                                   */
/**********************************************************************/
volatile TeaStateEnum _state;

uint32_t _state_time, _state_start;
uint32_t _set_time;
uint32_t melody[] = {c, R, c, R};
uint32_t beats[] = {32, 32, 32, 128};

/**********************************************************************/
/* LOCAL FUNCTIONS                                                    */
/**********************************************************************/
ISR(PCINT0_vect) {
  if (digitalRead(PIN_START)) {  /* is start pressed? */
    //delay(5); /* quick hack for debounce */
    if (_state == TEA_BREWING) { 
      _state = TEA_FINISHED;
    } else {
      _state = TEA_START_PRESSED;
    }
  }
}

void reset_state_time() { 
  _state_start = millis(); 
}

uint32_t get_state_time() {
  return (millis() - _state_start);
}

uint32_t get_set_time() {
  float val = (analogRead(PIN_POTI) + analogRead(PIN_POTI) + analogRead(PIN_POTI)) / 3.0;
  val = (float)MAX_TEA_TIME * val / 1023.0;
  return (uint32_t)val;
}

void finish_sequence() {
  set_servo(SERVO_MAX, SERVO_POSITIONING_TIME);
  /* shake arm */
  for (uint8_t i = 0; i < END_BREWING_SHAKING; i++) {
    set_servo(SERVO_MAX, END_BREWING_SHAKE_POSITIONING_TIME);
    set_servo(SERVO_MAX - END_BREWING_SHAKE_DISTANCE, END_BREWING_SHAKE_POSITIONING_TIME);
  }
  set_servo(SERVO_MAX, END_BREWING_SHAKE_POSITIONING_TIME);
  
  for (uint8_t l = 0; l < TEA_FINISHED_BLINK; l++) {
    play_song(melody, beats, sizeof(melody)/sizeof(melody[0]));
  }
}

void blink_led(uint8_t num, uint16_t frequency) {
  for (uint8_t i = 0; i < num; i++){
    digitalWrite(PIN_LED, HIGH);
    delay(frequency);
    digitalWrite(PIN_LED, LOW);
    delay(frequency);
  }
}

/* offset + amplitue*cos(w*t + phase)*/
void pulse_led() {
  const int val = 120 + 120 * sin(((2.0 * PI) / DURING_BREWING_FADE_PERIODE) *  millis());
  analogWrite(PIN_LED, constrain(val, 0, 255));
}

/**********************************************************************/
/* SETUP                                                              */
/**********************************************************************/
void setup() {  
  _state = TEA_POWER_ON;
  
  pinMode(PIN_POTI, INPUT);
  pinMode(PIN_START, INPUT);
  pinMode(PIN_LED, OUTPUT);
  
  init_tone(PIN_SPEAKER, PIN_LED);
  init_servo(PIN_SERVO);
  
  /* power on visualization */    
  blink_led(WAKE_UP_BLINK, WAKE_UP_FREQUENCY);
  
  /* use arduino function instead:  */
  GIMSK = 0b00100000;   /* turns on pin change interrupts */
  PCMSK = 0b00010000;   /* turn on interrupts on pin PB4 */
  sei();                /* enables interrupts (bzw. _SEI();) */
}

/**********************************************************************/
/* LOOP                                                               */
/**********************************************************************/
void loop() {
  switch(_state) {
    case TEA_BREWING:
      pulse_led();
      if (get_state_time() > get_set_time())
        _state = TEA_FINISHED;
    break;
    case TEA_START_PRESSED:
      reset_state_time();
      blink_led(TEA_START_BLINK, TEA_START_FREQUENCY); /* short delay and blink */
      set_servo(SERVO_MIN, SERVO_POSITIONING_TIME);
      _state = TEA_BREWING;
    break;
    case TEA_FINISHED: 
      finish_sequence();
      _state = TEA_POWER_ON;
    break;
    case TEA_POWER_ON:
    default:
      digitalWrite(PIN_LED, HIGH); /* show led that machine is power on */
      if (get_state_time() > WARNING_STILL_ON_AFTER) {
        play_song(melody, beats, sizeof(melody)/sizeof(melody[0]));
        reset_state_time();
      }
    break;
  }
}
