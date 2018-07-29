/*
Optimized for:
ATtiny85
8Mhz
Program via Arduino:
1. To program the ATtiny85 we need to first set Arduino Uno in ISP mode. Connect your Arduino Uno to the PC. Open Arduino IDE and open the ArduinoISP example file (File -> Examples -> ArduinoISP) and upload it.
Disable all the wiring...
2. Wire up ATtiny
3. Select ATtiny85, 8MHz internal -> Bootloader brennen
4. 
*/
#include <SoftwareServo.h>     
//#include <avr/sleep.h>           
#include <avr/interrupt.h>     

#define PIN_POTI  3 //A3
#define PIN_START 4 //A2
#define PIN_SERVO 1 //D1
#define PIN_LED   0 //D0
#define PIN_SPEAKER 2 //A1

#define SERVO_MIN 45
#define SERVO_MAX 135
#define SERVO_POSITIONING_TIME 1600  //in ms

#define MAX_TEA_TIME 600000 // 780 000 = 13 min ; 600 000 ms = 10 min
#define WARNING_STILL_ON_AFTER 50000   //go to sleep after this time (in ms 30 000 ms = 30 s)

#define WAKE_UP_BLINK 3 //wake up
#define WAKE_UP_FREQUENCY 400 //in ms

#define TEA_START_BLINK 3
#define TEA_START_FREQUENCY 300

#define DURING_BREWING_FADE_PERIODE 5000.0

#define END_BREWING_SHAKING 3 //number of shaking the teabag
#define END_BREWING_SHAKE_DISTANCE 10 //in degree
#define END_BREWING_SHAKE_POSITIONING_TIME 300 //in ms

#define TEA_FINISHED_BLINK 5

// DEFINITION OF TONES  ==========================================
//       note, period, &  frequency. 
#define  c     3830    // 261 Hz 
#define  d     3400    // 294 Hz 
#define  e     3038    // 329 Hz 
#define  f     2864    // 349 Hz 
#define  g     2550    // 392 Hz 
#define  a     2272    // 440 Hz 
#define  b     2028    // 493 Hz 
#define  C     1912    // 523 Hz 
#define  R     0       // to represent a rest
//int melody[] = {c, R, c, R};
//int beats[]  = { 32, 32, 32, 128}; // 32 => 320ms
int melody[] = {c, R, c, R};
int beats[]  = { 32, 32, 32, 128}; // 32 => 320ms

int MAX_COUNT = sizeof(melody) / 2; // Melody length, for looping. (2 byte)
long tempo = 10000; // Set overall tempo
int pause = 2000; // // Set length of pause between notes
// Loop variable to increase Rest length
int rest_count = 1;//0;//00; //<-BLETCHEROUS HACK; See NOTES
// Initialize core variables
int tone_ = 0;
int beat = 0;
long duration  = 0;
int last_angle = 0; /*for servo*/
//GLOBAL VARIABLES ==============================================
float tea_time, time_elapsed;
bool tea_brewing, cancel_tea_brewing_b;
bool toggle = false; /*for blinking while playing tone*/
uint32_t time_tea_started;
uint8_t mcucr1, mcucr2, led_brightness, i;
SoftwareServo servo;
volatile bool start_tea_b;

// PLAY TONE  ===================================================
void play_tone() {
  long elapsed_time = 0;
  if (tone_ > 0) { // if this isn't a Rest beat, while the tone has 
    //  played less long than 'duration', pulse speaker HIGH and LOW
    while (elapsed_time < duration) {
      digitalWrite(PIN_SPEAKER,HIGH);
      delayMicroseconds(tone_ / 2);
      digitalWrite(PIN_SPEAKER, LOW);
      delayMicroseconds(tone_ / 2);
      // Keep track of how long we pulsed
      elapsed_time += (tone_);
    } 
  }
  else { // Rest beat; loop times delay
    for (int j = 0; j < rest_count; j++) { // See NOTE on rest_count
      delay(duration/1000);
      //delayMicroseconds(duration);  
    }                                
  }                                 
}

void play_song() {
  for (int i=0; i<MAX_COUNT; i++) {
    tone_ = melody[i];
    beat = beats[i];
    duration = beat * tempo; // Set up timing
    /*blink with led while playing*/
    digitalWrite(PIN_LED, toggle);
    toggle = !toggle;
    play_tone(); 
    delayMicroseconds(pause);
  }
}

void setServo(uint8_t degree, uint32_t duration) {
  int diff = degree - last_angle;
  diff = abs(diff);
  int w = (float)duration / (float)diff * 0.5 * 1000.0;
  if(last_angle < degree) {
    for (float k = last_angle; k < degree; k+=0.5) {
      servo.write(k); 
      delayMicroseconds(w);
      SoftwareServo::refresh(); 
    }
  } else if (last_angle > degree) {
    for (float k = last_angle; k > degree; k-=0.5) {
      servo.write(k);
      delayMicroseconds(w); 
      SoftwareServo::refresh(); 
    }
  }
  last_angle = degree;
}

// BLINK STATUS LED ==============================================
void blink_led(int num, uint16_t frequency) {
  for(i = 0; i<num; i++){
    digitalWrite(PIN_LED, HIGH);
    delay(frequency);
    digitalWrite(PIN_LED, LOW);
    delay(frequency);
  }
}

// SETUP =========================================================
void setup() {  
  tea_time = 0;
  //start = false;
  tea_brewing = false;
  time_elapsed = 0;
  time_tea_started = 0;
  start_tea_b = false;
  cancel_tea_brewing_b = false;
  pinMode(PIN_POTI, INPUT);
  pinMode(PIN_START, INPUT);
  pinMode(PIN_SERVO, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_SPEAKER, OUTPUT);
  
  servo.attach(PIN_SERVO);
  servo.setMaximumPulse(2100);
  servo.setMinimumPulse(550);
  servo.write(SERVO_MAX);              
  for(int i=0; i<600/20; i++) { //stellzeit
    SoftwareServo::refresh();  
    delay(20); 
  }
  last_angle = SERVO_MAX;
  
  //start_time = millis();
  blink_led(WAKE_UP_BLINK, WAKE_UP_FREQUENCY);
  digitalWrite(PIN_LED, HIGH);
  ////cli(); 
  ////General Interrupt Mask Register, External Interrupts: 0b01000000; Pin Change Interrupts: 0b00100000
  //GIMSK = 0b00100000;    // turns on pin change interrupts
  ////Pin Change Mask Register 
  //PCMSK = 0b00010000;    // turn on interrupts on pin PB4
  //sei();                 // enables interrupts (bzw. _SEI();)
  //attachInterrupt(PIN_START, tea_start_brewing, RISING); /* is not working probably*/
  GIMSK = 0b00100000;    // turns on pin change interrupts
  PCMSK = 0b00010000;    // turn on interrupts on pin PB4
  sei();
}

void loop() {
  
  time_elapsed = millis() - time_tea_started;
  
  if (tea_brewing) {  
    during_brewing();
    // Check if tea is finished
    if (time_elapsed > tea_time || cancel_tea_brewing_b) {
      tea_finished();    
      cancel_tea_brewing_b = false;
    }
  } else {
    if (time_elapsed > WARNING_STILL_ON_AFTER) {
      /*for(int l = 0; l<TEA_FINISHED_BLINK; l++) {*/
        play_song();
        digitalWrite(PIN_LED, HIGH);
        //time_elapsed = 0;
        time_tea_started = millis();
      //}
    }
  }

  if (start_tea_b) {
    tea_start_brewing();
    start_tea_b = false;
  }
}

void during_brewing() {
  /* offset + amplitue*cos(w*t + phase)*/
  //int ledValue = 120 + 127*(cos(OMEGA*timer)+PHASE);
  //analogWrite(PIN_LED, 128 + 127 * sin(((2.0 * PI) / DURING_BREWING_FADE_PERIODE) *  millis()));
  // Useful to avoid LED values outside the bounds [0;255]
  int val = 120 + 120 * sin(((2.0 * PI) / DURING_BREWING_FADE_PERIODE) *  millis());
  if (val > 255) val = 255;
  else if(val < 0) val = 0;
  
  analogWrite(PIN_LED, val);
}
float readTeaTime()
{
  return ((float)MAX_TEA_TIME * analogRead(PIN_POTI) / 1023.0);
}

void tea_start_brewing() {
  blink_led(TEA_START_BLINK, TEA_START_FREQUENCY); /* short delay and blink */
  
  tea_brewing = true; //set tea making flag
  /*get current time value in ms from potentiometer*/
  // tea_time = (float)MAX_TEA_TIME - (float)MAX_TEA_TIME * analogRead(PIN_POTI) / 1023.0;
  tea_time = readTeaTime() + readTeaTime() + readTeaTime(); /* for stabilize reading, read multiple times */
  tea_time /= 3.0;
  
  setServo(SERVO_MIN, SERVO_POSITIONING_TIME);
  time_tea_started = millis();  //start time measure
}

void tea_finished() {
  tea_brewing = false;
  setServo(SERVO_MAX, SERVO_POSITIONING_TIME);
  /*shake arm*/
  for (i = 0; i < END_BREWING_SHAKING; i++) {
    setServo(SERVO_MAX, END_BREWING_SHAKE_POSITIONING_TIME);
    setServo(SERVO_MAX - END_BREWING_SHAKE_DISTANCE, END_BREWING_SHAKE_POSITIONING_TIME);
  }
  setServo(SERVO_MAX, END_BREWING_SHAKE_POSITIONING_TIME);
  
  for(int l = 0; l<TEA_FINISHED_BLINK; l++) {
    play_song();
  }
  /*after finished tea brewing show led that machine is still on*/
  digitalWrite(PIN_LED, HIGH);
  /*reset elapsed time*/
  time_elapsed = 0.0; 
}


ISR(PCINT0_vect)
{
  if (digitalRead(PIN_START)) {  // is start pressed?
    if(tea_brewing) cancel_tea_brewing_b = true;
    else start_tea_b = true;
  }
}

