#include <SoftwareServo.h>
#include <avr/sleep.h>           
#include <avr/interrupt.h>       

#define PIN_POTI  3 //A3
#define PIN_START 4 //A2
#define PIN_SERVO 1 //D1
#define PIN_LED   0 //D0
#define PIN_SPEAKER 2 //A1

#define SERVO_MIN 45
#define SERVO_MAX 135
#define SERVO_POSITIONING_TIME 1600  //in ms

#define MAX_TEA_TIME 780000 // 780 000 = 13 min ; 600 000 ms = 10 min
#define SLEEP_AFTER 20000   //go to sleep after this time (in ms 30 000 ms = 30 s)

#define WAKE_UP_BLINK 2 //wake up or reset
#define WAKE_UP_FREQUENCY 400 //in ms

#define TEA_START_BLINK 3
#define TEA_START_FREQUENCY 300

#define END_BREWING_SHAKING 3 //number of shaking the teabag
#define END_BREWING_SHAKE_DISTANCE 10 //in degree
#define END_BREWING_SHAKE_POSITIONING_TIME 300 //in ms

#define TEA_FINISHED_BLINK 7

#define BODS 7                   //BOD Sleep bit in MCUCR
#define BODSE 2                  //BOD Sleep enable bit in MCUCR

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
int melody[] = {  c, R, c, R};
int beats[]  = { 32, 32, 32, 128}; // 32 => 320ms

int MAX_COUNT = sizeof(melody) / 2; // Melody length, for looping. (2 byte)
long tempo = 10000; // Set overall tempo
int pause = 2000; // // Set length of pause between notes
// Loop variable to increase Rest length
int rest_count = 100; //<-BLETCHEROUS HACK; See NOTES
// Initialize core variables
int tone_ = 0;
int beat = 0;
long duration  = 0;

//GLOBAL VARIABLES ==============================================
float tea_time, elapsed_time;
bool start, tea_brewing;
uint32_t start_time; 
uint8_t mcucr1, mcucr2, led_brightness, i;
SoftwareServo servo;

// PLAY TONE  ===================================================
void playTone() {
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
      delayMicroseconds(duration);  
    }                                
  }                                 
}

// PLAY SONG ===================================================
bool toggle = false;
void playSong() {
  for (int i=0; i<MAX_COUNT; i++) {
    tone_ = melody[i];
    beat = beats[i];
    duration = beat * tempo; // Set up timing
    // Blink LED
    digitalWrite(PIN_LED, toggle);
    toggle = !toggle;
    
    playTone(); 
    delayMicroseconds(pause);
  }
}

// GO TO DEEP SLEEP ==============================================
void goToSleep(void) {
    // GIMSK |= _BV(INT0);                    //enable INT0
    // MCUCR &= ~(_BV(ISC01) | _BV(ISC00));   //INT0 on low level
    ACSR |= _BV(ACD);                         //disable the analog comparator
    ADCSRA &= ~_BV(ADEN);                     //disable ADC
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    //turn off the brown-out detector.
    //must have an ATtiny45 or ATtiny85 rev C or later for software to be able to disable the BOD.
    //current while sleeping will be <0.5uA if BOD is disabled, <25uA if not.
    cli();
    mcucr1 = MCUCR | _BV(BODS) | _BV(BODSE);  //turn off the brown-out detector
    mcucr2 = mcucr1 & ~_BV(BODSE);
    MCUCR = mcucr1;
    MCUCR = mcucr2;
    sei();                         //ensure interrupts enabled so we can wake up again
    sleep_cpu();                   //go to sleep
    cli();                         //wake up here, disable interrupts
    // GIMSK = 0x00;               //disable INT0
    sleep_disable();               
    sei();                         //enable interrupts again (but INT0 is disabled from above)
}

// WRITE SERVO ANGLE  ============================================
int last_angle = 0;
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
  start = false;
  tea_brewing = false;
  elapsed_time = 0;

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
  
  start_time = millis();
  blink_led(WAKE_UP_BLINK, WAKE_UP_FREQUENCY);
  digitalWrite(PIN_LED, HIGH);
  //cli(); 
  //General Interrupt Mask Register, External Interrupts: 0b01000000; Pin Change Interrupts: 0b00100000
  GIMSK = 0b00100000;    // turns on pin change interrupts
  //Pin Change Mask Register 
  PCMSK = 0b00010000;    // turn on interrupts on pin PB4
  sei();                 // enables interrupts (bzw. _SEI();)
}

void loop() {
  if (tea_brewing) {
    elapsed_time = millis() - start_time;
    analogWrite(PIN_LED, 60);
    // Check if tea is finished
    if (elapsed_time > tea_time) {
      endRoutine();    
      digitalWrite(PIN_LED, LOW);  
      goToSleep(); 
    }
  } else {
    if (millis() - start_time > SLEEP_AFTER) {
      digitalWrite(PIN_LED, LOW); // TODO auskommentieren? müsste auch ohne aus gehen?! geht nicht aus!
      goToSleep(); 
    }
  }

  if (start) {
    blink_led(TEA_START_BLINK, TEA_START_FREQUENCY);
    startRoutine();
    start = false;
  }
}

//Start the tea procedure
void startRoutine() {
  tea_brewing = true; //set tea making flag
  //get current time value time in ms
  tea_time = (float)MAX_TEA_TIME - (float)MAX_TEA_TIME * analogRead(PIN_POTI) / 1023.0;
  setServo(SERVO_MIN, SERVO_POSITIONING_TIME);
  start_time = millis();  //start time measure
}

//End tea procedure
void endRoutine() {
  tea_brewing = false;
  setServo(SERVO_MAX, SERVO_POSITIONING_TIME);
  //shake arm
  for (i = 0; i < END_BREWING_SHAKING; i++) {
    setServo(SERVO_MAX, END_BREWING_SHAKE_POSITIONING_TIME);
    setServo(SERVO_MAX - END_BREWING_SHAKE_DISTANCE, END_BREWING_SHAKE_POSITIONING_TIME);
  }
  setServo(SERVO_MAX, END_BREWING_SHAKE_POSITIONING_TIME);
  for(int l = 0; l<TEA_FINISHED_BLINK; l++) {
    playSong();
  }
}

//Interrupt Service Routine
ISR(PCINT0_vect)
{
  if (digitalRead(PIN_START)) {  // is start pressed?
    start = true;
  }
}

