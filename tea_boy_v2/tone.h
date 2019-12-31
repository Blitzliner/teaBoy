/**********************************************************************/
/* VARIABLES FOR PLAYLING SOUND                                       */
/**********************************************************************/
/*       note, period, &  frequency. */
#define  c     3830    /* 261 Hz */ 
#define  d     3400    /* 294 Hz */
#define  e     3038    /* 329 Hz */
#define  f     2864    /* 349 Hz */
#define  g     2550    /* 392 Hz */
#define  a     2272    /* 440 Hz */
#define  b     2028    /* 493 Hz */
#define  C     1912    /* 523 Hz */
#define  R     0       /* to represent a rest */

#define SONG_TEMPO 10000 /* Set overall tempo */
#define NOTES_PAUSE 2000 /* Set length of pause between notes */
#define TONE_REST_LENGTH 1 /* Loop variable to increase Rest length */

int _pin_speaker, _pin_led;

// PLAY TONE  ===================================================
void init_tone(int pin_speaker, int pin_led) {
  _pin_speaker = pin_speaker;
  _pin_led = pin_led;
  
  pinMode(_pin_speaker, OUTPUT);
  pinMode(_pin_led, OUTPUT);
}

void play_tone(uint32_t tone, uint32_t duration) {
  long elapsed_time = 0;
  if (tone > 0) { /* if this isn't a Rest beat, while the tone has  */
    /* played less long than 'duration', pulse speaker HIGH and LOW */
    while (elapsed_time < duration) {
      digitalWrite(_pin_speaker,HIGH);
      delayMicroseconds(tone / 2);
      digitalWrite(_pin_speaker, LOW);
      delayMicroseconds(tone / 2);
      elapsed_time += (tone); /* Keep track of how long we pulsed */
    } 
  }
  else { /* Rest beat; loop times delay */
    for (uint8_t j = 0; j < TONE_REST_LENGTH; j++) { /* See NOTE on rest_count */
      delay(duration/1000.0);
    }                                
  }                                 
}

void play_song(uint32_t melody[], uint32_t beats[], uint8_t length) {
  bool toggle = false; /* for blinking while playing tone */
  
  for (uint8_t i = 0; i < length; i++) {
    digitalWrite(_pin_led, toggle); /* blink with led while playing */
    toggle = !toggle;
    play_tone(melody[i], (beats[i] * SONG_TEMPO)); 
    delayMicroseconds(NOTES_PAUSE);
  }
}
