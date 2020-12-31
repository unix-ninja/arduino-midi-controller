// /Applications/Arduino.app/Contents/Java/hardware/teensy/avr/cores/teensy3/usb_desc.h

#include <limits.h>
#include <stdbool.h>

#define NOTE_ON true
#define NOTE_OFF false
#define DSB 1 // Serial in for SIPO 164
#define PL 2 // PL for PISO 165
#define CE 11 // enable for PISO
#define QH 12 // Serial out from PISO 165
#define LED 13
#define CLOCK_164 14
#define CLOCK_165 15
#define POT_PANEL 23 // multiplexer output
#define POT_PITCH 22 // pitch wheel output
#define POT_MOD 21 // mod wheel output
#define VCC_MONITOR 20 // use this to monitor supply voltage
#define POT_HIST_RES 40
//#define POT_ENABLE 5 // Enable pin for multiplexer

struct keys
{
  bool deep_state;
  unsigned int deep_time;
  bool soft_state;
  unsigned int soft_time;
};

typedef struct keys keys_t;

keys_t press[49];
int pots[18];
int potS[18][POT_HIST_RES];
int poti[18] = {0};
int pitch_wheel(0);
int mod_wheel(0);
int pot_channel[4] = {4,5,6,7};
//int pot_loop(0);

void setup()
{
  Serial.begin(38400);
  pinMode(DSB, OUTPUT);
  pinMode(PL, OUTPUT);
  pinMode(QH, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(CLOCK_164, OUTPUT);
  pinMode(CLOCK_165, OUTPUT);

  pinMode(pot_channel[0], OUTPUT);
  pinMode(pot_channel[1], OUTPUT);
  pinMode(pot_channel[2], OUTPUT);
  pinMode(pot_channel[3], OUTPUT);

  digitalWrite(LED, HIGH);

  // fill pot arrays
  for (int i=0; i< 15; i++)
  {
    for (int ii=0; ii< POT_HIST_RES; ii++)
    {
      potS[i][ii] =  map(floor(analogRead(POT_PANEL)/10), 4, 100, 0, 63);
      potS[i][ii] = floor(potS[i][ii] + (potS[i][ii] * 0.08));
      if (potS[i][ii] < 0) potS[i][ii] = 0;
      if (potS[i][ii] > 127) potS[i][ii] = 127;
    }
  }
}

void toggle_key(int key, bool state, bool deep)
{
  unsigned int *keypress;
  bool *keystate;
  unsigned int time = millis();
  int channel = 1;
  int velocity = 0;
  int octave = 3 * 12;
  
  if (deep)
  {
    keypress = &(press[key-1].deep_time);
    keystate = &(press[key-1].deep_state);
  } else {
    keypress = &(press[key-1].soft_time);
    keystate = &(press[key-1].soft_state);
  }
  
  if (state == NOTE_ON)
  {
    if (*keystate == NOTE_OFF)
    {
      *keypress = time;
      *keystate = NOTE_ON;
      if (deep)
      {
        // calculate velocity here
        if (press[key-1].soft_time > *keypress)
        {
          time = (UINT_MAX - *keypress) + press[key-1].soft_time;
        } else {
          time = *keypress - press[key-1].soft_time;
        }
        if (time < 16)
        {
          velocity = 127;
        }
        else if (time > 310)
        {
          velocity = 12;
        }
        else
        {
          velocity = 120 - (time / 2.4);
        }
        usbMIDI.sendNoteOn(octave + key, velocity, channel);
      }      
    }
  } else {
    if (*keystate == NOTE_ON)
    {
      if (deep)
      {
        usbMIDI.sendNoteOff(octave + key, 0, channel);
      }
      *keystate = NOTE_OFF;
    }
  }
}

void shiftInKeys(int offset)
{
  int bucket;
  for (int i = 1; i <= 8; ++i) {
    int key = offset + ((((((9-i)%2)+(9-i))/2)-1)*8);
    digitalWrite(CLOCK_165, LOW);
    bucket = digitalRead(QH);
    if (1 <= key && key <= 49)
    {
      toggle_key(key, (bucket ? true : false), (i%2 == 0 ? true : false));
    }
    digitalWrite(CLOCK_165, HIGH);
  }
}

void readPot(int pot)
{
  int channel = 1;
  int num = pot;
  int potentiometer = 0;

  // set multiplexer channel
  for (int i=0; i<4; i++)
  {
    if (num & 1)
    {
      digitalWrite(pot_channel[i], HIGH);
    } else {
      digitalWrite(pot_channel[i], LOW);
    }
    num >>= 1;
  }

  // read multiplexer channel
  float supply_step;
  supply_step = 8.3;
  potentiometer = map(floor(analogRead(POT_PANEL)/10), 4, 100, 0, 63);
  potentiometer = floor((potentiometer) + (potentiometer * 0.08));
  if (potentiometer < 0) potentiometer = 0;
  if (potentiometer > 127) potentiometer = 127;
  if(pots[pot])
  {
    potentiometer = 0.8 * pots[pot] + 0.2 * potentiometer;
  }
  if (pots[pot] != potentiometer)
  {
    {
      potS[pot][poti[pot]] = potentiometer;
      if(poti[pot] < POT_HIST_RES)
      {
        poti[pot]++;
      } else {
        poti[pot] = 0;
      }
    } 

    int sum;
    for (int i=0; i<POT_HIST_RES; i++)
    {
      sum += potS[pot][i];
    }
    sum /= POT_HIST_RES;
    
    if (sum != pots[pot])
    {
      pots[pot] = sum;

      sum *= 2;
      sum = 0.8 * (pots[pot]*2) + 0.2 * sum;
      sum *= 0.97692;
      usbMIDI.sendControlChange(14 + pot, sum, channel);
      // TODO: send MIDI here
    }
  }
}

void loop()
{
  int offset = -11;
  for (int i=1; i<=8; i++)
  {
    if (i == 1)
    {
      digitalWrite(DSB, HIGH);
    } else {
      offset++;
    }
    digitalWrite(CLOCK_164, HIGH);
    digitalWrite(PL, LOW);
    digitalWrite(DSB, LOW);
//    delay(500);
    digitalWrite(PL, HIGH);
    digitalWrite(CLOCK_164, LOW);

    // shift first byte
    shiftInKeys(offset);

    // shift second byte
    shiftInKeys(offset+32);

    // read first pot row
    readPot(i - 1);

    // read second pot row
    readPot(i - 1 + 8);
  }

  int potentiometer;
  // for now, hard code channel to 1
  int channel = 1;
  
  // read pitch wheels
  potentiometer = (analogRead(POT_PITCH));
  potentiometer = floor(potentiometer / 3) * 3;
  potentiometer = (potentiometer - 320);
  if(potentiometer == 231 || potentiometer == 233) potentiometer = 232;
  potentiometer -= 28;
  if (potentiometer < 0) potentiometer = 0;
  potentiometer = 0.6 * pitch_wheel + 0.4 * potentiometer;

    int mid_min = 171;
    int mid_max = 176;
    int mid_norm = 174;
    
    if (potentiometer > (mid_min-1) && potentiometer < (mid_max+1))
    {
      potentiometer = mid_norm;
    }
    
  if (pitch_wheel != potentiometer)
  { 
    pitch_wheel = potentiometer;
    
    // let's adjust the output here before sending.
    // the value is unstable when adjusting the global
    potentiometer *= 41;
    if ( potentiometer == (mid_norm * 41)) potentiometer = 8192;
    if (potentiometer > 16382) potentiometer = 16382;
    if (potentiometer < -8192) potentiometer = -8192;

    // override pitch wheel
    potentiometer = 0;
    
    // pitch wheel is cc 1
//    usbMIDI.sendPitchBend(potentiometer, channel);
//    Serial.println("pot[pitch]: "+String(potentiometer));
  }
  
  // read mod wheels
  potentiometer = (analogRead(POT_MOD) - 330) / 2.89;
  potentiometer = 0.8 * mod_wheel + 0.2 * potentiometer;
  if (potentiometer < 0) potentiometer = 0;
  if (potentiometer > 127) potentiometer = 127;
  if (mod_wheel != potentiometer)
  {
    mod_wheel = potentiometer;
    // mod wheel is cc 1
    usbMIDI.sendControlChange(1, mod_wheel, channel);
  }
}
