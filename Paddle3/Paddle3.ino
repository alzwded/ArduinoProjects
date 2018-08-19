/*
  D5 -------------- POT -x
                    |
  D2 -- S1 -- R1 -- C1
                    |
  GND --------------+

  D7 -- X1 -- GND

  C1 = C2 = 4.7n
  R1 = R2 = 1k
  S1 = 74HC14

  Paddle controller:
  POT = 1M
  X1 = push button

  when POT is fully turned right, R(D2, C1) = 0Ohm

  Connect DB9:

                           -> PORT1
                           -> PORT2
  D7 --                    -> PORT3
  D4 --                    -> PORT4
  D2 -- R2 -- S12 -- C2 -- -> PORT5
                           -> PORT6
  D5 --                    -> PORT7
  GND -                    -> PORT8
  D3 -- R1 -- S12 -- C1 -- -> PORT9

  C1 -- GND
  C2 -- GND
  
  S1 - schmitt trigger
  5V -- S1Vcc
  GND - S1GND

*/

#include <avr/wdt.h>

// toggle between debug dump and actual device
#define SERIAL_DUMP 1
#define HID_DEVICE 2
#define OUTPUT SERIAL_DUMP

// time the math done when printing values
//#define SERIAL_TIMING
// time how long a loop takes; 
// disable other delay calls and sleeps for half a second each loop
//#define LOOP_TIMING

// apply a simple low pass filter
#define LOWPASS

// time to sleep at end of loop
static const int LOOP_DELTA = 12;

// paddle pin mapping
static const byte paddlePins[2] = {  2,  3 };
static const byte buttonPins[2] = {  4,  7 };
static const byte powerPin      =    5;

static const int overhead[3] = { 
  51,       /* low overhead;  ~5% */
  51,       /* high overhead; ~5% */
  +26,      /* nudge it to the left a bit */
};

// trigger bitfield set by interrupts
// bit1 - paddle A
// bit0 - paddle B
volatile byte triggered;
// paddles which were encountered
volatile byte encountered;
// interrupt writes here; 
// we could use paddles[] directly, but we want
// to do that only after a button was pressed
volatile int paddlesTmp[2];
int paddles[2], mins[2], maxs[2]; /* paddle state, 16bit */
#ifdef LOWPASS
// history-based low pass filter
float histories[2];
static const float LOWPASS_B = 0.9;
#endif

/* button state
   bit0   paddle left
   bit1   paddle right
   bit7   started acting as HID device

   I'd keep this as a global register allocation
   but arduino ide and gcc do weird things and
   end up dumping function definitions in front
   which break global register linking. 
   So... keep it here as a global variable (pff slow)
*/
byte buttons; /* button state */

void Paddle1Charged()
{
  paddlesTmp[0] = TCNT1;
  triggered |= 0x2;
}

void Paddle2Charged()
{
  paddlesTmp[1] = TCNT1;
  triggered |= 0x1;
}

inline bool Started()
{
  return buttons |= ( (buttons != 0) << 7 );
}

inline void MyPrint(byte rc)
{
#if OUTPUT == SERIAL_DUMP
# ifdef SERIAL_TIMING
  unsigned long ttt = micros();
# else
  Serial.print(paddles[rc]); Serial.print('|');
# endif
#elif OUTPUT == HID_DEVICE
#else
# error "OUTPUT mode not defined"
#endif
  // compute analog joystick range 0-1023
  
  // clamp from 10-1013, i.e. add a dead zone at the edges
  // ...or something like that. Basically give us a buffer
  int pvalue = paddles[rc] + overhead[2];
  if(pvalue < overhead[0] + mins[rc]) pvalue = mins[rc] + overhead[0];
  if(pvalue > maxs[rc] - overhead[1]) pvalue = maxs[rc] - overhead[1];
  pvalue -= mins[rc] + overhead[0];
  
#if 0
  Serial.print(pvalue);
  Serial.print("|");
  Serial.print(maxs[rc] - mins[rc]);
  Serial.print("|");
  Serial.print(overhead[1] + overhead[0]);
#endif
  
  // use FPU real quick for better math
  // normalize values to 0-1023
  float f = pvalue;
  f /= (maxs[rc] - mins[rc] - overhead[1] - overhead[0]);
  f *= 1023;
#ifdef LOWPASS
  histories[rc] = histories[rc] - LOWPASS_B * (histories[rc] - f);
  f = histories[rc];
#endif
  // round; we should be between 0-1023
  f += 0.5;
  int r16 = (int)f;
  // make sure we're actually between 0-1023
  r16 &= 0x03FF;

  // TODO rolling average to alleviate glitches.
  // i.e. add a low pass filter on the values.
  // we still have memory and cycles, right?
  
#if OUTPUT == SERIAL_DUMP
# ifdef SERIAL_TIMING
  Serial.print(micros() - ttt);
  Serial.print("us\t");
# endif
#endif

#if OUTPUT == SERIAL_DUMP
  static char buffer[] = "0000?\t";

  register unsigned char ra asm("r14");
  ra = 4;
  while (ra--) {
    register unsigned char rb = r16 % 10;
    buffer[ra] = 48 + rb;
    r16 /= 10;
  }
  buffer[4] = (' ' + ((buttons & (1 << rc)) != 0));

  Serial.print(buffer);
#elif OUTPUT == HID_DEVICE
  // TODO write as HID joystick
  // Right now it only dumps the state of the controls over USB serial.
#else
# error "Output not defined"
#endif
}

void setup()
{ 
  // trigger for RC networks
  pinMode(powerPin, OUTPUT);
  digitalWrite(powerPin, LOW);
  // intialize paddle pins
  for (register unsigned char rc = 0; rc < 2; ++rc) {
    pinMode(paddlePins[rc], INPUT_PULLUP);
    pinMode(buttonPins[rc], INPUT_PULLUP);
  }

  // state
  buttons = 0;
  paddles[0] = paddles[1] = 0;
  mins[0] = mins[1] = 100;
  maxs[0] = maxs[1] = 300;

  triggered = 0;
  encountered = 0;
  
  // attach faaling edge interrupt on the paddle pins
  // schmitt trigger keeps pins high until the rc network
  // rises to ~2.4V when it drops low
  attachInterrupt(digitalPinToInterrupt(paddlePins[0]), Paddle1Charged, FALLING);
  attachInterrupt(digitalPinToInterrupt(paddlePins[1]), Paddle2Charged, FALLING);

  // setup TCNT1
  TCCR1A = 0; // normal mode; no OC, no PWM
  /* CS12 CS11 CS10
     0    0    0      off
     0    0    1      no prescaling
     0    1    0      1/8
     0    1    1      1/64
     1    0    0      1/256
     1    0    1      1/1024
     1    1    0      ext, falling edge
     1    1    1      ext, rising edge
  */
  // 1/64 prescaler gives good values ~0..~1000
  TCCR1B = (1 << CS11) | (1 << CS10);
  TCCR1C = 0; // no force output compare

  TIMSK1 = 0; // no timer interrupts

#if OUTPUT == SERIAL_DUMP
  Serial.begin(9600);
#elif OUTPUT == HID_DEVICE
#else
# error "Output not defined"
#endif
}

void loop()
{
#ifdef LOOP_TIMING
  unsigned long ttt = millis();
#endif
  register unsigned char ra; // accumulator; never stored to memory so hint it's a register
  
  // read pushbuttons' state
  ra = !digitalRead(buttonPins[0]);
  buttons |= ra << 1;
  ra = !digitalRead(buttonPins[1]);
  buttons |= ra;

  // check if we've been enabled
  if (Started()) {
    // prepare for reading pots
    paddles[0] = paddles[1] = 0;
    triggered = 0;

    // start very tight counter cycle

    // TSM = hold rest signals asserted
    // PSRSYNC = prescaler for TCNT0 & TCNT1
    GTCCR = (1 << TSM) | (1 << PSRSYNC);
    // set our timer to 0
    TCNT1 = 0;
    // allow TCNT1 to run
    GTCCR = 0x0;
    // start charging the cap
    digitalWrite(powerPin, HIGH);
    // loop
    // TODO allow only one paddle to be connected
    while(triggered != encountered);
    // cool, now ground the caps
    digitalWrite(powerPin, LOW);
    // the timer can keep running, I don't care
    // let's store the timing on the paddles
    paddles[0] = paddlesTmp[0];
    paddles[1] = paddlesTmp[1];

    // update range
    for(byte rb = 0; rb < 2; ++rb) {
      if (mins[rb] > paddles[rb]) mins[rb] = paddles[rb];
      if (maxs[rb] < paddles[rb]) maxs[rb] = paddles[rb];
    }
    
    // output
    MyPrint(0);
    MyPrint(1);
    Serial.println();
    
    // reset button state
    buttons &= 0xFC; 
  
#ifndef LOOP_TIMING
    delay(LOOP_DELTA);
#endif  
  } else {
#ifndef LOOP_TIMING;
    // flip the power on and off on the RC networks
    // to get them used to oscillating :-)
    digitalWrite(powerPin, HIGH);
    delay(LOOP_DELTA/4);
    digitalWrite(powerPin, LOW);
    delay(LOOP_DELTA);
    // remember which paddles were encountered, if any
    // technically if none are encountered, there's no way
    // Started() will ever return true
    encountered = triggered;
#endif
  }
  
  /*
    Now, everybody's favourite part: TIMING!
    
    Our 1MOhm 4.7nF RC network takes ~4.7ms to charge
    Actually it only takes *us* ~4 ms to charge since we only
    go 46-48% of the way instead of 67%
    
    To discharge the cap, we need to wait 4 * 3 =~ 12ms or such
    to get it below 5% charge; which is well enough for us
    
    To recap what this loop does:
    - a quarter of the time it charges the cap
    - the rest of the time is spent writing to serial and waiting
    
    Since the moment we're done reading the the timer,
    we have to waste 12ms or more.
        
    Why mention all of this? 4ms to charge caps + 1-2ms
    for serial communication + 12ms of sleep puts us at
    max 18ms per loop (and much better when the pots are
    turned right). ~17ms is enough for 60Hz snappiness.
    W00T!
  */
  
#ifdef LOOP_TIMING
  Serial.print(millis() - ttt); Serial.print("ms\n");
  delay(500); // debug
#endif
}
