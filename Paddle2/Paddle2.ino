/*
  D2 -------- POT -x
              |
  A0 -- R1 -- C1
              |
  GND --------+

  D7 -- X1 -- GND

  C1 = C2 = 2.2n
  R1 = R2 = 1k

  Paddle controller:
  POT = 1M
  X1 = push button

  when POT is fully turned right, R(D2, C1) = 0Ohm

  Connect DB9:

                           -> PORT1
                           -> PORT2
  D7 --                    -> PORT3
  D4 --                    -> PORT4
  A1 -- R2 -- S12 -- C2 -- -> PORT5
                           -> PORT6
  D2 --                    -> PORT7
  GND -                    -> PORT8
  A0 -- R1 -- S12 -- C1 -- -> PORT9

  C1 -- GND
  C2 -- GND
  
  S1 - schmitt trigger
  5V -- S1Vcc
  GND - S1GND

*/

// toggle between debug dump and actual device
#define SERIAL_DUMP 1
#define HID_DEVICE 2
#define OUTPUT SERIAL_DUMP

// time the math done when printing values
//#define SERIAL_TIMING
// time how long a loop takes; 
// disable other delay calls and sleeps for half a second each loop
//#define LOOP_TIMING

// loops to wait between pot reads
// it can take a long time to discharge the pot
static const byte magic = 2;
// two pots => double that
static const byte magic2 = magic * 2;

// paddle pin mapping
static const byte paddlePins[2] = { A0, A1 };
static const byte buttonPins[2] = {  4,  7 };

static const int overhead[3] = { 
  22,       /* low overhead;  ~5% */
  22,       /* high overhead; ~5% */
  +16,      /* nudge it to the left a bit */
};

int paddles[2], mins[2], maxs[2]; /* paddle state, 16bit */
/* Loop counter
   0 - serial send
   1 - read pot A
   2 - serial send
   3 - read pot B
 */
byte rd;
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
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  // intialize paddle pins
  for (register unsigned char rc = 0; rc < 2; ++rc) {
    pinMode(paddlePins[rc], INPUT);
    pinMode(buttonPins[rc], INPUT_PULLUP);
  }

  // state
  rd = 0;
  buttons = 0;
  paddles[0] = paddles[1] = 0;
  mins[0] = mins[1] = 100;
  maxs[0] = maxs[1] = 300;

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
  
  // increment state
  rd = (rd + 1) % magic2;

  // check if it's a cycle where we're reading a pot
  if (Started() && (rd % magic)) {
    // prepare for reading pots
    // index of joystick we're measuring on this cycle
    byte rb;
    rb = rd / magic;
    paddles[rb] = 0;

    // start very tight counter cycle

    // TSM = hold rest signals asserted
    // PSRSYNC = prescaler for TCNT0 & TCNT1
    GTCCR = (1 << TSM) | (1 << PSRSYNC);
    // set our timer to 0
    TCNT1 = 0;
    // start charging the cap
    digitalWrite(2, HIGH);
    // allow TCNT1 to run
    GTCCR = 0x0;
    // it's important to do as little as possible here
    // to get a good resolution
    do {
      // read pin state
      ra = digitalRead(paddlePins[rb]);
      // did we hit 67%? (ish; TTL level should be there about)
      //
      // normally pin held high by schmitt trigger
      // so it gets set low once the cap rises past the
      // schmitt trigger's trigger level
      //
      // the data sheet says ~2.4V or so, which puts us well
      // within the linear ramp-up for the RC network
    } while (ra);
    // quickly pull out the value of TCNT1
    paddles[rb] = TCNT1;
    // cool, now ground the caps
    digitalWrite(2, LOW);
    // the timer can keep running, I don't care

    // update range
    if (mins[rb] > paddles[rb]) mins[rb] = paddles[rb];
    if (maxs[rb] < paddles[rb]) maxs[rb] = paddles[rb];
  } else {
    // stub read
    ra = digitalRead(paddlePins[rd / magic]);
  }

  // read pushbuttons' state
  ra = !digitalRead(buttonPins[0]);
  buttons |= ra << 1;
  ra = !digitalRead(buttonPins[1]);
  buttons |= ra;
  if (Started() && ((rd % magic) == 0)) {
    // output
    MyPrint(1);
    MyPrint(0);
    Serial.println();
    
    // reset button state
    buttons &= 0xFC;
    
#ifndef LOOP_TIMING
    delay(3);
#endif
  } else {
#ifndef LOOP_TIMING
    delay(2);
#endif
  }
  
  /*
    Now, everybody's favourite part: TIMING!
    
    Our 1MOhm 2.2nF RC network takes ~2.2ms to charge
    Actually it only takes *us* 2 ms to charge since we only
    go 46-48% of the way instead of 67%
    
    To discharge the cap, we need to wait 2 * 3 = 6ms or such
    to get it below 5% charge; which is well enough for us
    
    To recap what this loop does:
    - half of the time it charges the cap
    - half of the time it supposedly writes to serial
    
    Since the moment we're done reading the the timer,
    we have to waste 6ms or more.
    
    Assuming the joystic library is snappy enough,
    doing a sleep(2ms) should make us ready for the next
    analogue read.
    
    Why all that math? Well, to read both paddles:
    - wait for cap to charge  (2ms)
    - read A
    - wait to discharge (1/3) (2ms)
    - write output (2/3)      (~1ms)
    - wait to discharge (3/3) (3ms)
    - wait for cap to charge  (2ms)
    - read B
    - wait to discharge (1/3) (2ms)
    - write output (2/3)      (~1ms)
    - wait to discharge (3/3) (3ms)
    
    That's 8*~2 = 16ms. 17ms is our target 60Hz. W00T!
    
    And technically it goes faster when the paddles are turned
    right since the cap charges super fast in that case.
  */
  
#ifdef LOOP_TIMING
  Serial.print(millis() - ttt); Serial.print("ms  ");
  Serial.print(rd); Serial.println();
  delay(500); // debug
#endif

}
