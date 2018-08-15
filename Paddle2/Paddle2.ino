/*
  D2 ------- POT -x
              |
  A0 -- R1 -- C1
              |
  GND --------+

  D7 -- X1 -- GND

  C1 = C2 = 4.7n
  R1 = R2 = 1k

  Paddle controller:
  POT = 1M
  X1 = push button

  when POT is fully turned right, R(D2, C1) = 0Ohm

  Connect DB9:

                       PORT1
                       PORT2
  D7 --             -> PORT3
  D4 --             -> PORT4
  A1 -- R2 -- C2 -- -> PORT5
                       PORT6
  D2 --             -> PORT7
  GND -             -> PORT8
  A0 -- R1 -- C1 -- -> PORT9

  C1 -- GND
  C2 -- GND

*/

// cycles to wait between pot reads
// it can take a long time to discharge the pot
static const byte magic = 2;
// two pots => double that
static const byte magic2 = magic * 2;

// paddle pin mapping
static const byte paddlePins[2] = { A0, A1 };
static const byte buttonPins[2] = {  4,  7 };

byte ra, /* accumulator */
     rb, /* paddle index */
     rc, /* counter */
     rd; /* state (read analogue?) */
byte started; /* started writing to USB */
byte buttons; /* button state */
int paddles[2], mins[2], maxs[2]; /* paddle state */

inline void MyPrint(byte rc)
{
  // use FPU real quick
  float f = (paddles[rc] - mins[rc]);
  f /= (maxs[rc] - mins[rc]);
  f *= 1023;
  // we should be between 0-1023
  int r16 = (int)f;
  r16 &= 0x03FF;
  
#if 1
  static char buffer[] = "0000?\t";
  
  ra = 4;
  while(ra--) {
    rb = r16 % 10;
    buffer[ra] = 48 + rb;
    r16 /= 10;
  }
  buffer[4] = (' ' + ((buttons & (1 << rc)) != 0));
  
  Serial.print(buffer);
#else
  // TODO write as HID joystick
  // Right now it only dumps the state of the controls over USB serial.
#endif
}

void setup()
{
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  for(rc = 0; rc < 2; ++rc) {
    pinMode(paddlePins[rc], INPUT);
    pinMode(buttonPins[rc], INPUT_PULLUP);
  }
  
  buttons = 0;
  paddles[0] = paddles[1] = 0;
  mins[0] = mins[1] = 10;
  maxs[0] = maxs[1] = 190;
  started = 0;
  
  Serial.begin(9600);
}

void loop()
{
  // increment state
  rd = (rd + 1) % magic2;
  
  // check if it's a cycle where we're reading a pot
  if (rd % magic) {
    // prepare for reading pots
    rc = 0;
    rb = rd / magic;
    paddles[rb] = 0;
    
    // start very tight cycle
    digitalWrite(2, HIGH);
    do {
      // read pin state
      ra = digitalRead(paddlePins[rb]);
      // increment counter
      ++paddles[rb];
      // did we hit 67%? (ish; TTL)
    } while(!ra);
    // cool, no ground the caps
    digitalWrite(2, LOW);
    
    // update range
    if(mins[rb] > paddles[rb]) mins[rb] = paddles[rb];
    if(maxs[rb] < paddles[rb]) maxs[rb] = paddles[rb];
  }
  
  // read pushbuttons' state
  buttons = 0;
  ra = !digitalRead(buttonPins[0]);
  buttons |= ra << 1;
  ra = !digitalRead(buttonPins[1]);
  buttons |= ra;
  
  if(started = started || buttons) {
    // output
    MyPrint(0);
    MyPrint(1);
    Serial.println();
  }
  
  delay(10); // Wait for 10ms
}
