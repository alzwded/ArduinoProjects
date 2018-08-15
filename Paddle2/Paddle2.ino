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

/*
  Right now it only dumps the state of the controls over USB serial.

  TODO
  import joystick library
  improve timing
*/

static const int magic = 6;

int ra, /* read paddle */
    rb, /* read button */
    rc, /* counter of left paddle */
    rd; /* state (read analogue?) */
int paddles[2], buttons;

void setup()
{
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(7, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  buttons = 0;
  paddles[0] = paddles[1] = 0;
  Serial.begin(9600);
}

void loop()
{
  rd = (rd + 1) % magic;
  buttons = 0;
  if (rd == 0 || rd == magic / 2) {
    paddles[rd / (magic / 2)] = 0;
    digitalWrite(2, HIGH);
    rc = 0;
    do {
      ra = digitalRead((rd / (magic / 2)) ? A1 : A0);
      if (!ra) {
        ++rc;
      }
      else
      {
        paddles[rd / (magic / 2)] = rc;
        digitalWrite(2, LOW);
        break;
      }
    } while (1);
  }
  ra = !digitalRead(7);
  buttons |= ra << 1;
  ra = !digitalRead(4);
  buttons |= ra;

  Serial.print(paddles[0]);
  Serial.print(' ');
  Serial.print(paddles[1]);
  Serial.print(' ');
  Serial.println(buttons);
  delay(10); // Wait for 1000 millisecond(s)
}
