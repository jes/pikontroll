/* pikontroll - Arduino code for controlling my PiKon telescope
 *
 * it assumes there is one servo and 2 motors, the motors are driven by an L298N, and each motor has a quadrature encoder
 *
 * configure pin assignments in setup() on the lines marked "// pin"
 * note that the Uno can only do interrupts on pins 2 and 3, so you need one each of those pins
 * for each of the quadrature encoders
 *
 * commands are sent to this program over the Serial port, at 9600 baud, see the text in
 * serial_command() to learn how to use it
 *
 * jes 2018
 */

#include <Servo.h>

#define BACKWARDS -1
#define STOPPED 0
#define FORWARDS 1

#define BUFSZ 128

// continuous drive to within EPSILON steps of the target, and then use the
// "us_per_step" value to look up how long to run the motors for
#define EPSILON 40
// don't bother trying to get closer than MIN_EPSILON steps
#define MIN_EPSILON 4

typedef struct Motor {
  long pos;
  long target;
  bool havetarget;
  bool ok;
  int dir;
  unsigned long last;
  unsigned int us_per_step;
  int enc1, enc2; // encoder pins
  int forwardpin, reversepin; // motor control pins
} Motor;

int servous = 1500;
const int servomin = 525;
const int servomax = 2300;

volatile Motor motor[2];
Servo servo;

void setup() {
  Serial.begin(9600);
  Serial.println("Setup...");

  // initialise servo
  servo.attach(8); // pin
  servo.writeMicroseconds(servous);

  // setup motor pins (note that the "enc1" pins must support interrupts)
  motor[0].enc1 = 2; // pin
  motor[0].enc2 = 4; // pin
  motor[0].forwardpin = 10; // pin
  motor[0].reversepin = 9; // pin
  motor[1].enc1 = 3; // pin
  motor[1].enc2 = 5; // pin
  motor[1].forwardpin = 11; // pin
  motor[1].reversepin = 12; // pin

  // configure microsecs required for each step when moving a small number of steps
  motor[0].us_per_step = 600;
  motor[1].us_per_step = 600;

  // initialise encoders
  pinMode(motor[0].enc1, INPUT);
  pinMode(motor[0].enc2, INPUT);
  pinMode(motor[1].enc1, INPUT);
  pinMode(motor[1].enc2, INPUT);
  attachInterrupt(digitalPinToInterrupt(motor[0].enc1), statechange0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor[1].enc1), statechange1, CHANGE);

  // initialise motor driver pins
  pinMode(motor[0].forwardpin, OUTPUT);
  pinMode(motor[0].reversepin, OUTPUT);
  pinMode(motor[1].forwardpin, OUTPUT);
  pinMode(motor[1].reversepin, OUTPUT);

  // ensure both motors are stopped
  stopmotor(0); stopmotor(1);
  
  Serial.println("Ready.");
}

// interrupts to read motor quadrature
void statechange0() { statechange(0); }
void statechange1() { statechange(1); }
void statechange(int m) {
  if (digitalRead(motor[m].enc1) == digitalRead(motor[m].enc2))
    motor[m].pos--;
  else
    motor[m].pos++;
  motor[m].last = millis();
}

// functions to control motors
void wait_for_stop(int m) {
  motordir(m, STOPPED);
  // wait until at least 10ms since the last step
  while (motor[m].last > millis() - 10) { }
}
void stopmotor(int m) {
  motordir(m, STOPPED);
}
void forwards(int m) {
  motordir(m, FORWARDS);
}
void backwards(int m) {
  motordir(m, BACKWARDS);
}
void motordir(int m, int d) {
  if (d != 0 && motor[m].dir == -d) {
    // stop the motor for 100ms before reversing direction, to avoid voltage spikes in power supply
    motordir_timed(m, STOPPED, 100);
  }
  motor[m].dir = d;
  digitalWrite(motor[m].forwardpin, d == 1);
  digitalWrite(motor[m].reversepin, d == -1);
}
void motordir_timed(int m, int d, unsigned long ms) {
  motordir_timed_us(m, d, ms*1000);
}
void motordir_timed_us(int m, int d, unsigned long us) {
  unsigned long endt = micros() + us;
  motor[m].dir = d;
  digitalWrite(motor[m].forwardpin, d == 1);
  digitalWrite(motor[m].reversepin, d == -1);
  while (micros() < endt) {}
  digitalWrite(motor[m].forwardpin, 0);
  digitalWrite(motor[m].reversepin, 0);
  motor[m].dir = STOPPED;
}

// run motor m for 100ms both forwards and backwards, to see if it is working
void testmotor(int m) {
  long pos;
  bool ok = true;

  // forwards
  pos = motor[m].pos;
  motordir_timed(m, FORWARDS, 100);

  if (motor[m].pos == pos) {
    Serial.print("Motor "); Serial.print(m); Serial.println(" not running on forwards pin");
    ok = false;
  } else {
    // stop for 100ms to let it settle
    motordir_timed(m, STOPPED, 100);
  }

  // if motor ran backwards instead of forwards, swap pin assignments and check again
  if (motor[m].pos < pos) {
    Serial.print("Motor "); Serial.print(m); Serial.println(" ran in reverse instead of forwards; swapping pin assignment and trying again");

    int pin = motor[m].forwardpin;
    motor[m].forwardpin = motor[m].reversepin;
    motor[m].reversepin = pin;

    pos = motor[m].pos;
    motordir_timed(m, FORWARDS, 100);

    if (motor[m].pos == pos) {
      Serial.print("Motor "); Serial.print(m); Serial.println(" not running on forwards pin after swapping pin assignment");
      ok = false;
    } else {
      // stop for 100ms to let it settle
      motordir_timed(m, STOPPED, 100);
    }

    // still running backwards?!
    if (motor[m].pos < pos) {
      Serial.print("Motor "); Serial.print(m); Serial.println(" still running in reverse after swapping pin assignment");
      ok = false;
    }
  }

  // backwards
  pos = motor[m].pos;
  motordir_timed(m, BACKWARDS, 100);

  if (motor[m].pos == pos) {
    Serial.print("Motor "); Serial.print(m); Serial.println(" not running on reverse pin");
    ok = false;
  } else if (motor[m].pos > pos) {
    Serial.print("Motor "); Serial.print(m); Serial.println(" ran forwards on reverse pin");
    ok = false;
  }

  if (ok) {
    Serial.print("Motor "); Serial.print(m); Serial.println(" ok");
  }

  motor[m].ok = ok;
}

void loop() {
  handle_serial();

  // make sure motors are turning in the desired direction to reach their target
  for (int i = 0; i < 2; i++) {
    if (!motor[i].havetarget)
      continue;
      
    if (abs(motor[i].pos - motor[i].target) < EPSILON && motor[i].dir != STOPPED) {
      stopmotor(i);
    } else if (motor[i].pos > motor[i].target+EPSILON && motor[i].dir != BACKWARDS) {
      backwards(i);
    } else if (motor[i].pos < motor[i].target-EPSILON && motor[i].dir != FORWARDS) {
      forwards(i);
    }

    /* jog it for a few microseconds based on the configured time between steps in order to take up the
     * last EPSILON number of ticks to get closer to the target
     */
     if (motor[i].dir == STOPPED && abs(motor[i].pos - motor[i].target) > MIN_EPSILON) {
       wait_for_stop(i);
       if (abs(motor[i].pos - motor[i].target) > MIN_EPSILON && abs(motor[i].pos - motor[i].target) < EPSILON) {
         motordir_timed_us(i, motor[i].pos > motor[i].target ? BACKWARDS : FORWARDS, motor[i].us_per_step * abs(motor[i].pos - motor[i].target));
         wait_for_stop(i);
       }
     }
  }
}

void handle_serial() {
  static char buf[BUFSZ];
  static int p;
  
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r' || c== '\n' || p == BUFSZ-1) {
      buf[p++] = 0;
      serial_command(buf);
      p = 0;
    } else {
      buf[p++] = c;
    }
  }
}

void serial_command(char *buf) {
  char **params = split(buf);

  if (strcmp(params[0], "help") == 0) {
    Serial.print(
      "commands:\r\n"
      "   help           - show help\r\n"
      "   drive N D [M]  - drive motor N (0,1) in direction D (-1,0,1) [for M millisecs] (disables target)\r\n"
      "   target N P     - make motor N (0,1) move to position P (-2bn .. +2bn)\r\n"
      "   read N         - read state of motor N\r\n"
      "   servo N        - set motor microseconds to N (servomin .. servomax)\r\n"
      "   readservo      - read servo microseconds and bounds\r\n"
      "   test N         - re-test motor N (0,1)\r\n");
      
  } else if (strcmp(params[0], "drive") == 0) {
    if (!params[1] || !params[2]) {
      Serial.println("usage: drive N D");
      return;
    }
  
    int num = atoi(params[1]);
    int dir = atoi(params[2]);
    if (num < 0 || num > 1) {
      Serial.println("error: motor must be 0 or 1");
      return;
    }
    if (dir < -1 || dir > 1) {
      Serial.println("error: direction must be -1, 0, or 1");
      return;
    }

    if (params[3]) {
      int ms = atoi(params[3]);
      if (ms < 0) {
        Serial.println("error: can't drive motor for negative millisecs");
        return;
      }
      motordir_timed(num, dir, atoi(params[3]));
    } else {
      motordir(num, dir);
    }
    motor[num].havetarget = false;
    
  } else if (strcmp(params[0], "target") == 0) {
    if (!params[1] || !params[2]) {
      Serial.println("usage: target N P");
      return;
    }
  
    int num = atoi(params[1]);
    long target = atol(params[2]);
    if (num < 0 || num > 1) {
      Serial.println("error: motor must be 0 or 1");
      return;
    }

    motor[num].target = target;
    motor[num].havetarget = true;
    
  } else if (strcmp(params[0], "read") == 0) {
    if (!params[1]) {
      Serial.println("usage: read N");
      return;
    }
    
    int num = atoi(params[1]);
    if (num < 0 || num > 1) {
      Serial.println("error: motor must be 0 or 1");
      return;
    }

    Serial.print("motor "); Serial.print(num); Serial.print(": ");
    Serial.print("pos="); Serial.print(motor[num].pos);
    Serial.print(" dir="); Serial.print(motor[num].dir);
    Serial.print(" last="); Serial.print(millis() - motor[num].last);
    Serial.print(" target="); Serial.print(motor[num].target);
    Serial.print(" havetarget="); Serial.print(motor[num].havetarget);
    Serial.print(" ok="); Serial.print(motor[num].ok);
    Serial.print("\r\n");
    
  } else if (strcmp(params[0], "servo") == 0) {
    if (!params[1]) {
      Serial.println("usage: servo N");
      return;
    }

    int us = atoi(params[1]);
    if (us < servomin)
      us = servomin;
    if (us > servomax)
      us = servomax;
      
    servous = us;
    servo.writeMicroseconds(servous);
    
  } else if (strcmp(params[0], "readservo") == 0) {
    Serial.print("servo: us="); Serial.print(servous);
    Serial.print(" min="); Serial.print(servomin);
    Serial.print(" max="); Serial.print(servomax);
    Serial.print("\r\n");
    
  } else if (strcmp(params[0], "test") == 0) {
    if (!params[1]) {
      Serial.println("usage: test N");
      return;
    }

    int num = atoi(params[1]);
    if (num < 0 || num > 1) {
      Serial.println("error: motor must be 0 or 1");
      return;
    }

    testmotor(num);
  }
}

// replace each space in buf with a \0, and return a (static!) nul-terminated array of pointers to the string parts
char **split(char *buf) {
  static char *parts[16];
  int n = 0;
  
  char *p = buf;
  while (*p && n < 14) {
    parts[n++] = p;
    while (*p && *p != ' ')
      p++;
    if (*p == ' ')
      *(p++) = 0;
  }
  parts[n++] = 0;
  
  return parts;
}
