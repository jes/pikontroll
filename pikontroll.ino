/* pikontroll - Arduino code for controlling my PiKon telescope
 *
 * it assumes there is one servo and 2 motors, the motors are driven by an L298N, and each motor has a quadrature encoder
 *
 * configure pin assignments in setup() on the lines marked "// pin"
 * if your motors run backwards, either swap the encoder pins or the motor control pins, but
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

// aim to be within EPSILON steps of the target point
#define EPSILON 5

typedef struct Motor {
  long pos;
  long target;
  bool havetarget;
  int dir;
  unsigned long last;
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
  motor[m].dir = d;
  digitalWrite(motor[m].forwardpin, d == 1);
  digitalWrite(motor[m].reversepin, d == -1);
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
    // TODO: jog it for a few microseconds based on the average time between ticks in order to take up the last EPSILON number of ticks to get bang on the target?
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
    } else if (c != '\r') {
      buf[p++] = c;
    }
  }
}

void serial_command(char *buf) {
  char **params = split(buf);

  if (strcmp(params[0], "help") == 0) {
    Serial.print(
      "commands:\n"
      "   help       - show help\n"
      "   drive N D  - drive motor N (0,1) in direction D (-1,0,1) (disables target)\n"
      "   target N P - make motor N (0,1) move to position P (-2bn .. +2bn)\n"
      "   read N     - read state of motor N\n"
      "   servo N    - set motor microseconds to N (servomin .. servomax)\n"
      "   readservo  - read servo microseconds and bounds\n");
      
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

    motordir(num, dir);
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
    Serial.print("\n");
    
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
    Serial.print("\n");
    
  }
}

// replace each space in buf with a \0, and return a (static!) nul-terminated array of pointers to the string parts
char **split(char *buf) {
  static char *parts[16];
  int n = 0;
  
  char *p = buf;
  while (*p) {
    parts[n++] = p;
    while (*p && *p != ' ')
      p++;
    if (*p == ' ') {
      *p = 0;
      p++;
    }
  }
  parts[n++] = 0;
  
  return parts;
}
