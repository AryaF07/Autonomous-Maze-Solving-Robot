// FUME - autonomous maze-solving robot
// Three fixed HC-SR04 ultrasonic sensors (left, front, right) feeding a
// rule-based left-hand wall follower with obstacle avoidance.

//motor driver
int ENA = 5;   // left motor PWM
int ENB = 10;  // right motor PWM
int IN1 = 9;
int IN2 = 8;
int IN3 = 7;
int IN4 = 6;

//ultrasonic sensors (each one needs its own trig/echo pair)
int F_TRIG = 13;
int F_ECHO = 12;
int L_TRIG = 4;   // PIN GUESSED - check against your wiring
int L_ECHO = 3;   // PIN GUESSED - check against your wiring
int R_TRIG = 11;  // PIN GUESSED - check against your wiring
int R_ECHO = 2;   // PIN GUESSED - check against your wiring

//sensor settings
const int PINGS_PER_READING = 3;    // median of this many pings
const long MAX_DISTANCE_CM = 200;   // anything further (or no echo) counts as open
const unsigned long ECHO_TIMEOUT_US = MAX_DISTANCE_CM * 58;  // ~58 us per cm round trip
const int PING_GAP_MS = 5;          // let echoes die out between pings

//navigation thresholds (cm) and timings (ms) - tune these on the real maze
int distance_collision = 30;  // front closer than this = wall ahead
int side_open = 40;           // side further than this = opening to turn into
int side_too_close = 8;       // side closer than this = steer away from wall
const int TURN_90_MS = 450;          // CALIBRATE: time for a ~90 degree pivot
const int FORWARD_INTO_GAP_MS = 300; // CALIBRATE: drive into an opening after turning
const int BACK_UP_MS = 250;
const int STEP_MS = 50;              // how long each forward step lasts

bool turning;
bool direction; // true for left false for right
bool hadLeftWall = false; // seen a left wall since the last left turn

long leftDist, frontDist, rightDist;

void setup() {
  Serial.begin(9600);
  pinMode(F_TRIG, OUTPUT);
  pinMode(F_ECHO, INPUT);
  pinMode(L_TRIG, OUTPUT);
  pinMode(L_ECHO, INPUT);
  pinMode(R_TRIG, OUTPUT);
  pinMode(R_ECHO, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  analogWrite(ENA,255); //left
  analogWrite(ENB,255); //right
  turning = false;
  direction = true;
}

void loop() {
  readSensors();

  // 1) Already turning away from a wall: keep rotating until the front is clear.
  if(turning) {
    if(direction) {
      rotateLeft();
    }
    else {
      rotateRight();
    }
    if(frontDist > distance_collision) {
      turning = false;
    }
    return;
  }

  // 2) Left-hand rule: if a left wall we were following opens up, take the turn.
  //    hadLeftWall stops it spinning in circles while the left stays open.
  if(leftDist <= side_open) {
    hadLeftWall = true;
  }
  else if(hadLeftWall) {
    rotateLeft();
    delay(TURN_90_MS);
    moveForward();
    delay(FORWARD_INTO_GAP_MS);
    hadLeftWall = false;  // wait until we pick up a left wall again
    return;
  }

  // 3) Wall ahead: turn toward whichever side has more room.
  if(frontDist < distance_collision) {
    stop();
    if(leftDist < distance_collision && rightDist < distance_collision) {
      // dead end: back off a little, then spin until a way out appears
      moveBack();
      delay(BACK_UP_MS);
    }
    turning = true;
    direction = (leftDist > rightDist) ? true : false;
    return;
  }

  // 4) Corridor: go forward, nudging away from a wall that is too close.
  if(leftDist < side_too_close) {
    rotateRight();
    delay(STEP_MS / 2);
  }
  else if(rightDist < side_too_close) {
    rotateLeft();
    delay(STEP_MS / 2);
  }
  moveForward();
  delay(STEP_MS);
}

void readSensors() {
  leftDist  = readDistance(L_TRIG, L_ECHO);
  frontDist = readDistance(F_TRIG, F_ECHO);
  rightDist = readDistance(R_TRIG, R_ECHO);

  Serial.print("L: ");
  Serial.print(leftDist);
  Serial.print("cm  F: ");
  Serial.print(frontDist);
  Serial.print("cm  R: ");
  Serial.print(rightDist);
  Serial.println("cm");
}

// Median of several pings, so one bad echo doesn't trigger a turn.
long readDistance(int _trig, int _echo) {
  long samples[PINGS_PER_READING];
  for(int i = 0; i < PINGS_PER_READING; i++) {
    samples[i] = distance(_trig, _echo);
    delay(PING_GAP_MS);
  }
  // insertion sort (tiny array)
  for(int i = 1; i < PINGS_PER_READING; i++) {
    long key = samples[i];
    int j = i - 1;
    while(j >= 0 && samples[j] > key) {
      samples[j + 1] = samples[j];
      j--;
    }
    samples[j + 1] = key;
  }
  return samples[PINGS_PER_READING / 2];
}

void moveForward(){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void rotateLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void rotateRight(){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void moveBack(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// One ping, in cm. Returns MAX_DISTANCE_CM if no echo arrives in time.
long distance(int _trig, int _echo) {
  digitalWrite(_trig, LOW);
  delayMicroseconds(2);
  digitalWrite(_trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(_trig, LOW);
  unsigned long duration = pulseIn(_echo, HIGH, ECHO_TIMEOUT_US);
  if(duration == 0) {
    return MAX_DISTANCE_CM;  // timed out: nothing in range
  }
  long distance = (duration / 2) * 0.0343;
  if(distance > MAX_DISTANCE_CM) {
    distance = MAX_DISTANCE_CM;
  }
  return distance;
}

void stop(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
