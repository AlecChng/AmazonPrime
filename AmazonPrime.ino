#include <Wire.h>
#include <RoboShield.h>

RoboShield roboShield(0);


//Constants
int halting = -1;
int line_follow = 1;
int middle_inter = 2;
int right_turn_1 = 3;
int right_turn_align_left = 4;
int approach_supply = 5;
int align_claw = 6;
int retreat = 7;
int turn_around_1 = 8;
int return_start = 9;
int align_left_inner = 10;
int turn_to_score = 11;
int score = 12;
int reverse_reset = 13;
int turn_around_reset = 14;
int turn_align_outer_reset = 15;
int turn_align_inner_left_reset = 16;
int right_turn_2 = 17;
int turn_around_2 = 18;

int state = line_follow;


int kp = 7;
int error = 0;

int threshold = 790;

int r_motor = 2;
int l_motor = 3;
int winch = 0;
int claw_motor = 0;
int first_loop = 1;

int left_inner = 0;
int left_outer = 7;
int right_inner = 5;
int right_outer = 3;
int center = 10;

int def_speed = 17;

int forward = 1;
int backward = -1;

int open_claw = 0;
int close_claw = 1;
int claw_state = close_claw;



void setup() {
  Serial.begin(9600);
  if (roboShield.buttonPressed())
    roboShield.debuggingMode();
  
  controlClaw(close_claw);
  roboShield.setMotor(winch, 30);
  delay(3000);
  roboShield.setMotor(winch, 0);

}

bool onLine(int sensor) {
  bool result = false;
  
  if (roboShield.getAnalog(sensor) >= threshold) {
    result = true;
  }
  
  return result;
}

void halt() {
  roboShield.setMotor(2, 0);
  roboShield.setMotor(3, 0); 
}

int getPosition() {
  int pos = 0;
  if (onLine(right_outer)) {
    pos = 4;
  }
  else if (onLine(left_outer)) {
    pos = -4;
  }
  else if(onLine(right_inner)) {
    pos = 2;
  }
  else if (onLine(left_inner)) {
    pos = -2;
  }
  return pos;
}

void lineFollow() {
   error = getPosition();

  int correction = kp * error;
  int r_speed = def_speed - correction;
  int l_speed = def_speed + correction;
  if (r_speed > 128) {
    r_speed = 128;
  }
  if (r_speed <-128) {
    r_speed = -128;
  }
  if (l_speed > 128) {
    l_speed = 128;
  }
  if (l_speed < -128) {
    l_speed = -128;
  }
  roboShield.setMotor(r_motor, r_speed);
  roboShield.setMotor(l_motor, -l_speed);
}


void moveStraight(int dir, int right, int left) {
  //at 8.08 V
    roboShield.setMotor(r_motor, dir * right);
    roboShield.setMotor(l_motor, dir * -left);
}

bool hitIntersection() {
  int sum = 0;
  if (onLine(right_outer) ) {
    sum++;
  }
  if (onLine(left_outer)) {
    sum++;
  }
  if (onLine(right_inner)) {
    sum++;
  }
  if (onLine(left_inner)) {
    sum++;  
  }
  if (onLine(center)) {
    sum++;  
  }
  return sum >= 3;
}

void controlClaw(int c_state) {
  if (c_state == open_claw) {
    roboShield.setServo(claw_motor, 90);      
  }
  else if (c_state == close_claw) {
    roboShield.setServo(claw_motor, -100);
  }
}

void raiseLift() {
  roboShield.setMotor(winch, 20);
  delay(500);
}

void lowerLift() {
  roboShield.setMotor(winch, 20);
  delay(500);
}

void haltLift() {
  roboShield.setMotor(winch, 0);
}

void loop() {

  if (state == line_follow) {
    lineFollow();
      if (hitIntersection()) {
        state = middle_inter;
      }
  }
  else if (state == middle_inter) {

    moveStraight(forward, 20, 25);
    delay(400);
    state = right_turn_1;
  }
  else if (state == right_turn_1) {
    if (onLine(right_outer)) {
      roboShield.setMotor(r_motor, 0);
      roboShield.setMotor(l_motor, 0);
      state = right_turn_2;
      
    }
    else {
      roboShield.setMotor(r_motor, 20);
      roboShield.setMotor(l_motor, 20);
    }
  }
  else if (state == right_turn_2) {
  
    if (onLine(left_outer)) {
      roboShield.setMotor(r_motor, 0);
      roboShield.setMotor(l_motor, 0);
      state = right_turn_align_left;
      
    }
    else {
      roboShield.setMotor(r_motor, 20);
      roboShield.setMotor(l_motor, 20);
    }
  }
  else if (state == right_turn_align_left) {
    if (onLine(right_inner)) {
      roboShield.setMotor(r_motor, 0);
      state = approach_supply;
      
    }
    else {
      roboShield.setMotor(l_motor, 25);
    }
  }
  else if (state == approach_supply) {
    lineFollow();
    if (hitIntersection()) {
      state = align_claw;
    }
  }
  else if (state == align_claw) {
    moveStraight(forward, 15, 22);
    delay(315);
    halt();
    roboShield.setMotor(winch, -20);
    delay(1300);
    roboShield.setMotor(winch, 0);
    controlClaw(open_claw);
    state = retreat;
  }
  else if (state == retreat) {
    moveStraight(backward, 30, 40);
    delay(700);
    halt();
    roboShield.setMotor(winch, -20);
   
    delay(1500);
    roboShield.setMotor(winch, 0);
    controlClaw(open_claw);
    state = turn_around_1;
  }
  else if (state == turn_around_1) {
    if (onLine(left_outer)) {
      roboShield.setMotor(l_motor, 0);
      roboShield.setMotor(r_motor, 0);
      state = turn_around_2;
    }
    else {
      roboShield.setMotor(r_motor, -15);
      roboShield.setMotor(l_motor, -15);
    }
  }
  else if (state == turn_around_2) {
    if (onLine(right_outer)) {
      roboShield.setMotor(l_motor, 0);
      roboShield.setMotor(r_motor, 0);
      state = align_left_inner;
    }
    else {
      roboShield.setMotor(r_motor, -15);
      roboShield.setMotor(l_motor, -15);
    }
  }
  else if (state == align_left_inner) {
    if (onLine(left_inner)) {
      roboShield.setMotor(l_motor, 0);
      state = return_start;
    }
    else {
      roboShield.setMotor(r_motor, -20);
      roboShield.setMotor(l_motor, -15);
    }
  }
  else if (state == return_start) {
    lineFollow();
    if (hitIntersection()) {
      state = turn_to_score;
      moveStraight(forward, 30, 40);
      delay(200);
      halt();
    }
    Serial.println("return_start");
  }
  else if (state == turn_to_score) {
    Serial.println("turn_to_SCORE");
    if (onLine(right_inner)) {
      roboShield.setMotor(l_motor, 0);
      roboShield.setMotor(r_motor, 0);
      state = score;
    }
    else {
      roboShield.setMotor(r_motor, -15);
      roboShield.setMotor(l_motor, -15);
    }
  }
  else if (state == score) {
    int saved_speed = def_speed;
    def_speed = 17;
    lineFollow();
    if (hitIntersection()) {

      state = turn_around_reset;
      moveStraight(forward, 20, 22);
      delay(315);
      halt();
      delay(500);
      controlClaw(close_claw);
      halt();
      roboShield.setMotor(winch, 40);
      delay(3000);
      roboShield.setMotor(winch, 0);
      
    }
    def_speed = saved_speed;
  }
  else if (state == turn_around_reset) {
    if (onLine(right_inner)) {
      roboShield.setMotor(l_motor, 0);
      roboShield.setMotor(r_motor, 0);
      state = turn_align_outer_reset;
    }
    else {
      roboShield.setMotor(r_motor, -15);
      roboShield.setMotor(l_motor, -15);
    }
  }
  else if (state == turn_align_outer_reset) {
    if (onLine(right_outer)) {
      roboShield.setMotor(l_motor, 0);
      roboShield.setMotor(r_motor, 0);
      state = turn_align_inner_left_reset;
    }
    else {
      roboShield.setMotor(r_motor, -15);
      roboShield.setMotor(l_motor, -15);
    }
  }
  else if (state ==  turn_align_inner_left_reset) {
    if (onLine(left_inner)) {
      roboShield.setMotor(l_motor, 0);
      roboShield.setMotor(r_motor, 0);
      state = line_follow;
      first_loop = 0;
    }
    else {
      roboShield.setMotor(r_motor, -15);
      roboShield.setMotor(l_motor, -15);
    }
  }
  else if (state == halting) {
    halt();
  }
}

 

