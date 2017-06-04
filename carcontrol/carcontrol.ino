/*The differences aren't much here, I just merged our two sources real quickly and updated the port locations and got it to compile */
#include <limits.h>

#define distsens_1 A5 //L
#define distsens_2 A3 //Mid
#define distsens_3 A1 //Right
#define h_in1 6 //d6 //Right Motor Digital Control
#define h_in2 7 //d7
#define h_in3 9 //D9 Left Motor Digital Control
#define h_in4 10 //D10
#define h_pwm_l 5 //D3 Left Motor Analog Write
#define h_pwm_r 3 //D5 Right Motor Analog Write
#define turn_right 0
#define turn_left 1

struct pid { //Note that because we have two different types of distance sensors (Andrew's works a little differently than Jeffrey's we should have two different errors. To stay straight though we can just use one side right?)
  double integral = 0;
  int prev = 0;
  double kp = 1; //the ks should be negative to counteract error
  double ki = 0;
  double kd = 0.8;
};

struct motor_val{
 motor_val(){l_motor = 0; r_motor = 0; return;}
 int l_motor, r_motor;
};

const int DT = 1;
long calib[3]; // desired distance for left and right, far for center
const int SAMP = 100; // # of samples averaged for calibration
const int AVE = 10; // # of samples averaged for distance reading
const long MAX_TURN_TIME = 200;
const int RDLOW = 100, RDHIGH = 200;
const int MIN_WORKING_VARIANCE = 50;
const int min_wall_val = 200; // the closest we're going to get to a wall in front, higher than this means there's a wall in front
const int min_wall_side = -60; // the farthest we're going to get to a wall on the side, higher than this means there's a wall on the side
const int default_speed = 160;
struct motor_val motors;

inline void mode(struct pid* in);

inline int approx(int x, int y){ // &INT_MAX doesn't actually get the absolute value; -1 & INT_MAX returns INT_MAX instead of 1 or 0.
  int diff_neg = (x-y)>>15;
  return (((x-y)^diff_neg)-diff_neg) < MIN_WORKING_VARIANCE;
}

template<typename T> inline void slowPrint(T s);
template<typename T> inline void slowPrint(T s){
  if(millis()%500 == 0)
    Serial.println(s);
}

inline void timed_turn(const int x){
  const int init_time = millis();
  if(x == turn_right){
    turn_r();
    do{
    analogWrite(h_pwm_l, 200);
    analogWrite(h_pwm_r, 200);
    } while(millis() - init_time < MAX_TURN_TIME);
    analogWrite(h_pwm_l, 0);
    analogWrite(h_pwm_r, 0);
  }
  else if(x == turn_left){
    turn_l();
    do{
    analogWrite(h_pwm_l, 200);
    analogWrite(h_pwm_r, 200);
    } while(millis() - init_time < MAX_TURN_TIME);
    analogWrite(h_pwm_l, 0);
    analogWrite(h_pwm_r, 0);
  }
  return;
}

inline void forward(){
    digitalWrite(h_in1, HIGH);
    digitalWrite(h_in2, LOW);
    digitalWrite(h_in3, HIGH);
    digitalWrite(h_in4, LOW);
    slowPrint("F");
    return;
 }
inline void backward(){
    digitalWrite(h_in1, LOW);
    digitalWrite(h_in2, HIGH);
    digitalWrite(h_in3, LOW);
    digitalWrite(h_in4, HIGH);
    slowPrint("B");
    return;
 }

inline void s_brake(){
    digitalWrite(h_in1, HIGH);
    digitalWrite(h_in2, HIGH);
    digitalWrite(h_in3, HIGH);
    digitalWrite(h_in4, HIGH);
    slowPrint("STOP");
    return;
 }

inline void coast(){
    digitalWrite(h_in1, LOW);
    digitalWrite(h_in2, LOW);
    digitalWrite(h_in3, LOW);
    digitalWrite(h_in4, LOW);
    slowPrint("COAST");
    return;
 }

inline void turn_l(){
    digitalWrite(h_in1, HIGH);
    digitalWrite(h_in2, LOW);
    digitalWrite(h_in3, LOW);
    digitalWrite(h_in4, HIGH);
    slowPrint("L");
    return;
  }
inline void turn_r(){
    digitalWrite(h_in1, LOW);
    digitalWrite(h_in2, HIGH);
    digitalWrite(h_in3, HIGH);
    digitalWrite(h_in4, LOW);
    slowPrint("R");
    return;
}

inline void curve_l(){/*Curves around the left side of the rodent*/
    digitalWrite(h_in1, LOW);
    digitalWrite(h_in2, HIGH);
    digitalWrite(h_in3, LOW);
    digitalWrite(h_in4, LOW);
    slowPrint("L CURVE");
    return;
}
inline void curve_r(){
    digitalWrite(h_in1, LOW);
    digitalWrite(h_in2, LOW);
    digitalWrite(h_in3, HIGH);
    digitalWrite(h_in4, LOW);
    slowPrint("R CURVE");
    return;
}
template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }

void resetPid(struct pid *e) {
  e->integral = 0;
  e->prev = 0;
}

int getFix(struct pid *e, int error) {
  double d = (error - e->prev)/DT;
  e->integral += error * DT;
  e->prev = error;
  //Serial.println((int)(e->kp * error));
  //Serial.println((int)(e->ki * e->integral));
  //Serial.println((int)(e->kd * d));
  return (int)(e->kp * error + e->ki * e->integral + e->kd * d);
}

int aleft[AVE], amid[AVE], aright[AVE];
int count; // the index of the most recent read. Add 1 to get the one that will be removed.
long dleft, dmid, dright;
void setup() {
  Serial.begin(9600);
  pinMode(distsens_1, INPUT);
  pinMode(distsens_2, INPUT);
  pinMode(distsens_3, INPUT);
  pinMode(h_in1, OUTPUT);
  pinMode(h_in2, OUTPUT);
  pinMode(h_in3, OUTPUT);
  pinMode(h_in4, OUTPUT);
  pinMode(h_pwm_l, OUTPUT);
  pinMode(h_pwm_r, OUTPUT);
  for(int i = 0; i < SAMP; i++) {
    calib[0] += analogRead(distsens_1);
    calib[1] += analogRead(distsens_2);
    calib[2] += analogRead(distsens_3);
  }
  calib[0] /= SAMP;
  calib[1] /= SAMP;
  calib[2] /= SAMP;
  forward();
  Serial.println(calib[0]);
  Serial.println(calib[1]);
  Serial.println(calib[2]);
  //  Serial << "True Average:\n" << avg;
  for(count = 0; count < AVE; count++) {
    aleft[count] = analogRead(distsens_1);
    dleft += aleft[count];
    amid[count] = analogRead(distsens_2);
    dmid += amid[count];
    aright[count] = analogRead(distsens_3);
    dright += aright[count];
  }
}

struct pid lmon;
struct pid rmon;
void loop() {
/*+ * pseudocode:
+   *  if right < rlow:  //For straight turning
+   *    turn left a bit
+   *  else if right > rhigh:
+   *    turn right a bit
+   *  else if center < clow:
+   *    keep going until center ~= left or right (whichever is pointing to a wall) #We can make this even easier by defining CLOW as calib[0] or calib[1], which we calibrate in the center of the road at the start between two walls.
    Turn to open side. Go straight (goto start of loop)
+   *  pid for right distance, error is difference from range rlow to rhigh, not just a set value
+   */
/*
 * General notes: the value of each sensor goes up when distance goes down.
 */
  analogWrite(h_pwm_l, motors.l_motor);
  analogWrite(h_pwm_r, motors.r_motor);
  count = (count + 1) % AVE;
  dleft -= aleft[count];
  dmid -= amid[count];
  dright -= aright[count];
  aleft[count] = analogRead(distsens_1);
  amid[count] = analogRead(distsens_2);
  aright[count] = analogRead(distsens_3);
  dleft += aleft[count];
  dmid += amid[count];
  dright += aright[count];
  int left = (int)(dleft / AVE);
  int mid = (int)(dmid / AVE);
  int right = (int)(dright / AVE);
  
  left -= calib[0];
  mid -= calib[1];
  right -= calib[2];
  // Going straight
  if(right > min_wall_side) { // follow the right wall if it's going straight
    motors.r_motor = constrain(default_speed + getFix(&rmon, right), default_speed/4, 2*default_speed);
    motors.l_motor = default_speed;
  }
  else if(left > min_wall_side) { // otherwise follow the left wall if it's going straight
    motors.r_motor = default_speed;
    motors.l_motor = constrain(default_speed + getFix(&lmon, left), default_speed/4, 2*default_speed);
  } // otherwise you're probably fine, just keep speeds the same
  if(millis() % 500 == 0) {
    Serial.print("Left:  ");
    Serial.println(left);
    Serial.print("\nMid:   ");
    Serial.println(mid);
    Serial.print("\nRight: ");
    Serial.println(right);
    Serial.print("\n");
    //Serial.println(motors.r_motor);
    //Serial.println(motors.l_motor);
    //Serial.println(default_speed + getFix(&rmon, right));
  }
  /*
  if(right < 0){ //Error is towards the right, adjust right motor speed;
    rerror = right - RDLOW;
    motors.l_motor = rerror; //There error is positive right?
  motors.r_motor = default_speed;
  }
  else if(right > 0) {//Error is towards the left, adjust left motor speed
    rerror = right - RDHIGH;
  motors.r_motor = rerror; //Our bounds for this is probably bad? from positive 0 - 255
  motors.l_motor= default_speed;
  }*/
  
  if(mid > min_wall_val){ //If we've hit a wall, then do the following:
    if(!approx(right, left)){
      if(right < left){
        if(millis()%500 == 0)
          Serial.print("Right \n");
        turn_r(); //Our code may naturally want to just go right based off of pwm. If it does great. Otherwise, we can just force it to spin for a set amount of time, and then just use PID to stay in the center.
        while(mid > min_wall_val) {} // turn until the mid sensor is not facing a wall. PID should straighten it out.
      }
      else{
        if(millis()%500 == 0)
          Serial.print("Left \n");
        turn_l();
        while(mid > min_wall_val) {}
      }
    }
    else if(approx(mid, right)){ //Just in case we get caught into a deadend, we simply go backwards. This will mean we'd have to be watching left instead of right, so I'd prefer to turn 180 instead. This shouldn't happen.
      //if(millis()%500 == 0)
      //  Serial.print("Back \n");
      //backward();
      turn_l();
      while(mid > min_wall_val) {}
    }
    else { // This is a T-junction. Wall in front, left and right are open. This shouldn't happen.
      turn_r();
      while(mid > min_wall_val) {}
    }
  }
  forward();
}

