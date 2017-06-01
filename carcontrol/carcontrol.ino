/*The differences aren't much here, I just merged our two sources real quickly and updated the port locations and got it to compile */
#include <limits.h>

#define distsens_1 A5 //L
#define distsens_2 A4 //Mid
#define distsens_3 A3 //Right
#define h_in1 6 //d6 //Right Motor Digital Control
#define h_in2 7 //d7
#define h_in3 9 //D9 Left Motor Digital Control
#define h_in4 10 //D10
#define h_pwm_l 3 //D3 Left Motor Analog Write
#define h_pwm_r 5 //D5 Right Motor Analog Write

struct pid {
  int integral = 0;
  int prev = 0;
  int kp = -1; //the ks should be negative to counteract error
  int ki = -1;
  int kd = -1;
};
const int DT = 1;
long calib[3]; // minimum reading, theoretically infinite distance
const int SAMP = 10;
int pwm_l = 0;
int pwm_r = 0;

inline void mode(struct pid* in);

inline void forward(){
    digitalWrite(h_in1, HIGH);
    digitalWrite(h_in2, LOW);
    digitalWrite(h_in3, HIGH);
    digitalWrite(h_in4, LOW);
    return;
 }
inline void backward(){
    digitalWrite(h_in1, LOW);
    digitalWrite(h_in2, HIGH);
    digitalWrite(h_in3, LOW);
    digitalWrite(h_in4, HIGH);
    return;
 }

inline void s_brake(){
    digitalWrite(h_in1, HIGH);
    digitalWrite(h_in2, HIGH);
    digitalWrite(h_in3, HIGH);
    digitalWrite(h_in4, HIGH);
    return;
 }

inline void coast(){
    digitalWrite(h_in1, LOW);
    digitalWrite(h_in2, LOW);
    digitalWrite(h_in3, LOW);
    digitalWrite(h_in4, LOW);
    return;
 }

inline void turn_l(){
    digitalWrite(h_in1, HIGH);
    digitalWrite(h_in2, LOW);
    digitalWrite(h_in3, LOW);
    digitalWrite(h_in4, HIGH);
    return;
  }
inline void turn_r(){
    digitalWrite(h_in1, LOW);
    digitalWrite(h_in2, HIGH);
    digitalWrite(h_in3, HIGH);
    digitalWrite(h_in4, LOW);
    return;
}

inline void curve_l(){/*Curves around the left side of the rodent*/
    digitalWrite(h_in1, LOW);
    digitalWrite(h_in2, HIGH);
    digitalWrite(h_in3, LOW);
    digitalWrite(h_in4, LOW);
    return;
}
inline void curve_r(){
    digitalWrite(h_in1, LOW);
    digitalWrite(h_in2, LOW);
    digitalWrite(h_in3, HIGH);
    digitalWrite(h_in4, LOW);
  return;
}
template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }

void resetPid(struct pid *e) {
  e->integral = 0;
  e->prev = 0;
}

int getFix(struct pid *e, int error) {
  int d = (error - e->prev)/DT;
 //e->integral += e->error * DT;
  e->prev = error;
  return e->kp * error + e->ki * e->integral + e->kd * d;
}

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
  coast();
  Serial.println(calib[0]);
  Serial.println(calib[1]);
  Serial.println(calib[2]);
//  Serial << "True Average:\n" << avg;
}

int lspeed, rspeed;
const int RDLOW = 100, RDHIGH = 200;
const int CLOW = 80; //This is a BS number, I have no idea what this is off the top of my head.
const int MIN_WORKING_VARIANCE = 20;
struct pid rmonitor;
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
  int left = analogRead(distsens_1);
  int mid = analogRead(distsens_2);
  int right = analogRead(distsens_3);
  left -= calib[0];
  mid -= calib[1];
  right -= calib[2];
  if(millis() % 500 == 0) {
    Serial.print("Left:  ");
    Serial.println(left);
    Serial.print("\nMid:   ");
    Serial.println(mid);
    Serial.print("\nRight: ");
    Serial.println(right);
  Serial.print("\n");
  }
  
  int rerror = 0;
  if(right < RDLOW){ //Error is towards the right, adjust right motor speed;
    rerror = right - RDLOW;
  pwm_r = rerror; //Should we map this?
  }
  else if(right > RDHIGH) {//Error is towards the left, adjust left motor speed
    rerror = right - RDHIGH;
  pwm_r = rerror; //Our bounds for this is probably bad? from positive 0 - 255
  }
  if(mid < CLOW){
    if((((right - left)&INT_MAX) < MIN_WORKING_VARIANCE) && (((mid - right)& INT_MAX) < MIN_WORKING_VARIANCE)){ //Just in case we get caught into a deadend, we simply go backwards
      backward();
    }
    if(right < left && ((right - left)&INT_MAX < MIN_WORKING_VARIANCE)){
      turn_l(); //Our code may naturally want to just go right based off of pwm. If it does great. Otherwise, we can just force it to spin for a set amount of time, and then just use PID to stay in the center.
    }
    else if(left < right && ((right - left)&INT_MAX < MIN_WORKING_VARIANCE)){
      turn_r;
    }
  s_brake; //Something went wrong
  Serial.print("Well fuck, looks like I'm stuck \n");
  }
  else{
    forward();
    getFix(&rmonitor, rerror);  
  }
}

