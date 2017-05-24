#define LEFT A3
#define MID A4
#define RIGHT A5
#define LSPEED
#define RSPEED
#define L1
#define L2
#define R1
#define R2

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

void setup() {
  Serial.begin(9600);
  pinMode(LEFT, INPUT);
  pinMode(MID, INPUT);
  pinMode(RIGHT, INPUT);
  pinMode(LSPEED, OUTPUT);
  pinMode(RSPEED, OUTPUT);
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);
  for(int i = 0; i < SAMP; i++) {
    calib[0] += analogRead(LEFT);
    calib[1] += analogRead(MID);
    calib[2] += analogRead(RIGHT);
  }
  calib[0] /= SAMP;
  calib[1] /= SAMP;
  calib[2] /= SAMP;
}

int lspeed, rspeed;
const int RDLOW = 100, RDHIGH = 200;
struct pid rmonitor;
void loop() {
  /* pseudocode:
   *  if right < rlow:
   *    turn left a bit
   *  else if right > rhigh:
   *    turn right a bit
   *  else if center < clow:
   *    turn left a bit
   *  
   *  pid for right distance, error is difference from range rlow to rhigh, not just a set value
   */
  int left = analogRead(LEFT);
  int mid = analogRead(MID);
  int right = analogRead(RIGHT);
  left -= calib[0];
  mid -= calib[1];
  right -= calib[2];
  if(millis() % 500 == 0) {
    Serial.print("Left:  ");
    Serial.println(left);
    Serial.print("Mid:   ");
    Serial.println(mid);
    Serial.print("Right: ");
    Serial.println(right);
  }
  int rerror = 0;
  if(right < RDLOW)
    rerror = right - RDLOW;
  else if(right > RDHIGH)
    rerror = right - RDHIGH;
  getFix(&rmonitor, rerror);
}

void resetPid(struct pid *e) {
  e->integral = 0;
  e->prev = 0;
}

int getFix(struct pid *e, int error) {
  int d = (error - e->prev)/DT;
  e->integral += e->error * DT;
  e->prev = error;
  return e->kp * error + e->ki * e->integral + e->kd * d;
}
