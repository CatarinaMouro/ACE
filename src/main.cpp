#include <Arduino.h>

//creates pwm instance
#define SONAR_FRONT_PIN_trig 2
#define SONAR_FRONT_PIN_echo 3
#define SONAR_RIGHT_PIN_trig 4
#define SONAR_RIGHT_PIN_echo 5 
#define SONAR_LEFT_PIN_trig 6
#define SONAR_LEFT_PIN_echo 7 

typedef struct {
  int state, new_state;
  unsigned long tes, tis;
} fsm_t;

// Sonar Trigger variables
volatile fsm_t fsm_triggerSonar_Front;
volatile fsm_t fsm_triggerSonar_Right;
volatile fsm_t fsm_triggerSonar_Left;

// Sonar Echo variables
unsigned long Echotime_init_front;
unsigned long Echotime_init_right;
unsigned long Echotime_init_left;

volatile int cont_f, cont_r, cont_l;


// 
unsigned long interval, last_cycle;
unsigned long loop_micros;


// GET DISTANCES
// Max Range: 4m
// Min Range: 2cm
long duration_sound_front, distance_cm_front;
long duration_sound_right, distance_cm_right;
long duration_sound_left, distance_cm_left;


void Sonar_receiveecho_front();
void Sonar_receiveecho_right();
void Sonar_receiveecho_left();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(SONAR_FRONT_PIN_trig, OUTPUT);
  pinMode(SONAR_FRONT_PIN_echo, INPUT);
  pinMode(SONAR_RIGHT_PIN_trig, OUTPUT);
  pinMode(SONAR_RIGHT_PIN_echo, INPUT);
  pinMode(SONAR_LEFT_PIN_trig, OUTPUT);
  pinMode(SONAR_LEFT_PIN_echo, INPUT);

  interval = 100;

  fsm_triggerSonar_Front.state=0;
  fsm_triggerSonar_Right.state=0;
  fsm_triggerSonar_Left.state=0;

  sleep_ms(1000);

  attachInterrupt(digitalPinToInterrupt(SONAR_FRONT_PIN_echo), Sonar_receiveecho_front, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SONAR_RIGHT_PIN_echo), Sonar_receiveecho_right, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SONAR_LEFT_PIN_echo), Sonar_receiveecho_left, CHANGE);
}



// Speed of Sound: 340m/s = 29microseconds/cm
// Sound wave reflects from the obstacle, so to calculate the distance we consider half of the distance traveled.  
// DistanceInCms=microseconds/29/2 
long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29 / 2;
}

#if 0
void set_state(fsm_t& fsm, int new_state)
{
  if (fsm.state != new_state) {  // if the state chnanged tis is reset
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.tis = 0;
  }
}
#endif


#define set_state(fsm,new_state) {\
  if (fsm.state != new_state) {\
    fsm.state = new_state;\
    fsm.tes = millis();\
    fsm.tis = 0;\
  }\
}


void Sonar_receiveecho_front(){
  int sonar_echo_f=digitalRead(SONAR_FRONT_PIN_echo);

  cont_f=cont_f+1;

  if (fsm_triggerSonar_Front.state==1){
    Echotime_init_front=micros();
    fsm_triggerSonar_Front.new_state = 2;
    set_state(fsm_triggerSonar_Front, fsm_triggerSonar_Front.new_state);
  } 
  else if (sonar_echo_f==HIGH && fsm_triggerSonar_Front.state==2){
    duration_sound_front=micros()-Echotime_init_front;
    distance_cm_front=microsecondsToCentimeters(duration_sound_front);
    fsm_triggerSonar_Front.new_state = 0;
    set_state(fsm_triggerSonar_Front, fsm_triggerSonar_Front.new_state);
  }
}

void Sonar_receiveecho_right(){
  int sonar_echo_r=digitalRead(SONAR_RIGHT_PIN_echo);

  cont_r=cont_r+1;

  if (fsm_triggerSonar_Right.state==1){
    Echotime_init_right=micros();
    fsm_triggerSonar_Right.new_state = 2;
    set_state(fsm_triggerSonar_Right, fsm_triggerSonar_Right.new_state);
  }
  else if (sonar_echo_r==LOW && fsm_triggerSonar_Right.state==2){
    duration_sound_right=micros()-Echotime_init_right;
    distance_cm_right=microsecondsToCentimeters(duration_sound_right);
    fsm_triggerSonar_Right.new_state = 0;
    set_state(fsm_triggerSonar_Right, fsm_triggerSonar_Right.new_state);

  }
}

void Sonar_receiveecho_left(){
  int sonar_echo_l=digitalRead(SONAR_LEFT_PIN_echo);

  cont_l=cont_l+1;

  if (fsm_triggerSonar_Left.state==1){
    Echotime_init_left=micros();
    fsm_triggerSonar_Left.new_state = 2;
    set_state(fsm_triggerSonar_Left, fsm_triggerSonar_Left.new_state);
  }
  else if (sonar_echo_l==LOW && fsm_triggerSonar_Left.state==2){
    duration_sound_left=micros()-Echotime_init_left;
    distance_cm_left=microsecondsToCentimeters(duration_sound_left);
    fsm_triggerSonar_Left.new_state = 0;
    set_state(fsm_triggerSonar_Left, fsm_triggerSonar_Left.new_state);
  }
}



void send_trigger(){
  digitalWrite(SONAR_FRONT_PIN_trig,LOW);
  digitalWrite(SONAR_RIGHT_PIN_trig,LOW);
  digitalWrite(SONAR_LEFT_PIN_trig,LOW);
  delayMicroseconds(2);
  digitalWrite(SONAR_FRONT_PIN_trig,HIGH);
  digitalWrite(SONAR_RIGHT_PIN_trig,HIGH);
  digitalWrite(SONAR_LEFT_PIN_trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_FRONT_PIN_trig,LOW);
  digitalWrite(SONAR_RIGHT_PIN_trig,LOW);
  digitalWrite(SONAR_LEFT_PIN_trig,LOW);
}


  
void loop() 
{
  unsigned long now = millis();
  
  if (now - last_cycle > interval) 
  {
    loop_micros = micros();
    last_cycle = now;

    unsigned long cur_time = millis();   // Just one call to millis()
    fsm_triggerSonar_Front.tis = cur_time - fsm_triggerSonar_Front.tes;
    fsm_triggerSonar_Right.tis = cur_time - fsm_triggerSonar_Right.tes;
    fsm_triggerSonar_Left.tis = cur_time - fsm_triggerSonar_Left.tes;

    Serial.print(" cont: ");
    Serial.print(cont_f);
    Serial.print(cont_r);
    Serial.print(cont_l);
    Serial.print(" States: ");
    Serial.print(fsm_triggerSonar_Front.state);
    Serial.print(fsm_triggerSonar_Right.state);
    Serial.println(fsm_triggerSonar_Left.state);
    

    sleep_ms(1000);


    if (fsm_triggerSonar_Front.state == 0
        && fsm_triggerSonar_Right.state == 0
        && fsm_triggerSonar_Left.state == 0 ){

      send_trigger();
      fsm_triggerSonar_Front.new_state = 1;
      fsm_triggerSonar_Right.new_state = 1;
      fsm_triggerSonar_Left.new_state = 1;
    }
    // wait up 
    if (fsm_triggerSonar_Front.state == 1 && fsm_triggerSonar_Front.tis >= 3000){
      fsm_triggerSonar_Front.new_state = 0;
    } if (fsm_triggerSonar_Right.state == 1 && fsm_triggerSonar_Right.tis >= 3000){
      fsm_triggerSonar_Right.new_state = 0;
    } if (fsm_triggerSonar_Left.state == 1 && fsm_triggerSonar_Left.tis >= 3000){
      fsm_triggerSonar_Left.new_state = 0;
    }
    // wait down
    if (fsm_triggerSonar_Front.state == 2 && fsm_triggerSonar_Front.tis >= 3000){
      fsm_triggerSonar_Front.new_state = 0;
    } if (fsm_triggerSonar_Right.state == 2 && fsm_triggerSonar_Right.tis >= 3000){
      fsm_triggerSonar_Right.new_state = 0;
    } if (fsm_triggerSonar_Left.state == 2 && fsm_triggerSonar_Left.tis >= 3000){
      fsm_triggerSonar_Left.new_state = 0;
    }

    set_state(fsm_triggerSonar_Front, fsm_triggerSonar_Front.new_state);
    set_state(fsm_triggerSonar_Right, fsm_triggerSonar_Right.new_state);
    set_state(fsm_triggerSonar_Left, fsm_triggerSonar_Left.new_state);

    Serial.print(" Distance to wall (FRONT): ");
    Serial.print(String(distance_cm_front));
    Serial.print(" Distance to wall (RIGHT): ");
    Serial.print(String(distance_cm_right));
    Serial.print(" Distance to wall (LEFT): ");
    Serial.println(String(distance_cm_left));
  }
}

