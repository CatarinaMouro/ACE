#include <Arduino.h>
//#include <RP2040_PWM.h>



#define SONAR_RIGHT_PIN_trig 0
#define SONAR_RIGHT_PIN_echo 1
#define SONAR_LEFT_PIN_trig 7
#define SONAR_LEFT_PIN_echo 6
#define SONAR_FRONT_PIN_trig 8
#define SONAR_FRONT_PIN_echo 9

#define ENCODER_A_RIGHT 8
#define ENCODER_B_RIGHT 8
#define ENCODER_A_LEFT 8
#define ENCODER_B_LEFT 8
#define PWM_MOTOR_RIGHT 15
#define IN1_MOTOR_RIGHT 13
#define IN2_MOTOR_RIGHT 14
#define PWM_MOTOR_LEFT 10
#define IN1_MOTOR_LEFT 12
#define IN2_MOTOR_LEFT 11


unsigned long interval, last_cycle;
unsigned long loop_micros;
boolean follow; // 0->left & 1->right;



typedef struct {
  int state, new_state;
  unsigned long tes, tis;
} fsm_t;

void set_state(fsm_t& fsm, int new_state)
{
  if (fsm.state != new_state) {  // if the state chnanged tis is reset
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.tis = 0;
  }
}

fsm_t fsm_front;
fsm_t fsm_wall_right;

/*---------------------------------------------------------------/
/--------------------------SENSORES------------------------------/
/---------------------------------------------------------------*/

// Sonar Trigger variables
fsm_t fsm_triggerSonar_Front;
fsm_t fsm_triggerSonar_Right;
fsm_t fsm_triggerSonar_Left;

// Sonar Echo variables
unsigned long Echotime_init_front;
unsigned long Echotime_init_right;
unsigned long Echotime_init_left;

volatile int cont_f, cont_r, cont_l;

// GET DISTANCES
// Max Range: 4m
// Min Range: 2cm
long duration_sound_front, distance_cm_front;
long duration_sound_right, distance_cm_right;
long duration_sound_left, distance_cm_left;



// Speed of Sound: 340m/s = 29microseconds/cm
// Sound wave reflects from the obstacle, so to calculate the distance we consider half of the distance traveled.  
// DistanceInCms=microseconds/29/2 
long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29 / 2;
}



void Sonar_receiveecho_front(){
  int sonar_echo_f=digitalRead(SONAR_FRONT_PIN_echo);

  cont_f=cont_f+1;

  if (fsm_triggerSonar_Front.state==1){
    Echotime_init_front=micros();
    fsm_triggerSonar_Front.new_state = 2;
    set_state(fsm_triggerSonar_Front, fsm_triggerSonar_Front.new_state);
  } 
  else if (sonar_echo_f==LOW && fsm_triggerSonar_Front.state==2){
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






/*---------------------------------------------------------------/
/--------------------------MOTORES-------------------------------/
/---------------------------------------------------------------*/


boolean Direction;  //the rotation direction
volatile float velocity_R = 0;
volatile long prevT_R = 0;
volatile float velocity_L = 0;
volatile long prevT_L = 0;

void wheelSpeed_R()
{
  int Lstate = digitalRead(ENCODER_B_RIGHT);
  int increment = 0;
  if(Lstate==HIGH)
  {
    increment = 1;

    /*int val = digitalRead(ENCODER_B_RIGHT);
    if(val == LOW && Direction)
    {
      Direction = false; //Reverse
    }
    else if(val == HIGH && !Direction)
    {
      Direction = true;  //Forward
    }*/
  }
  else increment = -1;

  long currT = micros();
  float deltaT = ((float) (currT - prevT_R))/1.0e6;
  velocity_R = increment/deltaT;
  prevT_R = currT;
}

void wheelSpeed_L()
{
  int Lstate = digitalRead(ENCODER_B_LEFT);
  int increment = 0;
  if(Lstate==HIGH)
  {
    increment = 1;

    /*int val = digitalRead(ENCODER_B_LEFT);
    if(val == LOW && Direction)
    {
      Direction = false; //Reverse
    }
    else if(val == HIGH && !Direction)
    {
      Direction = true;  //Forward
    }*/
  }
  else increment = -1;

  long currT = micros();
  float deltaT = ((float) (currT - prevT_L))/1.0e6;
  velocity_L = increment/deltaT;
  prevT_L = currT;
}


void move(int dir,int value_r,int value_l){
  Serial.print("entrei move: ");
  Serial.print(dir);
  Serial.print(" ");
  Serial.print(value_r);
  Serial.print(" ");
  Serial.println(value_l);
  if (dir){
    digitalWrite(IN1_MOTOR_RIGHT, HIGH);   
    digitalWrite(IN2_MOTOR_RIGHT, LOW); 
    digitalWrite(IN1_MOTOR_LEFT, HIGH);   
    digitalWrite(IN2_MOTOR_LEFT, LOW);   
  }
  else{
    digitalWrite(IN1_MOTOR_RIGHT, LOW);   
    digitalWrite(IN2_MOTOR_RIGHT, HIGH); 
    digitalWrite(IN1_MOTOR_LEFT, LOW);   
    digitalWrite(IN2_MOTOR_LEFT, HIGH);   
  }
  analogWrite(PWM_MOTOR_RIGHT, value_r);   //PWM Speed Control
  analogWrite(PWM_MOTOR_LEFT, value_l);    //PWM Speed Control
}

void move_forward(){
  move(1, 255, 255);
}
void turn_right(){
  move(1, 0, 128);
}
void turn_left(){
  move(1, 128, 0);
}

/*---------------------------------------------------------------/
/-----------------------SETUP & LOOP-----------------------------/
/---------------------------------------------------------------*/


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(SONAR_FRONT_PIN_trig, OUTPUT);
  pinMode(SONAR_FRONT_PIN_echo, INPUT);
  pinMode(SONAR_RIGHT_PIN_trig, OUTPUT);
  pinMode(SONAR_RIGHT_PIN_echo, INPUT);
  pinMode(SONAR_LEFT_PIN_trig, OUTPUT);
  pinMode(SONAR_LEFT_PIN_echo, INPUT);


  pinMode(PWM_MOTOR_RIGHT, OUTPUT);   
  pinMode(IN1_MOTOR_RIGHT, OUTPUT); 
  pinMode(IN2_MOTOR_RIGHT, OUTPUT);   
  pinMode(PWM_MOTOR_LEFT, OUTPUT); 
  pinMode(IN1_MOTOR_LEFT, OUTPUT);   
  pinMode(IN2_MOTOR_LEFT, OUTPUT); 

  pinMode(ENCODER_A_RIGHT,INPUT);
  pinMode(ENCODER_B_RIGHT,INPUT);
  pinMode(ENCODER_A_LEFT,INPUT);
  pinMode(ENCODER_B_LEFT,INPUT);

  interval = 100;


  
  fsm_triggerSonar_Front.state=0;
  fsm_triggerSonar_Right.state=0;
  fsm_triggerSonar_Left.state=0;
  fsm_wall_right.state=0;
  fsm_front.state=0;
  follow = 1;

  attachInterrupt(digitalPinToInterrupt(SONAR_FRONT_PIN_echo), Sonar_receiveecho_front, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SONAR_RIGHT_PIN_echo), Sonar_receiveecho_right, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SONAR_LEFT_PIN_echo), Sonar_receiveecho_left, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(ENCODER_A_RIGHT), wheelSpeed_R, RISING);
  //attachInterrupt(digitalPinToInterrupt(ENCODER_A_LEFT), wheelSpeed_L, RISING);

}





/*

void loop() 
{
  unsigned long now = millis();
  
  if (now - last_cycle > interval) 
  {
    loop_micros = micros();
    last_cycle = now;
    unsigned long cur_time = millis();   // Just one call to millis()


    
    fsm_front.tis = cur_time - fsm_front.tes;
    fsm_wall_right.tis = cur_time - fsm_wall_right.tes;

    digitalWrite(IN1_MOTOR_RIGHT, HIGH);   
    digitalWrite(IN2_MOTOR_RIGHT, LOW); 
    digitalWrite(IN1_MOTOR_LEFT, HIGH);   
    digitalWrite(IN2_MOTOR_LEFT, LOW);   
    analogWrite(PWM_MOTOR_RIGHT, 255);   //PWM Speed Control
    analogWrite(PWM_MOTOR_LEFT, 255);    //PWM Speed Control


    /*
    //-----------------SENSORES-------------------//

    fsm_triggerSonar_Front.tis = cur_time - fsm_triggerSonar_Front.tes;
    fsm_triggerSonar_Right.tis = cur_time - fsm_triggerSonar_Right.tes;
    fsm_triggerSonar_Left.tis = cur_time - fsm_triggerSonar_Left.tes;

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

    /*
    Serial.print(" Distance to wall (FRONT): ");
    Serial.print(String(distance_cm_front));
    Serial.print(" Distance to wall (RIGHT): ");
    Serial.print(String(distance_cm_right));
    Serial.print(" Distance to wall (LEFT): ");
    Serial.println(String(distance_cm_left));
    * /

    


     //-----------------MOTORES-------------------//
    
    if (fsm_wall_right.state == 0 && fsm_wall_right.tis>1000){
      fsm_wall_right.new_state=1;
    }
    else if (fsm_wall_right.state == 1 && fsm_wall_right.tis>5000){
      fsm_wall_right.new_state=2;
    }
    else if (fsm_wall_right.state == 2 && fsm_wall_right.tis>500){
      fsm_wall_right.new_state=3;
    }
    else if (fsm_wall_right.state == 3 && fsm_wall_right.tis>500){
      fsm_wall_right.new_state=1;
    }
    set_state(fsm_wall_right, fsm_wall_right.new_state);

    if(fsm_wall_right.state==1) move_forward();
    else if(fsm_wall_right.state==2) turn_left();
    else if(fsm_wall_right.state==3) turn_right();

    
    Serial.print("fsm_wall_right: ");
    Serial.println(fsm_wall_right.state);
    * /


  }
}

*/
