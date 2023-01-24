#include <Arduino.h>


#define SONAR_RIGHT_PIN_trig 0
#define SONAR_RIGHT_PIN_echo 1
#define SONAR_LEFT_PIN_trig 7
#define SONAR_LEFT_PIN_echo 6
#define SONAR_FRONT_PIN_trig 8
#define SONAR_FRONT_PIN_echo 9

#define ENCODER_A_RIGHT 21
#define ENCODER_B_RIGHT 20
#define ENCODER_A_LEFT 19
#define ENCODER_B_LEFT 18
#define PWM_MOTOR_RIGHT 15
#define IN1_MOTOR_RIGHT 13
#define IN2_MOTOR_RIGHT 14
#define PWM_MOTOR_LEFT 10
#define IN1_MOTOR_LEFT 12
#define IN2_MOTOR_LEFT 11

#define DESIRED_DIST 25
#define MARGEM 0
#define DIST_MAX 500
#define DIST_MIN 10
#define VAL_MAX 50
#define SPEED_LINEAR 100
#define SPEED_TURN 50

unsigned long interval, last_cycle, intv_motors, init_motors;
unsigned long loop_micros;

int FOUND;
int DIRECTION; 
//DIR=0 ~> follow_right
//DIR=1 ~> follow_left


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

fsm_t fsm_cntr;
fsm_t fsm_find;
fsm_t fsm_right;
fsm_t fsm_left;

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

volatile int valores_right[5];
volatile int valores_left[5];
volatile int valores_front[5];
volatile int cont_f, cont_r, cont_l;


// GET DISTANCES
// Max Range: 4m
// Min Range: 2cm
long duration_sound_front, distance_cm_front, prev_dist_front;
long duration_sound_right, distance_cm_right, prev_dist_right;
long duration_sound_left, distance_cm_left, prev_dist_left;



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
    prev_dist_front = distance_cm_front;
    distance_cm_front=microsecondsToCentimeters(duration_sound_front);
    valores_front[cont_f]=distance_cm_front;
    cont_f=cont_f+1;
    if(cont_f>4) cont_f=0;
    fsm_triggerSonar_Front.new_state = 0;
    set_state(fsm_triggerSonar_Front, fsm_triggerSonar_Front.new_state);
  }
}

void Sonar_receiveecho_right(){
  int sonar_echo_r=digitalRead(SONAR_RIGHT_PIN_echo);

  if (sonar_echo_r==HIGH && fsm_triggerSonar_Right.state==1){
    Echotime_init_right=micros();
    fsm_triggerSonar_Right.new_state = 2;
    set_state(fsm_triggerSonar_Right, fsm_triggerSonar_Right.new_state);
  }
  else if (sonar_echo_r==LOW && fsm_triggerSonar_Right.state==2){
    duration_sound_right=micros()-Echotime_init_right;
    prev_dist_right = distance_cm_right;
    distance_cm_right=microsecondsToCentimeters(duration_sound_right);
    valores_right[cont_r]=distance_cm_right;
    cont_r=cont_r+1;
    if(cont_r>4) cont_r=0;
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
    prev_dist_left = distance_cm_left;
    distance_cm_left=microsecondsToCentimeters(duration_sound_left);
    valores_left[cont_l]=distance_cm_left;
    cont_l=cont_l+1;
    if(cont_l>4) cont_l=0;
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


int minimo_right(){
  int min=valores_right[0];
  for(int i=1; i<5; i++){
    if (valores_right[i]<min) min = valores_right[i];
  }
  return min;
} 




/*---------------------------------------------------------------/
/--------------------------MOTORES-------------------------------/
/---------------------------------------------------------------*/

volatile int count_wheel_R, count_wheel_L, dir_R, dir_L;
int wheel_R, wheel_L;





void wheelA_R(){
  count_wheel_R = count_wheel_R+1;
}
void wheelB_R(){
  count_wheel_R = count_wheel_R+1;
}


void wheelA_L(){
  count_wheel_L = count_wheel_L+1;
}
void wheelB_L(){
  count_wheel_L = count_wheel_L+1;
}

void set_motor(int value_r,int value_l){
  digitalWrite(IN1_MOTOR_RIGHT,  (value_r>0));   
  digitalWrite(IN2_MOTOR_RIGHT, !(value_r>0));  
  digitalWrite(IN1_MOTOR_LEFT,  !(value_l>0));   
  digitalWrite(IN2_MOTOR_LEFT,   (value_l>0));  
  
  analogWrite(PWM_MOTOR_RIGHT, abs(value_r));    //PWM Speed Control
  analogWrite(PWM_MOTOR_LEFT,  abs(value_l));    //PWM Speed Control
}

int move(int rotation_speed, int linear_speed){
  if(abs(rotation_speed)>50){
    if(rotation_speed<0) rotation_speed=-50;
    else rotation_speed=50;
  }

  /*linear_speed=linear_speed-0.2*rotation_speed;
  if(linear_speed>250) linear_speed=250;
  else if(linear_speed<0) linear_speed=0;
*/  


  int speed_r = linear_speed + rotation_speed;
  int speed_l = linear_speed - rotation_speed;
  int res=0;

  int K_r=1.04, K=1.18;
  int err_w = speed_r - speed_l;
  int err_r = wheel_R - wheel_L;
  speed_r = speed_r + K_r*(err_w-err_r);


  if(speed_r>250) {speed_r=250; res=-1;}
  if(speed_l>250) {speed_l=250; res=-1;}

  Serial.print("| Speed_R: ");
  Serial.print(speed_r);
  Serial.print("| Speed_L: ");
  Serial.print(speed_l);

  set_motor(speed_r,speed_l);

  return res;
}


void turn_right(){
  move(-SPEED_TURN, SPEED_LINEAR);
}
void turn_left(){
  move(SPEED_TURN, SPEED_LINEAR);
}
void move_stop(){
  set_motor(0, 0);
}



/*---------------------------------------------------------------/
/-------------------------CONTROLLERS----------------------------/
/---------------------------------------------------------------*/

int last_error_front, last_error_right, last_error_left;
int  integrate_front,  integrate_right,  integrate_left;


int follow_right(){
  float Ke_p=0.2, Ki_p=0.000, Kd_p=0;
  float Ke_n=3, Ki_n=0.000, Kd_n=0;
  int dist = minimo_right();
  int error_right = dist - DESIRED_DIST;
  integrate_right = integrate_right + error_right;
  if(integrate_right>VAL_MAX) integrate_right=VAL_MAX;
  if(integrate_right<-VAL_MAX) integrate_right=-VAL_MAX;
  int derivative_right = error_right - last_error_right;
  
  int rotation;
  if(error_right>0){
    rotation = Ke_p*error_right + Ki_p*integrate_right + Kd_p*derivative_right;
  }
  else{
    rotation = Ke_n*error_right + Ki_n*integrate_right + Kd_n*derivative_right;
  }
  
  Serial.print("\nDist: ");
  Serial.print(distance_cm_right);
  Serial.print(" | Dist_Right: ");
  Serial.print(String(dist));
  Serial.print(" | Error_Right: ");
  Serial.print(String(error_right));
  Serial.print(" | Integrate_Right: ");
  Serial.print(String(integrate_right));
  Serial.print(" | derivative_right: ");
  Serial.print(String(derivative_right));
  Serial.print("| rotation: ");
  Serial.print(String(rotation));

  last_error_right=error_right;

  return -rotation;
}

int follow_front(){
  float Ke=0.75, Ki=0.00001, Kd=0.003;
  int error_front = distance_cm_front - DESIRED_DIST;
  integrate_front = integrate_front + error_front;
  if(integrate_front>VAL_MAX) integrate_front=VAL_MAX;
  if(integrate_front<-VAL_MAX) integrate_front=-VAL_MAX;
  int derivative_front = error_front - last_error_front;
  
  int linear = Ke*error_front; //+Ki*integrate_right + Kd*derivative_front;

  
  Serial.print("\nError_Front: ");
  Serial.print(String(error_front));
  Serial.print(" | Integrate_Front: ");
  Serial.print(String(integrate_front));
  Serial.print("| Linear: ");
  Serial.print(String(linear));

  last_error_front=error_front;

  return linear;
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
  intv_motors = 100;
  FOUND=0;
  DIRECTION=0;
  count_wheel_R=0;
  count_wheel_L=0;
  dir_R=0;
  dir_L=0;
  cont_r=0;

  
  fsm_triggerSonar_Front.state=0;
  fsm_triggerSonar_Right.state=0;
  fsm_triggerSonar_Left.state=0;
  fsm_right.state=0;
  fsm_left.state=0;
  fsm_cntr.state=1;
  fsm_find.state=0;

  attachInterrupt(digitalPinToInterrupt(SONAR_FRONT_PIN_echo), Sonar_receiveecho_front, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SONAR_RIGHT_PIN_echo), Sonar_receiveecho_right, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SONAR_LEFT_PIN_echo), Sonar_receiveecho_left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_RIGHT), wheelA_R, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_LEFT), wheelA_L, RISING);
  //attachInterrupt(digitalPinToInterrupt(ENCODER_B_RIGHT), wheelB_R, RISING);
  //attachInterrupt(digitalPinToInterrupt(ENCODER_B_LEFT), wheelB_L, RISING);

}


void loop() 
{
  unsigned long now = millis();

  if(now-init_motors > intv_motors){

    Serial.print(" | Motor Right: ");
    Serial.print(count_wheel_R);
    Serial.print(" Motor Left: ");
    Serial.print(count_wheel_L);
    wheel_R=count_wheel_R;
    wheel_L=count_wheel_L;
    count_wheel_R=0;
    count_wheel_L=0;
    init_motors = now;
  }
  
  if (now - last_cycle > interval) 
  {
    loop_micros = micros();
    last_cycle = now;
    unsigned long cur_time = millis();   // Just one call to millis()


    
    fsm_cntr.tis = cur_time - fsm_cntr.tes;
    fsm_find.tis = cur_time - fsm_find.tes;
    fsm_right.tis = cur_time - fsm_right.tes;
    fsm_left.tis = cur_time - fsm_left.tes;


    
    //-----------------SENSORES-------------------//

    fsm_triggerSonar_Front.tis = cur_time - fsm_triggerSonar_Front.tes;
    fsm_triggerSonar_Right.tis = cur_time - fsm_triggerSonar_Right.tes;
    fsm_triggerSonar_Left.tis = cur_time - fsm_triggerSonar_Left.tes;

    if (fsm_triggerSonar_Front.state == 0 ){
     //   && fsm_triggerSonar_Right.state == 0
     //   && fsm_triggerSonar_Left.state == 0 ){

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
    
*/
    
 
    int rotation=follow_right();
    //int linear=follow_front();
    move(rotation, 100);
    
  }
}


