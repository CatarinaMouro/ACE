#include <Arduino.h>

#define LED_PIN 25

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

#define N 3

#define DESIRED_DIST 10
#define DESIRED_DIST_FRONT 5
#define DESIRED_DIST_FRONT_L 2
#define MARGEM 5
#define MARGEM_FRONT_init 15
#define MARGEM_FRONT_fim 4 * MARGEM_FRONT_init
#define DIST_MAX 500
#define DIST_MIN 10
#define VAL_MAX 50
#define VAL_MAX_TURN 80
#define VAL_MAX_LINEAR 170
#define SPEED_LINEAR 100
#define SPEED_TURN 70
#define MAX_BACK_SPEED -100

#define RIGHT 1
#define LEFT 2
#define TIMEOUT 5000

unsigned long interval, last_cycle, intv_motors, init_motors;
unsigned long loop_micros;

int DIRECTION;

typedef struct
{
  int state, new_state;
  unsigned long tes, tis;
} fsm_t;

void set_state(fsm_t &fsm, int new_state)
{
  if (fsm.state != new_state)
  { // if the state chnanged tis is reset
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.tis = 0;
  }
}

fsm_t fsm_cntr;
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

volatile int valores_right[N];
volatile int valores_left[N];
volatile int cont_r, cont_l;

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

void Sonar_receiveecho_front()
{
  int sonar_echo_f = digitalRead(SONAR_FRONT_PIN_echo);

  if (fsm_triggerSonar_Front.state == 1)
  {
    Echotime_init_front = micros();
    fsm_triggerSonar_Front.new_state = 2;
    set_state(fsm_triggerSonar_Front, fsm_triggerSonar_Front.new_state);
  }
  else if (sonar_echo_f == LOW && fsm_triggerSonar_Front.state == 2)
  {
    duration_sound_front = micros() - Echotime_init_front;
    prev_dist_front = distance_cm_front;
    distance_cm_front = microsecondsToCentimeters(duration_sound_front);
    fsm_triggerSonar_Front.new_state = 0;
    set_state(fsm_triggerSonar_Front, fsm_triggerSonar_Front.new_state);
  }
}

void Sonar_receiveecho_right()
{
  int sonar_echo_r = digitalRead(SONAR_RIGHT_PIN_echo);

  if (sonar_echo_r == HIGH && fsm_triggerSonar_Right.state == 1)
  {
    Echotime_init_right = micros();
    fsm_triggerSonar_Right.new_state = 2;
    set_state(fsm_triggerSonar_Right, fsm_triggerSonar_Right.new_state);
  }
  else if (sonar_echo_r == LOW && fsm_triggerSonar_Right.state == 2)
  {
    duration_sound_right = micros() - Echotime_init_right;
    prev_dist_right = distance_cm_right;
    distance_cm_right = (sqrt(2) / 2) * microsecondsToCentimeters(duration_sound_right);
    valores_right[cont_r] = distance_cm_right;
    cont_r = cont_r + 1;
    if (cont_r > 4)
      cont_r = 0;
    fsm_triggerSonar_Right.new_state = 0;
    set_state(fsm_triggerSonar_Right, fsm_triggerSonar_Right.new_state);
  }
}

void Sonar_receiveecho_left()
{
  int sonar_echo_l = digitalRead(SONAR_LEFT_PIN_echo);

  if (fsm_triggerSonar_Left.state == 1)
  {
    Echotime_init_left = micros();
    fsm_triggerSonar_Left.new_state = 2;
    set_state(fsm_triggerSonar_Left, fsm_triggerSonar_Left.new_state);
  }
  else if (sonar_echo_l == LOW && fsm_triggerSonar_Left.state == 2)
  {
    duration_sound_left = micros() - Echotime_init_left;
    prev_dist_left = distance_cm_left;
    distance_cm_left = (sqrt(2) / 2) * microsecondsToCentimeters(duration_sound_left);
    valores_left[cont_l] = distance_cm_left;
    cont_l = cont_l + 1;
    if (cont_l > 4)
      cont_l = 0;
    fsm_triggerSonar_Left.new_state = 0;
    set_state(fsm_triggerSonar_Left, fsm_triggerSonar_Left.new_state);
  }
}

void send_trigger()
{
  digitalWrite(SONAR_FRONT_PIN_trig, LOW);
  digitalWrite(SONAR_RIGHT_PIN_trig, LOW);
  digitalWrite(SONAR_LEFT_PIN_trig, LOW);
  delayMicroseconds(2);
  digitalWrite(SONAR_FRONT_PIN_trig, HIGH);
  digitalWrite(SONAR_RIGHT_PIN_trig, HIGH);
  digitalWrite(SONAR_LEFT_PIN_trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_FRONT_PIN_trig, LOW);
  digitalWrite(SONAR_RIGHT_PIN_trig, LOW);
  digitalWrite(SONAR_LEFT_PIN_trig, LOW);
}

int minimo_right()
{
  int min = valores_right[0];
  for (int i = 1; i < N; i++)
  {
    if (valores_right[i] < min)
      min = valores_right[i];
  }
  return min;
}

int minimo_left()
{
  int min = valores_left[0];
  for (int i = 1; i < N; i++)
  {
    if (valores_left[i] < min)
      min = valores_left[i];
  }
  return min;
}

/*---------------------------------------------------------------/
/--------------------------MOTORES-------------------------------/
/---------------------------------------------------------------*/

volatile int count_wheel_R, count_wheel_L, dir_R, dir_L;
int wheel_R, wheel_L;

void wheelA_R()
{
  count_wheel_R = count_wheel_R + 1;
}
void wheelB_R()
{
  count_wheel_R = count_wheel_R + 1;
}

void wheelA_L()
{
  count_wheel_L = count_wheel_L + 1;
}
void wheelB_L()
{
  count_wheel_L = count_wheel_L + 1;
}

void set_motor(int value_r, int value_l)
{
  digitalWrite(IN1_MOTOR_RIGHT, (value_r > 0));
  digitalWrite(IN2_MOTOR_RIGHT, !(value_r > 0));
  digitalWrite(IN1_MOTOR_LEFT, !(value_l > 0));
  digitalWrite(IN2_MOTOR_LEFT, (value_l > 0));

  analogWrite(PWM_MOTOR_RIGHT, abs(value_r)); // PWM Speed Control
  analogWrite(PWM_MOTOR_LEFT, abs(value_l));  // PWM Speed Control
}

void move(int rotation_speed, int linear_speed)
{
  if (abs(rotation_speed) > VAL_MAX_TURN)
  {
    if (rotation_speed < 0)
      rotation_speed = -VAL_MAX_TURN;
    else
      rotation_speed = VAL_MAX_TURN;
  }
  if (abs(linear_speed) > VAL_MAX_LINEAR)
  {
    if (linear_speed < 0)
      linear_speed = -VAL_MAX_LINEAR;
    else
      linear_speed = VAL_MAX_LINEAR;
  }

  float K_r = 0.25; // K=1.18;
  int speed_r = linear_speed + rotation_speed;
  int speed_l = linear_speed - rotation_speed;

  int err_w = speed_r - speed_l;
  int err_r = wheel_R - wheel_L;
  speed_r = speed_r + K_r * (abs(err_w) - err_r);

  if (speed_r > 250)
    speed_r = 250;
  else if (linear_speed > 0 && speed_r < 0)
    speed_r = 0; // ?? TIRAR O = ??
  else if (linear_speed < 0 && speed_r < MAX_BACK_SPEED)
    speed_r = MAX_BACK_SPEED;

  if (speed_l > 250)
    speed_l = 250;
  else if (linear_speed > 0 && speed_l < 0)
    speed_l = 0;
  else if (linear_speed < 0 && speed_l < MAX_BACK_SPEED)
    speed_l = MAX_BACK_SPEED;

  
  Serial.print("\nSpeed_Linear: ");
  Serial.print(linear_speed);
  Serial.print("| Speed_R: ");
  Serial.print(speed_r);
  Serial.print("| Speed_L: ");
  Serial.print(speed_l);
  

  set_motor(speed_r, speed_l);
}

void turn_right(int Linear)
{
  int turn = SPEED_TURN;
  if (distance_cm_left < MARGEM_FRONT_init)
    turn = turn + 1.15 * distance_cm_left;
  move(-turn, Linear);
}
void turn_left(int Linear)
{
  int turn = SPEED_TURN;
  if (distance_cm_right < MARGEM_FRONT_init)
    turn = turn + 1.15 * distance_cm_right;
  move(turn, Linear);
}
void move_stop()
{
  set_motor(0, 0);
}

// JÁ N É PRECISO
void move_linear(int linear)
{
  int K_r = 1.1;

  int err_r = wheel_R - wheel_L;
  int linear_r = linear - K_r * err_r;

  if (linear > 250)
    linear = 250;
  if (linear_r > 250)
    linear_r = 250;

  set_motor(linear_r, linear);
}

/*---------------------------------------------------------------/
/-------------------------CONTROLLERS----------------------------/
/---------------------------------------------------------------*/

int last_error_front, last_error_right, last_error_left;
int integrate_front, integrate_right, integrate_left;

int follow_right()
{
  float Ke_p = 0.8, Ki_p = 0.000, Kd_p = 0;
  float Ke_n = 5, Ki_n = 0.000, Kd_n = 10;
  int dist = minimo_right();
  int error_right = dist - DESIRED_DIST;
  integrate_right = integrate_right + error_right;
  if (integrate_right > VAL_MAX)
    integrate_right = VAL_MAX;
  if (integrate_right < -VAL_MAX)
    integrate_right = -VAL_MAX;
  int derivative_right = error_right - last_error_right;

  int rotation;
  if (error_right > 0)
  {
    rotation = Ke_p * error_right + Ki_p * integrate_right + Kd_p * derivative_right;
  }
  else
  {
    rotation = Ke_n * error_right + Ki_n * integrate_right + Kd_n * derivative_right;
  }

  last_error_right = error_right;
/*
  Serial.print("\nDist_right: ");
  Serial.print(String(dist));
  Serial.print(" | Error_right: ");
  Serial.print(String(error_right));
  Serial.print(" | Integrate_right: ");
  Serial.print(String(integrate_right));
  Serial.print(" | derivative_right: ");
  Serial.print(String(derivative_right));
  Serial.print("| rotation: ");
  Serial.print(String(rotation));
*/
  return -rotation;
}

int follow_front(){
  float Ke = 7, Ki = 0, Kd = 0;
  int error_front = distance_cm_front - DESIRED_DIST_FRONT;
  integrate_front = integrate_front + error_front;
  if (integrate_front > VAL_MAX)
    integrate_front = VAL_MAX;
  if (integrate_front < -VAL_MAX)
    integrate_front = -VAL_MAX;
  int derivative_front = error_front - last_error_front;

  int linear = Ke * error_front + Ki * integrate_right + Kd * derivative_front;
  if (linear < 0)
    linear = 0;

  last_error_front = error_front;

  return linear;
}

int follow_left()
{
  float Ke_p = 0.4, Ki_p = 0.00, Kd_p = 0;
  float Ke_n = 6.5, Ki_n = 0.00, Kd_n = 10;
  int dist = minimo_left();
  int error_left = dist - (DESIRED_DIST+5);
  integrate_left = integrate_left + error_left;
  if (integrate_left > VAL_MAX)
    integrate_left = VAL_MAX;
  if (integrate_left < -VAL_MAX)
    integrate_left = -VAL_MAX;
  int derivative_left = error_left - last_error_left;

  int rotation;
  if (error_left > 0)
  {
    rotation = Ke_p * error_left + Ki_p * integrate_left + Kd_p * derivative_left;
  }
  else
  {
    rotation = Ke_n * error_left + Ki_n * integrate_left + Kd_n * derivative_left;
  }

  last_error_left = error_left;

  /*
  Serial.print("\nDist_left: ");
  Serial.print(String(dist));
  Serial.print(" | Error_left: ");
  Serial.print(String(error_left));
  Serial.print(" | Integrate_left: ");
  Serial.print(String(integrate_left));
  Serial.print(" | derivative_left: ");
  Serial.print(String(derivative_left));
  Serial.print("| rotation: ");
  Serial.print(String(rotation));
  */

  return rotation;
}

/*---------------------------------------------------------------/
/-----------------------SETUP & LOOP-----------------------------/
/---------------------------------------------------------------*/

void setup()
{
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

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

  pinMode(ENCODER_A_RIGHT, INPUT);
  pinMode(ENCODER_B_RIGHT, INPUT);
  pinMode(ENCODER_A_LEFT, INPUT);
  pinMode(ENCODER_B_LEFT, INPUT);

  interval = 40;
  intv_motors = 100;
  DIRECTION = 0;
  count_wheel_R = 0;
  count_wheel_L = 0;
  dir_R = 0;
  dir_L = 0;
  cont_r = 0;
  cont_l = 0;

  fsm_triggerSonar_Front.state = 0;
  fsm_triggerSonar_Right.state = 0;
  fsm_triggerSonar_Left.state = 0;
  fsm_right.state = 0;
  fsm_left.state = 1;
  fsm_cntr.state = 2;

  attachInterrupt(digitalPinToInterrupt(SONAR_FRONT_PIN_echo), Sonar_receiveecho_front, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SONAR_RIGHT_PIN_echo), Sonar_receiveecho_right, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SONAR_LEFT_PIN_echo), Sonar_receiveecho_left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_RIGHT), wheelA_R, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_LEFT), wheelA_L, RISING);
  // attachInterrupt(digitalPinToInterrupt(ENCODER_B_RIGHT), wheelB_R, RISING);
  // attachInterrupt(digitalPinToInterrupt(ENCODER_B_LEFT), wheelB_L, RISING);
}


unsigned long init_LED;
bool estado=0;
void loop()
{
  unsigned long now = millis();

  if ((now - init_motors) > intv_motors)
  {
    wheel_R = count_wheel_R;
    wheel_L = count_wheel_L;
    count_wheel_R = 0;
    count_wheel_L = 0;
    init_motors = now;
  }

  if ((now - init_LED) > 1000){
    digitalWrite(LED_PIN, estado);
    estado=!estado;
    init_LED=millis();
  }

  if (now - last_cycle > interval)
  {
    loop_micros = micros();
    last_cycle = now;
    unsigned long cur_time = millis(); // Just one call to millis()

    fsm_cntr.tis = cur_time - fsm_cntr.tes;
    fsm_right.tis = cur_time - fsm_right.tes;
    fsm_left.tis = cur_time - fsm_left.tes;

    //-----------------SENSORES-------------------//

    fsm_triggerSonar_Front.tis = cur_time - fsm_triggerSonar_Front.tes;
    fsm_triggerSonar_Right.tis = cur_time - fsm_triggerSonar_Right.tes;
    fsm_triggerSonar_Left.tis = cur_time - fsm_triggerSonar_Left.tes;

    if (fsm_triggerSonar_Front.state == 0 && fsm_triggerSonar_Right.state == 0 && fsm_triggerSonar_Left.state == 0){
      send_trigger();
      fsm_triggerSonar_Front.new_state = 1;
      fsm_triggerSonar_Right.new_state = 1;
      fsm_triggerSonar_Left.new_state = 1;
    }
    // wait up
    if (fsm_triggerSonar_Front.state == 1 && fsm_triggerSonar_Front.tis >= 3){
      fsm_triggerSonar_Front.new_state = 0;
      }
    if (fsm_triggerSonar_Right.state == 1 && fsm_triggerSonar_Right.tis >= 3){
      fsm_triggerSonar_Right.new_state = 0;
      }
    if (fsm_triggerSonar_Left.state == 1 && fsm_triggerSonar_Left.tis >= 3){
      fsm_triggerSonar_Left.new_state = 0;
    }
    // wait down
    if (fsm_triggerSonar_Front.state == 2 && fsm_triggerSonar_Front.tis >= 3){
      distance_cm_front=200;
      fsm_triggerSonar_Front.new_state = 0;
    }
    if (fsm_triggerSonar_Right.state == 2 && fsm_triggerSonar_Right.tis >= 3){
      distance_cm_right = 200;
      valores_right[cont_r] = distance_cm_right;
      cont_r = cont_r + 1;
      if (cont_r > 4)
        cont_r = 0;
      fsm_triggerSonar_Right.new_state = 0;
    }
    if (fsm_triggerSonar_Left.state == 2 && fsm_triggerSonar_Left.tis >= 3){
      distance_cm_left = 200;
      valores_left[cont_l] = distance_cm_left;
      cont_l = cont_l + 1;
      if (cont_l > 4)
        cont_l = 0;
      fsm_triggerSonar_Left.new_state = 0;
    }

    set_state(fsm_triggerSonar_Front, fsm_triggerSonar_Front.new_state);
    set_state(fsm_triggerSonar_Right, fsm_triggerSonar_Right.new_state);
    set_state(fsm_triggerSonar_Left, fsm_triggerSonar_Left.new_state);

/*
    Serial.print("| fsm_cntr: ");
    Serial.print(fsm_cntr.state);
    Serial.print("| fsm_right: ");
    Serial.print(fsm_right.state);
    Serial.print("| fsm_left: ");
    Serial.print(fsm_left.state);
*/
    Serial.print(" | dist_right: ");
    Serial.print(distance_cm_right);
    Serial.print("| dist_front: ");
    Serial.print(distance_cm_front);
    Serial.print("| dist_left: ");
    Serial.print(distance_cm_left);

    //-----------------FSM RIGHT-------------------//
    if (fsm_cntr.state != 1){
      fsm_right.new_state = 0;
    }
    else if (fsm_right.state == 1 && (distance_cm_left > (DESIRED_DIST + MARGEM) // left desimpedido
                                  && distance_cm_front < (DESIRED_DIST_FRONT + MARGEM_FRONT_init))){ // front impedido
      fsm_right.new_state = 2;
    }
    else if (fsm_right.state == 1 && ((distance_cm_left < (DESIRED_DIST + MARGEM)                       // left impedido
                                  && distance_cm_front < (DESIRED_DIST_FRONT + MARGEM_FRONT_init)) // front impedido
                                  || (distance_cm_left < (MARGEM)))){
      fsm_right.new_state = 3;
    }
    else if (fsm_right.state == 1 && (distance_cm_right > 4 * (DESIRED_DIST + MARGEM))){ // right desimpedido
      fsm_right.new_state = 4;
    }
    else if (fsm_right.state == 2 && distance_cm_front > (DESIRED_DIST_FRONT + MARGEM_FRONT_fim)){ // front desimpedido
      fsm_right.new_state = 1;
    }
    else if (fsm_right.state == 2 && (distance_cm_left < (MARGEM) // left impedido
                                  || distance_cm_front < (MARGEM))){ // front impedido
      fsm_right.new_state = 3;
    }
    else if (fsm_right.state == 3 && distance_cm_left > (DESIRED_DIST + MARGEM)){ // left desimpedido
      fsm_right.new_state = 2;
    }
    else if (fsm_right.state == 4 && distance_cm_right < (DESIRED_DIST + 3*MARGEM)){ // right impedido
      fsm_right.new_state = 1;
    }
    set_state(fsm_right, fsm_right.new_state);

    if (fsm_right.state == 1){
      int rotation = follow_right();
      int linear = follow_front();
      move(rotation, linear);
    }
    else if (fsm_right.state == 2){
      int linear = follow_front();
      turn_left(0.4 * linear);
    }
    else if (fsm_right.state == 3)
      move(0, MAX_BACK_SPEED);
    else if (fsm_right.state == 4){
      int linear = follow_front();
      turn_right(0.2 * linear);
    }

    //-----------------FSM LEFT-------------------//
    if (fsm_cntr.state!=2){
      fsm_left.new_state=0;
    }
    else if(fsm_left.state==1 && (distance_cm_right > (DESIRED_DIST+MARGEM)      
                              && distance_cm_front < (DESIRED_DIST_FRONT+MARGEM_FRONT_init))){  
      fsm_left.new_state=2;
    }
    else if(fsm_left.state==1 && ((distance_cm_right < (DESIRED_DIST+MARGEM)      
                              && distance_cm_front < (DESIRED_DIST_FRONT+MARGEM_FRONT_init)) 
                              || (distance_cm_right < (MARGEM)))){
      fsm_left.new_state=3;
    }
    else if(fsm_left.state==1 && (distance_cm_left > 4 * (DESIRED_DIST+MARGEM))){      
      fsm_left.new_state=4;
    }
    else if(fsm_left.state==2 && distance_cm_front > (DESIRED_DIST_FRONT+MARGEM_FRONT_fim)){  
      fsm_left.new_state=1;
    }
    else if(fsm_left.state==2 && (distance_cm_right < (MARGEM)    
                              || distance_cm_front < (MARGEM))){   
      fsm_left.new_state=3;
    }
    else if(fsm_left.state==3 && distance_cm_right > (DESIRED_DIST+MARGEM)){   
      fsm_left.new_state=2;
    }
    else if(fsm_left.state==4 && distance_cm_left < (DESIRED_DIST+3*MARGEM)){
      fsm_left.new_state=1;
    }

    set_state(fsm_left, fsm_left.new_state);

    if (fsm_left.state == 1){
      int rotation = follow_left();
      int linear = follow_front();
      move(rotation, linear);
    }
    else if (fsm_left.state == 2 || fsm_left.state == 5){
      int linear = follow_front();
      turn_right(0.38 * linear);
    }
    else if (fsm_left.state == 3)  
      move(0, MAX_BACK_SPEED);
    else if (fsm_left.state == 4){
      int linear = follow_front();
      turn_left(0.28 * linear);
    }



/*
     //-----------------FSM CONTROL-------------------//
    if (fsm_cntr.state==0 && ((distance_cm_right<50 && distance_cm_right>0)
                          || (distance_cm_front<30 && distance_cm_left>50))){
      DIRECTION=RIGHT;
      fsm_cntr.new_state=1;
      
      if (distance_cm_left > (DESIRED_DIST + MARGEM) // left desimpedido
        && distance_cm_front < (DESIRED_DIST_FRONT + MARGEM_FRONT_init)){ // front impedido
        fsm_right.new_state = 2;
      }
      else fsm_right.new_state = 1;
      set_state(fsm_right, fsm_right.new_state);

    }
    else if (fsm_cntr.state==0 && (distance_cm_left<50 && distance_cm_left>0)){
      DIRECTION=LEFT;
      fsm_cntr.new_state=2;

      if (distance_cm_right > (DESIRED_DIST + MARGEM) 
        && distance_cm_front < (DESIRED_DIST_FRONT_L + MARGEM_FRONT_init)){
          fsm_left.new_state = 2;
      }
      else fsm_left.new_state = 1;
      set_state(fsm_left, fsm_left.new_state);
    }
    else if (fsm_cntr.state==1 && fsm_right.state==4 && fsm_right.tis>TIMEOUT){
      fsm_cntr.new_state=0;
    }
    else if (fsm_cntr.state==2 && fsm_left.state==4 && fsm_left.tis>TIMEOUT){
      fsm_cntr.new_state=0;
    }
    else if(fsm_cntr.state==3 && DIRECTION==RIGHT && (distance_cm_right<50 || distance_cm_front<30)){
      fsm_cntr.new_state=1;
      
      if (distance_cm_left > (DESIRED_DIST + MARGEM) // left desimpedido
        && distance_cm_front < (DESIRED_DIST_FRONT + MARGEM_FRONT_init)){ // front impedido
        fsm_right.new_state = 2;
      }
      else fsm_right.new_state = 1;
      set_state(fsm_right, fsm_right.new_state);
    }
    else if(fsm_cntr.state==3 && DIRECTION==LEFT && (distance_cm_left<50 || distance_cm_front<30)){
      fsm_cntr.new_state=2;

      if (distance_cm_right > (DESIRED_DIST + MARGEM) 
        && distance_cm_front < (DESIRED_DIST_FRONT_L + MARGEM_FRONT_init)){
          fsm_left.new_state = 2;
      }
      else fsm_left.new_state = 1;
      set_state(fsm_left, fsm_left.new_state);
    }
    set_state(fsm_cntr, fsm_cntr.new_state);

    if(fsm_cntr.state==0 || fsm_cntr.state==3) move(0, VAL_MAX_LINEAR);
*/

    // int rotation=follow_right();
    // int rotation=follow_left();
    // int linear=follow_front();
    // move(rotation, linear);
  }

}