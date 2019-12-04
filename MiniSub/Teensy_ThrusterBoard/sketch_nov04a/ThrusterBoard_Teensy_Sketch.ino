//#include <ArduinoTcpHardware.h>
//#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/UInt64.h>

//this just needs to be declared before anything else because reasons
ros::NodeHandle nh;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//STRUCT DEFINITIONS

//Thruster struct
typedef struct Thruster {
  unsigned char duty_cycle = 255; //baseline = off; 0 = led1 on; 2*baseline = led2 on;
  unsigned char pin;
}thruster;

//Task struct
typedef struct Tasks {
  int state = 0;
  unsigned long period = 0;
  unsigned long elapsed_time = 0;
  int (*TickFct)(int);
  bool Active = true;
} task;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//GLOBAL VARIABLES

int global_period = 25000;
IntervalTimer myTimer;

thruster BASELINE;
char BASELINE_PIN = 9;
char BASELINE_VAL = 125;

thruster Thruster_Array[8];
int Pin_Array[8] = {3,4,5,6,20,21,22,23}; 

const unsigned char  tasks_size = 2;
task Tasks[tasks_size];

  /* THRUSTER ARRAY MAP
  0 T_FL; //frontal left 
  1 T_FR; //frontal right
  2 t_BL; //back left
  3 t_BR; //back right
  4 t_VFL; //vertical facing frontal left
  5 t_VFR; //vertical facing frontal right
  6 t_VBL; //vertical facing back left
  7 t_VBR; //vertical facing back right

  UInt64 (unsigned 64bit int) [8][8][8][8][8][8][8][8] < 64bit int
  partitioned such that       [7][6][5][4][3][2][1][0] < thruster array
  */
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//ROS stuff

//init ros 8bit int array variable
std_msgs::UInt64 arr_msg;

//runs after PollSubscribers causes the subscriber to spinonce()
//bitmasks arr_msg 
//each pin's PWM can be changed separately
void TSubCB(const std_msgs::UInt64& arr_msg) {
  unsigned char i;
  unsigned long long temp = arr_msg.data;
  for (i = 0; i < 8; i++) {
    temp = arr_msg.data;
    Thruster_Array[i].duty_cycle = ( (temp >> (8*i)) &0xFF );
  }
  Tasks[1].Active = true;
  digitalWrite(13,HIGH-digitalRead(13));
}

//init ROS subscriber to subscribe to node "ThrusterStates" and run TSubCB after it polls
ros::Subscriber<std_msgs::UInt64> TSub("ThrusterStates", &TSubCB);
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void setup() {
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(TSub);
  unsigned char i = 0;
  
  //init pin_array as output, BASELINE_PIN output, A9 input
  for(i = 0; i < 8; i++) {
    pinMode(Pin_Array[i],OUTPUT);
  }
    pinMode(BASELINE_PIN, OUTPUT);
    

  //init thruster array with pin values
  for (i = 0; i < 8; i++) {
    Thruster_Array[i].pin = Pin_Array[i];
  }
  //init BASELINE values
  BASELINE.pin = BASELINE_PIN;
  BASELINE.duty_cycle = BASELINE_VAL;
  
  //init tasks
  
  Tasks[0].state = 0;
  Tasks[0].period = global_period;
  Tasks[0].elapsed_time = 0;
  Tasks[0].TickFct = &PollSubscribers;
  Tasks[0].Active = true;//always happening, but run once on startup

  //OutputPWM (triggered SM)
  Tasks[1].state = 0;
  Tasks[1].period = 0;
  Tasks[1].elapsed_time = 0;
  Tasks[1].TickFct = &OutputPWM;
  Tasks[1].Active = true;//will need to be triggered, but run once on startup
  
  /* subscribes to jetson PWM commands and updates Thruster objects accordingly
  Tasks[1].state = 0;
  Tasks[1].period = 0; //triggered SM
  Tasks[1].elapsed_time = 0;
  Tasks[1].TickFct = &UpdateThrusters;
 */

  //init PWMs at baseline(LEDS off)
  /*
  for (i = 0; i < 8; i++){
  analogWrite(Thruster_Array[i].pin, BASELINE_VAL);
  }
  analogWrite(BASELINE.pin, BASELINE_VAL);
  */
  
  //begin timer at the end of setup
  myTimer.begin(Scheduler, global_period); //begin scheduler
  
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//runs every global_period
//just a scheduler
void Scheduler(){
  unsigned char i;
  for (i = 0; i < tasks_size; i++){
    if (Tasks[i].Active) {
      if (Tasks[i].elapsed_time >= Tasks[i].period) {
        Tasks[i].state = Tasks[i].TickFct(Tasks[i].state);
        Tasks[i].elapsed_time = 0;        
      }
      Tasks[i].elapsed_time += global_period;
    }
  }
  //digitalWrite(13,HIGH-digitalRead(13));
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//updates PWMs
//Triggered SM, trigger is in TSubCB()
int OutputPWM(int state) {
 unsigned char i;
  for (i = 0; i < 8; i++) {
   analogWrite(Thruster_Array[i].pin, Thruster_Array[i].duty_cycle);
  }
  
  //update BASELINE pwm (but like... when would you want to?)
  analogWrite(BASELINE.pin, BASELINE.duty_cycle);
  Tasks[1].Active = false;

  //digitalWrite(13,HIGH);
  return 0; 
  
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//poll ROS subcribers
int PollSubscribers(int state){
  nh.spinOnce();
  return 0;
  
}

void loop() {}
