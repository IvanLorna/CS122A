//Thruster struct
typedef struct Thruster {
  char duty_cycle = 125; //baseline = off; 0 = led1 on; 2*baseline = led2 on;
  char pin;
}thruster;

//Task struct
typedef struct Tasks {
  int state = 0;
  unsigned long period = 0;
  unsigned long elapsed_time = 0;
  int (*TickFct)(int);
  bool Active = true;
} task;

//GLOBAL
int global_period = 125000;
IntervalTimer myTimer;

thruster BASELINE;
char BASELINE_PIN = 9;
char BASELINE_VAL = 125;

thruster Thruster_Array[8];
int Pin_Array[8] = {3,4,5,6,20,21,22,23}; 

const unsigned char  tasks_size = 2;
task Tasks[tasks_size];

int Button_Pin = A9;

  /* THRUSTER ARRAY MAP
  0 T_FL; //frontal left 
  1 T_FR; //frontal right
  2 t_BL; //back left
  3 t_BR; //back right
  4 t_VFL; //vertical facing frontal left
  5 t_VFR; //vertical facing frontal right
  6 t_VBL; //vertical facing back left
  7 t_VBR; //vertical facing back right
  */

void setup() {
  unsigned char i = 0;
  
  //init pin_array as output, BASELINE_PIN output, A9 input
  for(i = 0; i < 8; i++) {
    pinMode(Pin_Array[i],OUTPUT);
  }
    pinMode(BASELINE_PIN, OUTPUT);
    pinMode(Button_Pin, INPUT);

  //init Serial display
  Serial.begin(9600);

  //init thruster array with pin values
  for (i = 0; i < 8; i++) {
    Thruster_Array[i].pin = Pin_Array[i];
  }
  //init BASELINE values
  BASELINE.pin = BASELINE_PIN;
  BASELINE.duty_cycle = BASELINE_VAL;
  
  //init tasks
  
  // triggered SM
  Tasks[0].state = 0;
  Tasks[0].period = 0; //triggered SM
  Tasks[0].elapsed_time = 0;
  Tasks[0].TickFct = &OutputPWM;
  Tasks[0].Active = true;//triggered SM, but run once on startup
  
  /* subscribes to jetson PWM commands and updates Thruster objects accordingly
  Tasks[1].state = 0;
  Tasks[1].period = 0; //triggered SM
  Tasks[1].elapsed_time = 0;
  Tasks[1].TickFct = &UpdateThrusters;
 */

  //init test/dummy values for PWMs
  Thruster_Array[0].duty_cycle = 250;
  Thruster_Array[1].duty_cycle = 220;
  Thruster_Array[2].duty_cycle = 200;
  Thruster_Array[3].duty_cycle = 170;
  Thruster_Array[4].duty_cycle = 125;
  Thruster_Array[5].duty_cycle = 100;
  Thruster_Array[6].duty_cycle = 75;
  Thruster_Array[7].duty_cycle = 0;

  
  //begin timer at the end of setup
  myTimer.begin(Scheduler, global_period); //begin scheduler

  //init ButtonPress as a triggered SM
  //attachInterrupt(digitalPinToInterrupt(Button_Pin), UpdateThrusters, RISING);
}

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
    else {
      Serial.println(int(i) + " is inactive");
    }
  }
}

void UpdateThrusters(){
  //increment all Thrusters
  for (unsigned int i = 0; i < 8; i++) {
    Thruster_Array[i].duty_cycle = ((Thruster_Array[i].duty_cycle + 25) % 255);
  }
  //insert OutputPWM into the array of tasks
  Tasks[0].Active = true;
}

//outputs PWM to pin array
//each pin's PWM can be changed separately
//NOTE::FIXME:: analogwrite continues to send steady PWM without needing to rewrite every tick,
//              so make this a trigger SM on change in value of duty_cycle
int OutputPWM(int state){
  unsigned int i;
  for (i = 0; i < 8; i++) {
   analogWrite(Thruster_Array[i].pin,Thruster_Array[i].duty_cycle);
  }
  
  //update BASELINE pwm (although right now it should never change)
  analogWrite(BASELINE.pin, BASELINE.duty_cycle);
  
  //make itself inactive to prevent it from continuously running 
  Tasks[0].Active = false;
  return 0;
}


void loop() {}
