//attachInterrupt(digitalPinToInterrupt(A1), Work, RISING);

//Thruster struct
typedef struct Thruster {
  char PWM = 4; //some value 0-8
  char cnt = 0;
  char LED = LOW;
}thruster;

//Task struct
typedef struct Task {
  int state = 0;
  unsigned long period = 0;
  unsigned long elapsed_time = 0;
  int (*TickFct)(int);
} task;

//GLOBAL
int global_period = 125000;
IntervalTimer myTimer;

thruster Thruster_Array[8];
int Pin_Array[8] = {A0,A1,A2,A3,A4,A5,A6,A7}; 

const unsigned char  tasks_size = 1;
task Tasks[1];
 
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
  
  //init A0-A7 as output
  for(i = 0; i < 8; i++) {
    pinMode(Pin_Array[i],OUTPUT);
  }

  //init Serial display
  Serial.begin(9600);

  //init tasks
  Tasks[0].state = 0;
  Tasks[0].period = 125000;
  Tasks[0].elapsed_time = 0;
  Tasks[0].TickFct = &OutputPWM;

  /* not yet
  Tasks[1].state = 0;
  Tasks[1].period = 0; //triggered SM
  Tasks[1].elapsed_time = 0;
  Tasks[1].TickFct = &UpdateThrusters;
  not yet */

  //begin timer at the end of setup
  myTimer.begin(Scheduler, global_period); //begin scheduler
}

//runs every global period
//just a scheduler
void Scheduler(){
  unsigned char i;
  for (i = 0; i < tasks_size; i++){
    if (Tasks[i].TickFct != NULL) {
      if (Tasks[i].elapsed_time >= Tasks[i].period) {
        Tasks[i].state = Tasks[i].TickFct(Tasks[i].state);
        Tasks[i].elapsed_time = 0;        
      }
      Tasks[i].elapsed_time += global_period;
    }
  }
}

//not yet
int UpdateThrusters(int state){
  Serial.println("2");
  return 2;
}

//outputs HIGH/LOW to A0-A7 (PWM)
//each pin's PWM can be changed separately
//due to visual output contraints, PWM works in fractions of 8 (1/8 uptime to 8/8 uptime)
int OutputPWM(int state){
  unsigned int i;
  for (i = 0; i < 8; i++) {
    if (Thruster_Array[i].cnt < Thruster_Array[i].PWM){
      digitalWrite(Pin_Array[i], HIGH);
    }
    else {
      digitalWrite(Pin_Array[i],LOW);
    }
    Thruster_Array[i].cnt++;
  }
  return 0;
}


void loop() {
//  idle; //FIXME:: add idle/sleep library
}
