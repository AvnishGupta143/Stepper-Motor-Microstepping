const int step_pin = 9 ;   //defining pulse or step pin   
const int dir_pin = 12 ;  //defining pulse or direction pin( HIGH - clockwise)
const int ena_pin = 2 ;  //defining pulse or enable pin....HIGH means driver is off and rotor is free

unsigned long curr_time;
unsigned long pre_time = 0;
float t1 ;
float t2 ;
int option ; 

long Actual_time_fwd;
long Actual_time_back;
long step_per_revolution ; 
float dist_in_single_step ;          //hardcoded value  ( in  mm) - This is the input which should be specified by the user
float distance_target = 450;          //hardcoded value  ( in  mm ) - This the target distance to move
long steps_to_target;
float s ;                             // take speed input (in mm/s)
double time;
long step_interval_micro;

long feedback_pos = 0;
long feedback_neg = 0;

volatile int flag = 1;

void(* resetFunc) (void) = 0; //declare reset function @ address 0

void Stop_Start_Motor()
{
  if (Serial.available()>0)
  {
    volatile char c = Serial.read();
    if(c =='a')
    {
      flag = 1;   // stop the motor
    }
    if (c =='b')
    {
      flag = 0;   // start the motor
    } 
    if (c =='c')
    {
      flag = 2;  // reset arduino 
    } 
  } 
}

void write_int_on_serial(long num)
{
  String Str = String(num,DEC);
  Str = String(Str + "\n");
  char* buf_Str = (char*) malloc(sizeof(char)*Str.length()+1);
  Str.toCharArray(buf_Str,Str.length()+1);
  Serial.write(buf_Str);
  delay(5);
  free(buf_Str);
}

void setup() 
{ 
  Serial.begin(57600);
  
  pinMode(step_pin,OUTPUT);
  pinMode(ena_pin,OUTPUT);
  pinMode(dir_pin,OUTPUT);
  
  digitalWrite(step_pin,LOW);
  digitalWrite(ena_pin,HIGH);
  digitalWrite(dir_pin,HIGH);
  
  // Reading the speed in mm/s
  while(!Serial.available());
  while(Serial.available()!=0)
  {
    s = Serial.parseFloat();
  }
  // ask for stepping mode to choose and feed the value for no. of steps and distance in one step accordingly
  delay(300);
  while(!Serial.available());
  option = Serial.parseInt();
  
  switch (option)
  {
    case 0:
      step_per_revolution = 200;
      dist_in_single_step = 0.02;
      break;
    case 1:
      step_per_revolution = 400;
      dist_in_single_step = 0.175;
      break;
    case 2:
      step_per_revolution = 800;
      dist_in_single_step = 0.02;
      break;
    case 3:
      step_per_revolution = 1600;
      dist_in_single_step = 0.02;
      break;
    case 4:
      step_per_revolution = 3200;
      dist_in_single_step = 0.01;
      break;
    case 5:
      step_per_revolution = 6400;
      dist_in_single_step = 0.02;
      break;
    case 6:
      step_per_revolution = 10000;
      dist_in_single_step = 0.002;
      break;
    case 7:
      step_per_revolution = 12800;
      dist_in_single_step = 0.001;
      break;
    case 8:
      step_per_revolution = 16000;
      dist_in_single_step = 0.02;
      break;  
    case 9:
      step_per_revolution = 20000;
      dist_in_single_step = 0.02;
      break;
    default :
      step_per_revolution = 200;
      dist_in_single_step = 0.02;
      break;  
  }
  
  flag = 1;
  steps_to_target = distance_target / dist_in_single_step ;
  time = (distance_target / s) ;                              
  step_interval_micro = (time * 1000000 / steps_to_target) - 21 - 6;//21 is substracted as a correction factor and 6 for the Motor_Start_Stop() runtime
  time = time * 100 ;// multiply by 100 so that decimal places are preserved
  
  write_int_on_serial(time);
  write_int_on_serial(step_per_revolution);
  write_int_on_serial(steps_to_target);
  write_int_on_serial(step_interval_micro);
  
  while(!Serial.available());
  digitalWrite(ena_pin,LOW);
  
}

void one_step_fwd()
{
  digitalWrite(step_pin,HIGH);  
  digitalWrite (step_pin,LOW);
  delayMicroseconds(step_interval_micro);
  //Stop_Start_Motor();
}

void one_step_back()
{
  digitalWrite(step_pin,HIGH);
  digitalWrite(step_pin,LOW);  
  delayMicroseconds(step_interval_micro);
  //Stop_Start_Motor();
}

void loop()
{
  if (flag == 0)
  {
    digitalWrite(ena_pin, LOW);
    digitalWrite(dir_pin,HIGH);  
    t1 = micros();
    for(int i=0;i<steps_to_target;i++)
    {
      curr_time = micros();
      Stop_Start_Motor(); // This function takes 4 microseconds to execute
      if ((curr_time - pre_time >= step_interval_micro || i==0) && flag == 0)
      {
         one_step_fwd();
         pre_time = curr_time;
         feedback_pos = feedback_pos + 1;
      }
    }
    t2 = micros();
    Actual_time_fwd = ((t2-t1)/1000000)*100; // multiply by 100 so that decimal places are preserved    
    write_int_on_serial(Actual_time_fwd);
    
    delay(2000);
    feedback_neg = 0;
  
    digitalWrite(dir_pin,LOW);
    t1 = micros();
    for(int i=0;i<steps_to_target;i++)
    {
      curr_time = micros();
      Stop_Start_Motor();
      if ((curr_time - pre_time >= step_interval_micro || i==0) && flag == 0)
      { 
         one_step_back();
         pre_time = curr_time;
         feedback_neg = feedback_neg - 1;
      }
    }
    t2 = micros();
    Actual_time_back = ((t2-t1)/1000000)*100; // multiply by 100 so that decimal places are preserved
    write_int_on_serial(Actual_time_back);
  
    delay(2000);    
    feedback_pos = 0;
    flag = 1 ;
  }
  else if (flag == 2)   
  {
    digitalWrite(ena_pin,HIGH);
    resetFunc();
  }
  else if(flag == 1)
  {
    digitalWrite(ena_pin,HIGH);  
    Stop_Start_Motor();
  }
}
