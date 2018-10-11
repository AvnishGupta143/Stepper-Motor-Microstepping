const int step_pin = 9 ;   //defining pulse or step pin   
const int dir_pin = 12 ;  //defining pulse or direction pin( HIGH - clockwise)
const int ena_pin = 2 ;  //defining pulse or enable pin....HIGH means driver is off and rotor is free
unsigned long curr_time;
unsigned long pre_time = 0;
unsigned int step_per_revolution = 200;
float dist_in_single_step = 0.02;    //hardcoded value  ( in  mm)
float distance_target = 50;        //hardcoded value  ( in  mm
unsigned int steps_to_target = distance_target / dist_in_single_step ;
float s = 1.5;                  // take speed input (in mm/s)
float time = distance_target / s ;
unsigned long step_interval_micro = time * 1000000 / steps_to_target;

void setup() 
{ 
  Serial.begin(9600);
  pinMode(step_pin,OUTPUT);
  pinMode(ena_pin,OUTPUT);
  pinMode(dir_pin,OUTPUT);
  digitalWrite(step_pin,LOW);
  digitalWrite(ena_pin,HIGH);
  digitalWrite(dir_pin,HIGH);
  Serial.println(steps_to_target);
  Serial.println(step_interval_micro);
}

void one_step_fwd()
{
  digitalWrite(dir_pin,HIGH);
  digitalWrite(step_pin,HIGH);  
  digitalWrite (step_pin,LOW);
  delayMicroseconds(step_interval_micro);
}

void one_step_back()
{
  digitalWrite(dir_pin,LOW);
  digitalWrite(step_pin,HIGH);
  digitalWrite(step_pin,LOW);
  delayMicroseconds(step_interval_micro);
}

void loop()
{
  digitalWrite(ena_pin,LOW);
  Serial.println("fwd");
  for(int i=0;i<steps_to_target;i++)
  {
    curr_time = micros(); 
    if (curr_time - pre_time >= step_interval_micro || i==0)
    { 
      one_step_fwd();
      pre_time = curr_time;
    }
  }
  digitalWrite(ena_pin,HIGH);
  delay(2000);
  digitalWrite(ena_pin,LOW);
  Serial.println("back");
  for(int i=0;i<steps_to_target;i++)
  {
    curr_time = micros(); 
    if (curr_time - pre_time >= step_interval_micro || i==0)
    { 
      one_step_back();
      pre_time = curr_time;
    }
  }
  delay(2000);
}
