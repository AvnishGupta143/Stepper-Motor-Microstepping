const int step_pin = 9 ;   //defining pulse or step pin   
const int dir_pin = 12 ;  //defining pulse or direction pin( HIGH - clockwise)
const int ena_pin = 2 ;  //defining pulse or enable pin....HIGH means driver is off and rotor is free

unsigned long curr_time;
unsigned long pre_time = 0;
float t1 ;
float t2 ; 

unsigned int step_per_revolution = 200; 
float dist_in_single_step = 0.001;    //hardcoded value  ( in  mm) - This is the input which should be specified by the user
float distance_target = 50;          //hardcoded value  ( in  mm ) - This the target distance to move
unsigned int steps_to_target = distance_target / dist_in_single_step ;
float s = 5;                       // take speed input (in mm/s)
float time = distance_target / s ;
unsigned long step_interval_micro = (time * 1000000 / steps_to_target) - 21 ; 

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
  Serial.print("Theoritical time taken: ");
  Serial.println(time);
  // ask for stepping mode to choose and feed the value for no. of steps and distance in one step accordingly  
}

void one_step_fwd()
{
  digitalWrite(step_pin,HIGH);  
  digitalWrite (step_pin,LOW);
  delayMicroseconds(step_interval_micro);
}

void one_step_back()
{
  digitalWrite(step_pin,HIGH);
  digitalWrite(step_pin,LOW);
  delayMicroseconds(step_interval_micro);
}

void loop()
{
  digitalWrite(ena_pin,LOW);
  digitalWrite(dir_pin,HIGH);
  //Serial.println("fwd");
  t1 = micros();
  for(int i=0;i<steps_to_target;i++)
  {
   curr_time = micros(); 
   if (curr_time - pre_time >= step_interval_micro || i==0)
     { 
      one_step_fwd();
      pre_time = curr_time;
     }
  }
  t2 = micros();
  
  Serial.print("Actual time taken to move fwd in seconds:");
  Serial.println((t2-t1)/1000000);
  digitalWrite(ena_pin,HIGH);
  
  delay(2000);
  
  digitalWrite(dir_pin,LOW);
  digitalWrite(ena_pin,LOW);
  t1 = micros();
  //Serial.println("back");
  for(int i=0;i<steps_to_target;i++)
  {
    curr_time = micros(); 
    if (curr_time - pre_time >= step_interval_micro || i==0)
    { 
      one_step_back();
      pre_time = curr_time;
    }
  }
  t2 = micros();
  
  Serial.print("Actual time taken to move back in seconds:");
  Serial.println((t2-t1)/1000000);
  
  delay(2000);
  
}
