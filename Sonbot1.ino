//12:35 - 16:30
#include <Servo.h>
#include <L298N.h>

// ------- L298N ---------
static byte ENA = 6;       static byte ENB = 9;     // for regulate speed it has to be pwm pin
static byte IN1 = 4;       static byte IN3 = 8;
static byte IN2 = 3;       static byte IN4 = 7;

L298N LeftMotor(ENA, IN1, IN2);
L298N RightMotor(ENB, IN3, IN4);

static int rotate90degrees_time = 500;     // time for 90 degree rotation 

void Forward();
void Backward();
void Stop();
void TurnLeft(int ms);                    // time in ms for rotate 90 degrees
void TurnRight(int ms);                   // change for your project

// ------ HC-SR04 --------
static byte TRIG = 11;
static byte ECHO = 12;

static long robot_length = 45.00;       // change for your project

long GetDistance();

// ------ Servo ----------
Servo myservo;

void ServoLeft();
void ServoMiddle();
void ServoRight();


void setup() 
{
   myservo.attach(10);
   LeftMotor.GPIO_Config();
   RightMotor.GPIO_Config();
   LeftMotor.EnableMotor();
   RightMotor.EnableMotor();
   //RightMotor.SetSpeed(150);
   //LeftMotor.SetSpeed(150);
   pinMode(TRIG, OUTPUT);
   pinMode(ECHO, INPUT);
   
      //Serial.begin(9600);
}

byte state = 0;
void loop() 
{
   long distance_cm = GetDistance();          //Serial.println(distance_cm);

   switch(state)
   {
      case 0:   
        ServoMiddle();
        if(distance_cm < robot_length) state = 1;
        Forward();
        break;

      case 1:  
        ServoMiddle();
        Stop();
        state = 2;
        break;

      case 2:
        ServoMiddle();
        if(TryRight()) state = 3;
        else state = 4;
        break; 

      case 3:
        ServoMiddle();
        //TurnRight(rotate90degrees_time);
        TurnLeft(rotate90degrees_time);       // custom 
        state = 0;
        break;

      case 4:
        ServoMiddle();
        if(TryLeft()) state = 5;
        else state = 6;
        break;

      case 5:
        ServoMiddle();
        TurnRight(rotate90degrees_time);    // custom for bad connection dc motors
        //TurnLeft(rotate90degrees_time);
        state = 0;
        break;

     case 6:
        ServoMiddle();
        TurnRight(rotate90degrees_time);
        TurnRight(rotate90degrees_time);
        state = 0;
        break;
   }
                                          //Serial.println(state); delay(1000);
}

// --------- FUNCTIONS DECLARATIONS -----------
void Forward()
{
  LeftMotor.Forward();
  RightMotor.Forward();
}

void Backward()
{
  LeftMotor.Backward();
  RightMotor.Backward();
}

void Stop()
{
  LeftMotor.Brake();
  RightMotor.Brake();
}

void TurnLeft(int ms)
{
  LeftMotor.Backward();
  RightMotor.Forward();
  delay(ms);
  LeftMotor.Brake();
  RightMotor.Brake();
}

void TurnRight(int ms)
{
  LeftMotor.Forward();
  RightMotor.Backward();
  delay(ms);
  LeftMotor.Brake();
  RightMotor.Brake();
}

long GetDistance()
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long time = pulseIn(ECHO, HIGH);    //Serial.println(time);

  return (time/58.00) - 0.5;
}

bool TryRight()
{
  ServoRight();
  if(GetDistance() > robot_length ) return true;
  else return false;
}

bool TryLeft()
{
  ServoLeft();
  if(GetDistance() > robot_length) return true;
  else return false;
}

void ServoLeft()
{
  myservo.write(180);
  delay(250);
}

void ServoMiddle()
{
  myservo.write(90);
  delay(250);
}

void ServoRight()
{
  myservo.write(0);
  delay(250);
}

