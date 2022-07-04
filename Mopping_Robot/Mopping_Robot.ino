#define MLAC 9
#define MLC 10
#define MRAC 6
#define MRC 5
#define ENCL 3
#define ENCR 2
#define RELAY 4
#define BUZZER A2
#define LED 11
#define TL 8
#define TR 7
#define TRG 12
#define BUTTON A3
#define BATT A6
#define CHRG A7
//MOTOR VARIABLES
int sp = 100, numberofsteps = 20, lsp = sp, rsp = sp*1.26;
bool ml=1, mr=1, rinmotion =0, linmotion = 0;

// Pulse count from encoder
volatile long encValL = 0, encValR = 0, setValL = 0, setValR = 0, prevall=0, prevalr=0;

// Counters for milliseconds during interval
unsigned long t1=0, t2=0, t3=0, t4 = 0, t5=0, t6=0, t7=0, tdelay=20; 
unsigned long setint1=27, setint2=27, lint=setint2+1, rint=setint2+1, timerl =0, timerr = 0;
//PARAMETRIC VARIABLES
float degree = 0, x=0, y = 0, tiredist= 15, tiredia = 7;

//VARIABLES
bool tl = 0, tr = 0, sw = 0, docked = 0;
float lastrotationdeg = 90;

/*
  :Version 1.0
  :Author: Dragos Calin
  :Email: dragos@intorobotics.com
  :License: BSD
  :Date: 06/04/2020
*/
 
 
/*define the sensor's pins*/
uint8_t trigPin = A0;
uint8_t echoPin = A1;
 
unsigned long timerStart = 0;
int TIMER_TRIGGER_HIGH = 10;
int TIMER_LOW_HIGH = 2;
int disttrg=0, thold =20;

float timeDuration, distance=300;
 
/*The states of an ultrasonic sensor*/
enum SensorStates {
  TRIG_LOW,
  TRIG_HIGH,
  ECHO_HIGH
};
 
SensorStates _sensorState = TRIG_LOW;
 
void startTimer() {
  timerStart = millis();
}
 
bool isTimerReady(int mSec) {
  return (millis() - timerStart) < mSec;
}

void ledfade(int delt = 1,bool fadein = 1, bool fadeout = 1)
{
  int brightness = 0;
  if(fadein)
  {
    for(brightness=0 ;brightness<255;brightness++)
   {
    analogWrite(LED, brightness);
    delay(delt);
    
   }
  }

  if(fadeout)
  {
    brightness=256;
    while(brightness!=0)
    {
      brightness--;
      analogWrite(LED, brightness);
      delay(delt);
    }
  }
}


void buzz(int freq=2000, int delt0=100, int numberofbeeps=1, int delt1= 50)
{
  int i = 0;
  while(i<numberofbeeps)
  {
    tone(BUZZER, freq);
    delay(delt0);
    noTone(BUZZER);
    if(numberofbeeps>1)
    {
      delay(delt1);
    }
    i++;
  }
  
}

void updateEncoderL()
{
  t2 = millis();
  if(t2-t1>tdelay)
  {
    lint= t2 - t1;
    timerl = t2;
    encValL++;
    t1=t2;
    if(!linmotion)
    {
      return;
    }
    if(lint>setint2 && lsp<255)
  {
    lsp++;
  }
  if(lint<setint1 && lsp>0)
  {
    lsp--;
  }

  if(ml)
  {
    analogWrite(MLC, lsp);
  }
  else
  {
    analogWrite(MLAC, lsp);
    
  }
  
  }
  
}

void updateEncoderR()
{
  t4 = millis();
  if(t4-t3>tdelay)
  {
    rint = t4 - t3;
    timerr = t4;
    encValR++;
    t3=t4;
    if(!rinmotion)
    {
      return;
    }
    if(rint>setint2 && rsp<255)
  {
    rsp++;
  }
  if(rint<setint1 && rsp>0)
  {
    rsp--;
  }

  if(mr)
  {
    analogWrite(MRC, rsp);
  }
  else
  {
    analogWrite(MRAC, rsp);
    
  }
  }
  
}



void motorl(int dir=0)
{
  analogWrite(MLC, 0);
  analogWrite(MLAC, 0);
  linmotion=1;
  if(dir == 0)
  {
    ml = 1;
    analogWrite(MLC, lsp);
    
  }

  else
  {
    ml = 0;
    analogWrite(MLAC, lsp);
    
  }
}

void motorr(int dir=0)
{
  analogWrite(MRC, 0);
  analogWrite(MRAC, 0);
  rinmotion = 1;
  if(dir == 0)
  {
    mr = 1;
    analogWrite(MRC, rsp);
    
  }

  else
  {
    mr = 0;
    analogWrite(MRAC, rsp);
    
  }
}


void stopl()
{
  if(ml == 0)
  {
    analogWrite(MLC, 0);
    analogWrite(MLAC, sp);
    delay(45);
    analogWrite(MLAC, 0);
  }

  else
  {
    analogWrite(MLAC, 0);
    analogWrite(MLC, sp);
    delay(45);
    analogWrite(MLC, 0);
  }
  linmotion = 0;
  
}


void stopr()
{
  if(mr == 0)
  {
    analogWrite(MRC, 0);
    analogWrite(MRAC, sp);
    delay(45);
    analogWrite(MRAC, 0);
  }

  else
  {
    analogWrite(MRAC, 0);
    analogWrite(MRC, sp);
    delay(45);
    analogWrite(MRC, 0);
  }
  rinmotion = 0;
  
}

void setspeed(int x=0)
{  
  unsigned long timer=millis();
  if((linmotion  && timer-timerl>1000) || (rinmotion && timer-timerr>1000))
  {
    stopl();
    stopr();
    motorl(1);
    motorr(1);
    delay(100);
    stopl();
    stopr();
    rotate(lastrotationdeg);
    motorl();
    motorr();
  }

  if(millis()-t7>15000)
  {
    motorl(1);
    motorr(1);
    delay(200);
    stopl();
    stopr();
    delay(1000);
    rotate(180);
    motorl();
    motorr();
    t7=millis();
  }
  
}


bool checksensor()
{
  bool outcome = 0;
  getDist();
  if(disttrg>0)
  {
    outcome = 1;
  }
  if(digitalRead(TL) == LOW)
  {
    tl = 1;
    outcome=1;
  }
  else
  {
    tl = 0;
  }

  if(digitalRead(TR) == LOW)
  {
    tr = 1;
    outcome=1;
  }
  else
  {
    tr = 0;
  }

  if(digitalRead(BUTTON) == LOW)
  {
    sw = 1;
  }
  else
  {
    sw = 0;
  }
  return outcome;
}

void turnoff()
{
  stopl();
  stopr();
  buzz();
  ledfade(1,1,0);
  digitalWrite(LED, HIGH);
  t5 = millis();
  while(millis()-t5 < 500ul)
  {
    if(digitalRead(BUTTON) == LOW)
    {
      buzz(3000, 300);
      ledfade(1,0,1);
      digitalWrite(RELAY, LOW);
      while(1)
      {
        Serial.print(analogRead(BATT));
        Serial.print("\t");
        Serial.println(analogRead(CHRG));
      }
    }
  }
  
  while(digitalRead(BUTTON))
  {
    delay(1);
    if(millis()-t5>60000)
    {
      ledfade(1,0,1);
      digitalWrite(RELAY, LOW);
      
    }
  }
  buzz();
  ledfade(3,0,1);
  motorl();
  motorr();
}

void rotate(float deg)
{
  lastrotationdeg = deg;
  long rotationsteps = (long)deg*tiredist*numberofsteps/(360*tiredia);
  Serial.println(rotationsteps);
  
  if(rotationsteps == 0)
  {
    return;
  }

  encValL = 0;
  encValR = 0;
  setValL = abs(rotationsteps);
  setValR = abs(rotationsteps);
  int digit = 0;
  disttrg = 0;

  if(rotationsteps<0)
  {
    ml=0;
    mr=1;
    analogWrite(MLC, lsp+20);
    analogWrite(MRAC, rsp+20);
    
  }
  else
  {
    ml=1;
    mr=0;
    analogWrite(MLAC, lsp+20);
    analogWrite(MRC, rsp+20);
    
  }

  timerl = millis();
  timerr = timerl;
  while(digit<2)
  {
    if(encValL>=setValL)
    {
      stopl();
      digit++;
    }

    if(encValR>=setValR)
    {
      stopr();
      digit++;
       
    }

    if(digitalRead(BUTTON)==LOW)
    {
      digit = 2;
      turnoff();
    }
    t6 = millis();
    if(t6-timerl>3000ul && t6-timerr>3000ul)
    {
      stopl();
      stopr();
      buzz(2500, 250, 2);
      digit = 2;
      if(lsp<235)
      {
        lsp += 20;
      }
      if(rsp<235)
      {
        rsp += 20;
      }
      else
      {
        digitalWrite(RELAY, LOW);
      }
       
    }
    
    //setspeed();
  
  }
  
}

void getDist()
{
  /*Switch between the ultrasonic sensor states*/
  switch (_sensorState) {
    /* Start with LOW pulse to ensure a clean HIGH pulse*/
    case TRIG_LOW: {
        digitalWrite(trigPin, LOW);
        startTimer();
        if (isTimerReady(TIMER_LOW_HIGH)) {
          _sensorState = TRIG_HIGH;
        }
      } break;
      
    /*Triggered a HIGH pulse of 10 microseconds*/
    case TRIG_HIGH: {
        digitalWrite(trigPin, HIGH);
        startTimer();
        if (isTimerReady(TIMER_TRIGGER_HIGH)) {
          _sensorState = ECHO_HIGH;
        }
      } break;
 
    /*Measures the time that ping took to return to the receiver.*/
    case ECHO_HIGH: {
        digitalWrite(trigPin, LOW);
        timeDuration = pulseIn(echoPin, HIGH);
        /*
           distance = time * speed of sound
           speed of sound is 340 m/s => 0.034 cm/us
        */
        distance = timeDuration*0.017;

        if(distance<thold)
        {
          disttrg++;
          
        }
        else
        {
          disttrg=0;
          
        }
        _sensorState = TRIG_LOW;
      } break;
      
  }//end switch
  
}

void moveforward(float cm)
{
  long steps = (long)((numberofsteps/(3.14*tiredia))*cm);
  Serial.println(steps);
  int digit = 0;
  if(steps<0)
  {
    motorl(1);
    motorr(1);
  }
  else
  {
    motorr();
    motorl();
  }
  encValL = 0;
  encValR = 0;
  
  while(digit<2)
  {
    if(encValL>=abs(steps) && linmotion==1)
    {
      stopl();
      stopr();
      digit=2;
    }
    
  }
  
}

void setprog1()
{
  rotate(-lastrotationdeg);
  getDist();
  if(distance>20)
  {
    moveforward(20);
    
  }
  else
  {
    if(distance>10)
    {
      moveforward(distance);
    }
    
  }
  rotate(lastrotationdeg);
}

void setalg1()
{
  if(disttrg>0)
  {
    rotate(lastrotationdeg);
    
  }
  else
  {
    if(tl)
    {
      rotate(-90); 
    }
    else
    {
      rotate(90);
    }
  }
}

void chargingcheck()
{
  if(analogRead(CHRG)>300)
   {
    delay(500);
    if(analogRead(CHRG)<600)
    {
      buzz(1500, 300, 2);
      
    }

    else
    {
      buzz(3600, 100, 3);
    }
    docked = 1;
    
   }
  
}



void setup() {
  
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, HIGH);
  
  pinMode(ENCL, INPUT_PULLUP);
  pinMode(ENCR, INPUT_PULLUP);
  pinMode(TL, INPUT_PULLUP);
  pinMode(TR, INPUT_PULLUP);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(MLAC, OUTPUT);
  pinMode(MLC, OUTPUT);
  pinMode(MRC, OUTPUT);
  pinMode(MRAC, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(RELAY, HIGH);


  // Attach interrupt 
  attachInterrupt(digitalPinToInterrupt(ENCL), updateEncoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCR), updateEncoderR, RISING);
  
   Serial.begin(9600);
   Serial.println("HELLO");
   buzz();
   chargingcheck();
   ledfade(1,1,0);
   while(digitalRead(BUTTON))
   {
    if(millis()>60000)
    {
      digitalWrite(RELAY, LOW);
    }
    
   }
   ledfade(3,0,1);
   if(docked)
   {
    moveforward(-20);
    rotate(180);
    
   }
   motorl();
   motorr();
   timerl = millis();
   timerr = timerl;
   
}


void loop() 
{
  if(checksensor())
  {
    stopr();
    stopl();
    setalg1();
    t7=millis();
    motorl();
    motorr();
  }
  setspeed();
  
  if(millis()>1800000)
  {
    buzz(3000, 500);
    digitalWrite(RELAY, LOW);
    
  }
  Serial.println(analogRead(BATT));

  if(sw)
  {
    turnoff();
  }
}
