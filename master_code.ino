/*
 Pontificia Universidad Catolica de Chile
 IEE2913 Diseño Eléctrico
 2016-2
 Alumnos:
 Rodrigo Cuzmar
 Louise Muller
 Cristian VIllalobos
 
 HC-SR04 Ping distance sensor:
 
 VCC to arduino 5v 
 GND to arduino GND
 Echo to Arduino pin 12
 Trig to Arduino pin 13
 
 Thanks for sort function to https://www.tutorialspoint.com/c_standard_library/c_function_qsort.htm
 */
 
#include <avr/interrupt.h>
#include <Servo.h>
#define echoPin 12 // Echo Pin
#define trigPin 13 // Trigger Pin
#define MOTOR1 11
#define MOTOR2 10

Servo myservo;
int pos;
int posFinal;
int Pos1 =180;
int Pos2 =0;
int distPos1;
int distPos2;
int minDist=400;
int muestras=4;
int errors=0;
int aux =0;
int mediana =1+ (muestras/2);
int values[] = { 0, 0, 0, 0, 0 };
long duration, distance, realDistance, oldDistance; // Duration used to calculate distance
//long pos = 50;

int set = 1500;

float promedio1;
float promedio2;

float Kp = 0.9;
float Ki = 0.06;
float Kd = 0.1;
float last_error = 0;
float Integral = 0;

float Kp1 = 0.9;
float Ki1 = 0.06;
float Kd1 = 0.1;
float last_error1 = 0;
float Integral1 = 0;

float PID_Controller(float IN, float SET, float dt, float last_error, float Integral);
float PID_Controller1(float IN, float SET, float dt, float last_error, float Integral);


 

 int pwm1 = 0;
 int pwm2 = 0;
 
 int die11 = 0;
 int die12 = 0;
 int die21 = 0;
 int die22 = 0;
 
 float vel11 = 0;
 float vel12 = 0;
 float vel21 = 0;
 float vel22 = 0;
 
 int CONTADOR = 0;
 
 
 
 volatile uint8_t portchistory = 0x00;     // default is high because the pull-up

float PID_Controller(float IN, float SET, float dt) {

  float OUT;
        float error;
        float Derivative;


  error = SET - IN;
  Integral = Integral + error*dt;
  Derivative = (error - last_error)/dt;
        last_error = error; 
  OUT =  (Kp*error + Ki*(Integral) + Kd*Derivative)*255/6000;  

  if (OUT > 255)
    OUT = 255;
  else if (OUT < 0)
    OUT = 0;

  return OUT;
}


float PID_Controller1(float IN, float SET, float dt) {

  float OUT;
        float error;
        float Derivative;

  error = SET - IN;
  Integral1 = Integral1 + error;
  Derivative = error - last_error1;
        last_error1 = error;

  OUT = (Kp1*error + Ki1*Integral*dt + Kd1*Derivative/dt)*255/6000;  

  if (OUT > 255)
    OUT = 255;
  else if (OUT < 0)
    OUT = 0;

  return OUT;
}

int cmpfunc (const void * a, const void * b)
{
   return ( *(int*)a - *(int*)b );
}


void setup() {
 //PID ports 
 pinMode(MOTOR1,OUTPUT);
 pinMode(MOTOR2,OUTPUT);  
 pinMode(6,OUTPUT);
 pinMode(5,OUTPUT);
 
 pinMode(A0,INPUT);
 pinMode(A1,INPUT);
 pinMode(A2,INPUT);
 pinMode(A3,INPUT);
 
 digitalWrite(6,HIGH);
 digitalWrite(5,LOW);
 digitalWrite(4,HIGH);
 digitalWrite(3,LOW);

 //puertos de sensado
 myservo.attach(9); 
 Serial.begin (9600);
 pinMode(trigPin, OUTPUT);
 pinMode(echoPin, INPUT);
 pinMode(8, OUTPUT);

 //////////////////////////////////////////////
 PCICR |= (1 << PCIE2);     // set PCIE0 to enable PCMSK0 scan
 PCMSK2 |= (1 << PCINT8) + (1 << PCINT9) + (1 << PCINT20) + (1 << PCINT21);   // set PCINT0 to trigger an interrupt on state change 
 /////////////////////////////////
 OCR2A = 65000;
 TIMSK2 |= _BV(OCIE2A);

}
//////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
void loop() {
  int m;
/* The following trigPin/echoPin cycle is used to determine the
 distance of the nearest object by bouncing soundwaves off of it. */ 
for(int j = 0; j <= 300; j+= 5)  // goes from 0 degrees to 180 degrees 
  {      // in steps of 1 degree 
     if (j<150) {
       pos = j + 25;
     }
     else pos = 325 - j;
     myservo.write(pos);               // tell servo to go to position in variable 'pos' 
     for(int n = 0; n <= muestras; n+= 1)
     {
     //ultrasonic
     digitalWrite(trigPin, LOW); 
     delayMicroseconds(2); 

     digitalWrite(trigPin, HIGH);
     delayMicroseconds(10); 
 
     digitalWrite(trigPin, LOW);
     duration = pulseIn(echoPin, HIGH);
     //
 
     //Calculate the distance (in cm) based on the speed of sound.
     distance = duration/58.2;
     values[n]=distance;

     delay(50);
     }
     qsort(values, 5, sizeof(int), cmpfunc);
     realDistance=values[mediana];
     Serial.println(realDistance);
     if (( realDistance < minDist )&&(pos>35)&&(pos<165)){
      minDist=realDistance;
      Pos1=pos;
     }
     distPos1=minDist+2;
     distPos2=minDist-2;
     if ( (realDistance <= distPos1) && (realDistance >= distPos2) ){
      if (pos<Pos1){
        Pos1=pos;
      }
      if (pos>Pos2){
        Pos2=pos;
      } 
     }
     //Serial.println(values[3]);
     //delay(50);
  }
  posFinal=(Pos1+Pos2)/2;
  myservo.write(posFinal);
  oldDistance=minDist;
Pos1 =180;
Pos2 =0;
minDist=400;
delay(500);
while ( true ){
       for(int n = 0; n <= muestras; n+= 1)
     {

     //ultrasonic
     digitalWrite(trigPin, LOW); 
     delayMicroseconds(2); 

     digitalWrite(trigPin, HIGH);
     delayMicroseconds(10); 
 
     digitalWrite(trigPin, LOW);
     duration = pulseIn(echoPin, HIGH);
     //
      
     //Calculate the distance (in cm) based on the speed of sound.
     distance = duration/58.2;
     values[n]=distance;

     delay(50);//delay malvado

     //aqui comienza codigo PID
       if (Serial.available()) {
          int inByte = Serial.read();
          Serial.println(inByte,DEC);
          if (inByte == 119){
            set = set + 50;
          }
          else if (inByte == 115){
            set = set - 50;
            if (set < 0){
              set=0;
            }
            
          }
          
        }
        
        
        promedio1 = (vel11 + vel12)/2;
        promedio2 = (vel21 + vel22)/2;
      
      /*
      Serial.print(vel11);
      Serial.print('\t');
      Serial.print('\t');
      Serial.print(vel12);
      
      Serial.print('\t');
      Serial.print('\t');
      */
      
      Serial.print(promedio1);
      
      Serial.print('\t');
      Serial.print('\t');
      Serial.print(set);
      
      Serial.print('\t');
      Serial.print('\t');
      Serial.print(pwm1);
      
      Serial.print('\t');
      Serial.print('\t');
      Serial.print(Integral);
      Serial.print('\t');
      Serial.print('\t');
      Serial.println(last_error);
      
      /*
      if (vel11 > 30){
        digitalWrite(13, HIGH);
      }
      else if (vel11 < 30){
        digitalWrite(13,LOW);
      }
      */
      
      pwm1 = PID_Controller(promedio1, set, 0.5);
      analogWrite(MOTOR1,pwm1);
      
      pwm2 = PID_Controller1(promedio2, set, 0.5);
      analogWrite(MOTOR2,pwm2);
     //fin codigo PID
     }
     
     qsort(values, 5, sizeof(int), cmpfunc);
     realDistance=values[mediana];
     if (aux<3){
        oldDistance=realDistance;
        set=500+10*oldDistance;
        aux=aux+1;
      }
      sei();
     Serial.println(realDistance);
     Serial.println(oldDistance);
     if((realDistance >= oldDistance+4) || (realDistance <= oldDistance-3)){
      Serial.println("Falle");
      errors=errors+1;
      if (errors==5){
        errors=0;
        aux=0;
        break;
      }
     }
     //aqui empezaba codigo del PID, pero era muy lento
     
     //hasta aqui llega while
}
}

SIGNAL (TIMER2_COMPA_vect)
{
    // action to be done every 1 sec
    if(CONTADOR == 500){
    vel11 = die11*5;
    vel12 = die12*5;
    die11 = 0;
    die12 = 0;
    
    
    vel21 = die21*5;
    vel22 = die22*5;
    die21 = 0;
    die22 = 0;
    
    
    CONTADOR = 0;
    //Serial.println(CONTADOR);
    }
    CONTADOR++;
    
}


ISR (PCINT2_vect)
{
    uint8_t changedbits;


    changedbits = PINC ^ portchistory;
    portchistory = PINC;

    
    if(changedbits & (1 << PINC0))
    {
        /* PCINT0 changed */
        die11 = die11 + 1;
        //Serial.println(die11);
    }
    
    if(changedbits & (1 << PINC1))
    {
        /* PCINT1 changed */
        die12 = die12 + 1;
    }

    if(changedbits & (1 << PINC2))
    {
        /* PCINT2 changed */
        die21 = die21 + 1;
    }
    
    if(changedbits & (1 << PINC3))
    {
        /* PCINT2 changed */
        die22 = die22 + 1;
    }

}


