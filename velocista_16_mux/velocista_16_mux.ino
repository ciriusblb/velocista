
#include "MUX74HC4067.h"

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;

MUX74HC4067 mux(2, A0, A1, A2, A3);

int a=0;
int b[16];
int sensor;

  #define pwmi      3   //~

  #define pwmd      11   //~

  int position=0;
  int salida_pwm=0;
  int velocidad=0;
  int derivativo=0;
  int proporcional=0;
  int error=0;
  int error_pasado;
  int integral=0;
  float kp=0;
  float kd=0;
  float ki=0;


  
void setup()
{
  Serial.begin(9600);
  pinMode(8, INPUT);
  
 pinMode(13,OUTPUT);

 myservo.attach(10);
 mux.signalPin(A4, INPUT, ANALOG);
 

}

void loop(){
if(digitalRead(8)== HIGH){
  for (pos = 0; pos <= 120; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(200);                       // waits 15ms for the servo to reach the position
  }
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
while(true){
velocidad=100;
kp=0.1;
kd=1;
ki=0;
 
pid(velocidad,kp,ki,kd);
}
}
}



void pid(int velocidad, float Kp, float Ki, float Kd)
{
  lectura();
  position = sensor;  //0 para linea 
                                                                //negra, 1 para linea blanca
  error = (position) - 720;   
  proporcional=error;
  integral=integral + error_pasado;  
  derivativo = (error - error_pasado);            //obteniedo el derivativo
  if (integral>255) integral=255;                              //limitamos la integral para no causar problemas
  if (integral<-255) integral=-255;
  salida_pwm =( proporcional * Kp ) + ( derivativo * Kd ) + (integral*Ki);
  
  if (  salida_pwm > velocidad )  salida_pwm = velocidad;       //limitamos la salida de pwm
  if ( salida_pwm < -velocidad )  salida_pwm = -velocidad;
  
  if (salida_pwm < 0)
 {
    motores(velocidad+salida_pwm, velocidad);
 }
 if (salida_pwm >0)
 {
     motores(velocidad, velocidad-salida_pwm);
 }

 error_pasado = error;  
}

void motores(int motor_izq, int motor_der){                                       

  if(motor_izq >= 0)
  { 
    analogWrite(pwmi,motor_izq); 
  }
  else
  {
    motor_izq = motor_izq*(-1); 
    analogWrite(pwmi,motor_izq);
  }
  if ( motor_der >= 0 ) //motor derecho
  {
    analogWrite(pwmd,motor_der);
  }
  else
  {
    motor_der= motor_der*(-1);
    analogWrite(pwmd,motor_der);
  }
}

void lectura(){
  int data;

  for (byte i = 0; i < 16; ++i)
  {
    // Reads from channel i. Returns a value from 0 to 1023
    data = mux.read(i);
   b[i]=(data*(i))/10;
   if((double)(data) * 100 / 1023 > 94) {a = a+1; b[i] = ((double)(data) * 100 / 1023)*i;}
   else if((double)(data) * 100 / 1023 <= 94) {a = a; b[i] = 0;}
    //Serial.print(data*(i+1));
    //Serial.print("  ");
  }
  sensor = (b[0]+b[1]+b[2]+b[3]+b[4]+b[5]+b[6]+b[7]+b[8]+b[9]+b[10]+b[11]+b[12]+b[13]+b[14]+b[15])/a + 1;
  Serial.println(sensor);
  a=0;
}

