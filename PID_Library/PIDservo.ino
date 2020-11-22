//This Arduino code is developed by I Lab
//Hit the SUBSCRIBE button for following our tutorial on arduino.
//You Tube Channel ID: https://www.youtube.com/c/IngenieroLab?sub_confirmation=1.
//Follow our facebook page: https://www.facebook.com/ingenierorobotico

//BTS7960 motor driver sketch 
#include "PinChangeInt.h"
#include <PID_v1.h>                                   // Thanks to Brett Beauregard for his nice PID library
#define encodPinA1      2                             // Quadrature encoder A pin
#define encodPinB1      3                             // Quadrature encoder B pin
int bientro = A0;
int R_IS = 6;
int R_EN = 9;
int R_PWM = 8;
int L_IS = 7;
int L_EN = 4;
int L_PWM = 5;
double kp = 5 , ki = 1 , kd = 0.01 ,input = 0, output = 0, setpoint = 0;   // modify kp, ki and kd for optimal performance
long temp;
volatile long encoderPos = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);  // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT'
void setup() {
  // put your setup code here, to run once:
   Serial.begin(9600);
 pinMode(R_IS, OUTPUT);
 pinMode(R_EN, OUTPUT);
 pinMode(R_PWM, OUTPUT);
 pinMode(L_IS, OUTPUT);
 pinMode(L_EN, OUTPUT);
 pinMode(L_PWM, OUTPUT);
 pinMode(A0, input);
 digitalWrite(R_IS, LOW);
 digitalWrite(L_IS, LOW);
 digitalWrite(R_EN, HIGH);
 digitalWrite(L_EN, HIGH);
  pinMode(encodPinA1, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(encodPinB1, INPUT_PULLUP);                  // quadrature encoder input B
  attachInterrupt(0, encoder, FALLING);               // update encoder position
  TCCR1B = TCCR1B & 0b11111000 | 1;                   // set 31KHz PWM to prevent motor noise
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);
}


void loop() {
  temp == analogRead(bientro);                              // increment position target with potentiometer value (speed), potmeter connected to A0
  if (temp < 0) {                                     // in case of overflow
    encoderPos = 0;
    temp = 0;
  }
  else
  setpoint = temp /500;                              // modify division to fit motor and encoder characteristics
  input = encoderPos ;                                // data from encoder
  myPID.Compute();                                    // calculate new output
  pwmOut(output);      
  }
 void pwmOut(int out) {                               
  if (out > 0) {
    analogWrite(R_PWM, out);                             // drive motor CW        
    analogWrite(L_PWM, 0);                                              
  }
  else {
    analogWrite(R_PWM, 0);                         
    analogWrite(L_PWM, abs(out));                        // drive motor CCW
  }
    Serial.println(input);
    Serial.println(",");
    Serial.println(temp);
     Serial.println();
}
void encoder()  {                                     // pulse and direction, direct port reading to save cycles
  if (PINB & 0b00000001)    encoderPos++;             // if (digitalRead(encodPinB1)==HIGH)  count ++;
  else                      encoderPos--;             // if (digitalRead(encodPinB1)==LOW)   count --;
}
