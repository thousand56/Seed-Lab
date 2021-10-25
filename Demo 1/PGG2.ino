
int number = 0;
int piValue = 0;
const byte encoder0PinA = 3;
const int encoderCounts = 800;
int encoder0PinB = 5;
int encoder0Pos = 0;
int encoder0PinALast = LOW;
int n = LOW;
volatile byte state = LOW;
//set encoder positions
int encoder1PinA = 2;
int encoder1PinB = 6;
int encoder1Pos = 0;
int encoder1PinALast = LOW;
int p = LOW;



//PID constants
double kp = 0.43178;
double ki = 0.086008;
double kd = 0;

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output;
double input1, output1;
double setPoint = 1*2038;
double cumError, rateError;
#include <Encoder.h>

//int encoder0PinA = 3;
//int encoder0PinB = 5;
//int encoder1PinA = 4;
//int encoder1PinB = 6;
void setup() {
//  Encoder rightEnc(LEFT_A, LEFT_B);
//  Encoder leftEnc(RIGHT_A, RIGHT_B);
  Serial.begin(115200);
  // put your setup code here, to run once:
  
  //Pin used to enable
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  pinMode(7, OUTPUT);

  pinMode(8, OUTPUT);

  pinMode(9, OUTPUT);

  pinMode(10, OUTPUT);

  pinMode(11, OUTPUT);

  pinMode(12, OUTPUT);


  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);
  pinMode(encoder1PinA, INPUT);
  pinMode(encoder1PinB, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoder0PinA), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), encoder1ISR, CHANGE);


  pinMode(13, OUTPUT);
  Serial.begin(115200); // start serial for output


  Serial.println("Ready!");  
  setPoint = 0;                          //set point at zero degrees



}


void loop(){
        input = encoder0Pos;                //read from rotary encoder connected to A0
        output = computePID(input);
        if(output < 0){
          digitalWrite(8, HIGH);
          digitalWrite(7, LOW);
        }else{
          digitalWrite(8, LOW);
          digitalWrite(7, HIGH);
        }
      analogWrite(9, -output);                //control the motor based on PID value
        


      Serial.print(setPoint);
      Serial.print(" ");
      Serial.print(input);
      Serial.print(" ");
      Serial.println(output);
      //  analogWrite(9,127);
        input1 = encoder1Pos;                //read from rotary encoder connected to A0
        output1 = computePID(input1);
        if(output1 < 0){
          digitalWrite(11, HIGH);
          digitalWrite(12, LOW);
        }else{
          digitalWrite(11, LOW);
          digitalWrite(12, HIGH);
        }
        analogWrite(10, -output1);
      




}

//PID calculations
double computePID(double inp){     
        currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation

        error = setPoint - inp;                                // determine error
        cumError += error * elapsedTime;                // compute integral
        //rateError = (error - lastError)/elapsedTime;   // compute derivative
        
        float out = kp*error;// + ki*cumError; //+ kd*rateError;                //PID output     
        
                 

        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time

        return out;                                        //have function return the PID output
}

void encoderISR(){
    n = digitalRead(encoder0PinA);
    if ((encoder0PinALast == LOW) && (n == HIGH)) {
      if (digitalRead(encoder0PinB) == LOW) {
        encoder0Pos--;
      } else {
        encoder0Pos++;
      }
      //Serial.println(encoder0Pos);
    }
  encoder0PinALast = n;
} //end of encoderISR
void encoder1ISR(){
    p = digitalRead(encoder1PinA);
    if ((encoder1PinALast == LOW) && (p == HIGH)) {
      if (digitalRead(encoder1PinB) == LOW) {
        encoder1Pos--;
      } else {
        encoder1Pos++;
      }
      //Serial.println(encoder0Pos);
    }
  encoder1PinALast = 0;
} //end of encoderISR
