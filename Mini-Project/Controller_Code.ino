#include <Wire.h>

#define SLAVE_ADDRESS 0x04
int number = 0;
//int state = 0;

int piValue = 0;


const byte encoder0PinA = 3;
const int encoderCounts = 800;
int encoder0PinB = 5;
int encoder0Pos = 0;
int encoder0PinALast = LOW;
int n = LOW;

volatile byte state = LOW;


//PID constants
double kp = 0.43178;
double ki = 0.086008;
double kd = 0;

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;

void setup(){

  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  pinMode(7, OUTPUT);

  pinMode(8, OUTPUT);

  pinMode(9, OUTPUT);

  pinMode(10, OUTPUT);

  pinMode(12, INPUT);


  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoder0PinA), encoderISR, CHANGE);


  pinMode(13, OUTPUT);
  Serial.begin(115200); // start serial for output
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);

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
      




}

void receiveData(int byteCount){
  while(Wire.available()){
    piValue = Wire.read();
    Serial.print("data received: ");
    Serial.println(piValue);
    if(piValue == 1){
      setPoint = (piValue * 200);
    }
    else if (piValue == 2){
      setPoint = (piValue * 200);

    else if (piValue == 3){
      setPoint = (piValue * 200);
    }
    else if (piValue == 4){
      setPoint = 0;
    }
  }
}

double computePID(double inp){     
        currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation

        error = setPoint - inp;                                // determine error
        cumError += error * elapsedTime;                // compute integral
        //rateError = (error - lastError)/elapsedTime;   // compute derivative
        
        float out = kp*error;// + ki*cumError; //+ kd*rateError;                //PID output     
        
                 

        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time


       // out = out*(256/3200);

        //if(out >= 127){
          //out = 127;
        //}else if(out < -128){
          //out = -128;
        //}
        
        //Serial.println(out);
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
