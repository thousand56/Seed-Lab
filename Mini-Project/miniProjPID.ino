#include <Wire.h>

#define SLAVE_ADDRESS 0x04
int number = 0;
int state = 0;



//PID constants
double kp = 2;
double ki = 5;
double kd = 1;
 
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;
 
void setup(){
  pinMode(13, OUTPUT);
  Serial.begin(9600); // start serial for output
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);

  Serial.println("Ready!");  
  setPoint = 0;                          //set point at zero degrees
}    
 
void loop(){
        input = analogRead(A0);                //read from rotary encoder connected to A0
        output = computePID(input);
        delay(100);
        analogWrite(3, output);                //control the motor based on PID value
 
}

void receiveData(int byteCount){
  while(Wire.available()){
    setPoint = Wire.read();
    Serial.print("data received: ");
    Serial.println(number);
  }
}
 
double computePID(double inp){     
        currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        
        error = setPoint - inp;                                // determine error
        cumError += error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative
 
        double out = kp*error + ki*cumError + kd*rateError;                //PID output               
 
        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time
 
        return out;                                        //have function return the PID output
}
