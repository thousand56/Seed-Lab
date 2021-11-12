
// Input forward distance in feet
double forwardFeet = 0;
// Input turning distance in radians (left is positive)
double turningRad = 0;

int turningMotorEncoderCounts = 252;
int forwardMotorEncoderCounts = 514;


int turningMaxMotor = 80; //in analog write units lol
int turningMinMotor = 50; //in analog write units
int turningMinAngle = 2; //max allowable offset from desired angle in encoder counts
int turningMinOffset = 5; //a larger number increases the radius of rotation center in encoder counts

int forwardMaxMotor = 80; //in analog write units
int forwardMinMotor = 30; //in analog write units
int forwardMinDistance = 2; //max allawable offset from desired distance in encoder counts
int forwardMinOffset = 3; //a larger number increases the amount of "S" turning it does


#include <Wire.h>

#define SLAVE_ADDRESS 0x04
int number = 0;




int forwardEncoder;
int turningEncoder;

int piValue = 0;
const byte rightEncoderPinA = 3;
const int encoderCounts = 800;
int rightEncoderPinB = 5;
int rightEncoderPos = 0;
int rightEncoderPinALast = LOW;
int n = LOW;
volatile byte state = LOW;
//set encoder positions
int leftEncoderPinA = 2;
int leftEncoderPinB = 6;
int leftEncoderPos = 0;
int leftEncoderPinALast = LOW;
int p = LOW;


int rightErrorGlobal;
int leftErrorGlobal;

double rightkp = 0.43178;
double rightki = 0.086008;
double rightkd = 0;

unsigned long rightCurrentTime, rightPreviousTime;
double rightElapsedTime;
double rightError;
double rightLastError;
double rightInput, rightOutput;
double leftInput, leftOutput;
double rightSetPoint = 0;
double rightCumError, rightRateError;


double leftkp = 0.43178;
double leftki = 0.086008;
double leftkd = 0;

unsigned long leftCurrentTime, leftPreviousTime;
double leftElapsedTime;
double leftError;
double leftLastError;
double leftSetPoint = 0;
double leftCumError, leftRateError;

double rightSetPointOrigin;
double leftSetPointOrigin;

int turning = 0;
int moveForward = 0;



//int encoder0PinA = 3;
//int encoder0PinB = 5;
//int encoder1PinA = 4;
//int encoder1PinB = 6;
void setup() {

  
//forwardEncoder = forwardFeet * forwardMotorEncoderCounts;
//turningEncoder = turningRad * turningMotorEncoderCounts;

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

  pinMode(13, OUTPUT); //for pi coms
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  

  pinMode(rightEncoderPinA, INPUT);
  pinMode(rightEncoderPinB, INPUT);
  pinMode(leftEncoderPinA, INPUT);
  pinMode(leftEncoderPinB, INPUT);

  attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), rightEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), leftEncoderISR, CHANGE);


  pinMode(13, OUTPUT);
  Serial.begin(115200); // start serial for output


  Serial.println("Ready!");  
  

  rightEncoderPos = 0;
  leftEncoderPos = 0;
  rightOutput = 0;
  leftOutput = 0;
  

  turning = 0;
  rightSetPointOrigin = turningEncoder;
  leftSetPointOrigin = -turningEncoder;
  rightSetPoint = rightSetPointOrigin;
  leftSetPoint = leftSetPointOrigin;
  

 delay(2000);
}


void loop(){

    if(number == 250){
      digitalWrite(7, LOW);
      digitalWrite(8, LOW);
      analogWrite(9, 0);
      analogWrite(10, 0);
      exit(0);
    }

        if (turning == 1){
          rightInput = rightEncoderPos;  
          rightOutput = -rightComputePID(rightInput);
          if(rightOutput < 0){
            digitalWrite(7, LOW);
          }else{
            digitalWrite(7, HIGH);
          }
   
          if (rightOutput < 0){
            rightOutput = -rightOutput;
          }
          if (rightOutput > turningMaxMotor){
            rightOutput = turningMaxMotor;
          }
          if(rightOutput < turningMinMotor && rightOutput > 0){
            rightOutput = turningMinMotor;
          }
          analogWrite(9, rightOutput); 
          
  
          leftInput = leftEncoderPos;
          leftOutput = -leftComputePID(leftInput);
          if(leftOutput < 0){
            digitalWrite(8, HIGH);
          }else{
            digitalWrite(8, LOW);
          }
          
          if (leftOutput < 0){
            leftOutput = -leftOutput;
          }
          if (leftOutput > turningMaxMotor){
            leftOutput = turningMaxMotor;
          }
          if(leftOutput < turningMinMotor && leftOutput > 0){
            leftOutput = turningMinMotor;
          }
  
          analogWrite(10, leftOutput);
          
          rightErrorGlobal = rightSetPointOrigin - rightInput;
          leftErrorGlobal = leftSetPointOrigin - leftInput;

  
          if (-rightInput - leftInput > turningMinOffset || -rightInput - leftInput < -turningMinOffset){
             //right motor is bigger than left motor
             rightSetPoint = rightInput;
          }
          else if(-leftInput-rightInput > turningMinOffset || -leftInput-rightInput < -turningMinOffset){
            //left motor is bigger
            leftSetPoint = leftInput;
          }else if(rightErrorGlobal < turningMinAngle && leftErrorGlobal < turningMinAngle){
            rightOutput = 0;
            leftOutput = 0;
            rightEncoderPos = 0;
            leftEncoderPos = 1;
            turning = 0;
            rightSetPointOrigin = forwardEncoder;
            leftSetPointOrigin = forwardEncoder;
            rightSetPoint = rightSetPointOrigin;
            leftSetPoint = leftSetPointOrigin;
            
          }  else{
            rightSetPoint = rightSetPointOrigin;
            leftSetPoint = leftSetPointOrigin;
          }

         


//=================================================================================================================
        }else if (moveForward == 1){
          rightInput = rightEncoderPos;                //read from rotary encoder connected to A0
          //one PID Function
          rightOutput = -rightComputePID(rightInput);
          if(rightOutput < 0){
            digitalWrite(7, LOW);
          }else{
            digitalWrite(7, HIGH);
          }
   
          if (rightOutput < 0){
            rightOutput = -rightOutput;
          }
          if (rightOutput > forwardMaxMotor){
            rightOutput = forwardMaxMotor;
          }
          if(rightOutput < forwardMinMotor){
            rightOutput = forwardMinMotor;
          }
          analogWrite(9, rightOutput);                //control the motor based on PID value
          
  
          leftInput = leftEncoderPos;                //read from rotary encoder connected to A0
          //second PID function
          leftOutput = -leftComputePID(leftInput);
          if(leftOutput < 0){
            digitalWrite(8, HIGH);
          }else{
            digitalWrite(8, LOW);
          }
          
          if (leftOutput < 0){
            leftOutput = -leftOutput;
          }
          if (leftOutput > forwardMaxMotor){
            leftOutput = forwardMaxMotor;
          }
          if(leftOutput < forwardMinMotor){
            leftOutput = forwardMinMotor;
          }
  
          analogWrite(10, leftOutput);
          
            
  
          if (rightInput - leftInput > forwardMinOffset){
             //right motor is bigger than left motor
             rightSetPoint = rightInput;
          }
          else if(leftInput-rightInput > forwardMinOffset){
            //left motor is bigger
            leftSetPoint = leftInput;
          }else if(rightOutput < forwardMinDistance && leftOutput < forwardMinDistance){
            rightOutput = 0;
            leftOutput = 0;
            
            moveForward = 0;
           
           
          }else{
            rightSetPoint = rightSetPointOrigin;
            leftSetPoint = leftSetPointOrigin;
          }

         

        }

        if(turning == 0 && moveForward ==0){
          analogWrite(9, 0); 
          analogWrite(10, 0);             
        }

        if(turning == 1 && moveForward == 1){
          turning = 0;
          moveForward = 0;
        }


} //end of loop



void receiveData(int byteCount){
  while(Wire.available()){
    number = Wire.read();

    if(number < 100){
      rightOutput = 0;
      leftOutput = 0;
      rightEncoderPos = 0;
      leftEncoderPos = 0;
      turning = 0;      
      forwardFeet = number*0.0833;
      forwardEncoder = forwardFeet * forwardMotorEncoderCounts;
      rightSetPointOrigin = forwardEncoder;
      leftSetPointOrigin = forwardEncoder;
      rightSetPoint = rightSetPointOrigin;
      leftSetPoint = leftSetPointOrigin;
      moveForward = 1;
    }else if(number > 99 && number < 175){
        rightEncoderPos = 0;
        leftEncoderPos = 0;
        rightOutput = 0;
        leftOutput = 0;
     
      turningRad = (number - 100)*0.01745;
      turningEncoder = turningRad * turningMotorEncoderCounts;
      rightSetPointOrigin = turningEncoder;
      leftSetPointOrigin = -turningEncoder;
      rightSetPoint = rightSetPointOrigin;
      leftSetPoint = leftSetPointOrigin;
      turning = 1;
    }else if(number > 174){
        rightEncoderPos = 0;
        leftEncoderPos = 0;
        rightOutput = 0;
        leftOutput = 0;
 

      turningRad = (-(number - 175))*0.01745;
      turningEncoder = turningRad * turningMotorEncoderCounts;
      rightSetPointOrigin = turningEncoder;
      leftSetPointOrigin = -turningEncoder;
      rightSetPoint = rightSetPointOrigin;
      leftSetPoint = leftSetPointOrigin;
      turning = 1;
    }
  }
}


//PID calculations
double rightComputePID(double inp){     
        rightError = rightSetPoint - inp;                                // determine error
        float out = rightkp*rightError;       
        return out;                                      
}
double leftComputePID(double inp){     
        leftError = leftSetPoint - inp;                                // determine error        
        float out = leftkp*leftError;
        return out;                                      
}



void rightEncoderISR(){
    n = digitalRead(rightEncoderPinA);
    if ((rightEncoderPinALast == LOW) && (n == HIGH)) {
      if (digitalRead(rightEncoderPinB) == LOW) {
        rightEncoderPos--;
      } else {
        rightEncoderPos++;
      }
    }
  rightEncoderPinALast = n;
} //end of encoderISR

void leftEncoderISR(){
    p = digitalRead(leftEncoderPinA);
    if ((leftEncoderPinALast == LOW) && (p == HIGH)) {
      if (digitalRead(leftEncoderPinB) == LOW) {
        leftEncoderPos--;
      } else {
        leftEncoderPos++;
      }
    }
  leftEncoderPinALast = p;
} //end of encoderISR
