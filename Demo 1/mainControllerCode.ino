
// Input forward distance in feet
double forwardFeet = 3;
// Input turning distance in radians (left is positive)
double turningRad = 1.57;

// 1 ft forward is 510 encoder counts
// 90 degree turn is 500 encoder counts


int forwardEncoder;
int turningEncoder;

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
//double kp = 0.01;
double kp = 0.43178;
double ki = 0.086008;
double kd = 0;

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output;
double input1, output1;
double setPoint = 0;
double cumError, rateError;

///double kp1 = 0.2;

double kp1 = 0.43178;
double ki1 = 0.086008;
double kd1 = 0;

unsigned long currentTime1, previousTime1;
double elapsedTime1;
double error1;
double lastError1;
double setPoint1 = 0;
double cumError1, rateError1;

double setPointOrigin;
double setPoint1Origin;

int turning = 0;
int moveForward = 0;



//int encoder0PinA = 3;
//int encoder0PinB = 5;
//int encoder1PinA = 4;
//int encoder1PinB = 6;
void setup() {

  
forwardEncoder = forwardFeet * 510;
turningEncoder = turningRad * 295;

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
  

  encoder0Pos = 0;
  encoder1Pos = 0;
  output = 0;
  output1 = 0;
  

  turning = 1;
  setPointOrigin = turningEncoder;
  setPoint1Origin = -turningEncoder;
  setPoint = setPointOrigin;
  setPoint1 = setPoint1Origin;
  

 delay(2000);
}


void loop(){


        if (turning == 1){
          input = encoder0Pos;                //read from rotary encoder connected to A0
           //one PID Function
          output = -computePID(input);
          if(output < 0){
            digitalWrite(7, LOW);
          }else{
            digitalWrite(7, HIGH);
          }
   
          if (output < 0){
            output = -output;
          }
          if (output > 127){
            output = 127;
          }
          analogWrite(9, output);                //control the motor based on PID value
          
  
          input1 = encoder1Pos;                //read from rotary encoder connected to A0
          //second PID function
          output1 = -compute1PID(input1);
          if(output1 < 0){
            digitalWrite(8, HIGH);
          }else{
            digitalWrite(8, LOW);
          }
          
          if (output1 < 0){
            output1 = -output1;
          }
          if (output1 > 127){
            output1 = 127;
          }
  
          analogWrite(10, output1);
          
            
  
          if (-input - input1 > 10){
             //right motor is bigger than left motor
             setPoint = input;
          }
          else if(-input1-input > 10){
            //left motor is bigger
            setPoint1 = input1;
          } else{
            setPoint = setPointOrigin;
            setPoint1 = setPoint1Origin;
          }

          if(output < 30 && output1 < 30){
            output = 0;
            output1 = 0;
            encoder0Pos = 0;
            encoder1Pos = 1;
            turning = 0;
            moveForward = 1;
            setPointOrigin = forwardEncoder;
            setPoint1Origin = forwardEncoder;
            setPoint = setPointOrigin;
            setPoint1 = setPoint1Origin;
            

         
            
          }


//=================================================================================================================
        }else if (moveForward == 1){
          input = encoder0Pos;                //read from rotary encoder connected to A0
          //one PID Function
          output = -computePID(input);
          if(output < 0){
            digitalWrite(7, LOW);
          }else{
            digitalWrite(7, HIGH);
          }
   
          if (output < 0){
            output = -output;
          }
          if (output > 127){
            output = 127;
          }
          analogWrite(9, output);                //control the motor based on PID value
          
  
          input1 = encoder1Pos;                //read from rotary encoder connected to A0
          //second PID function
          output1 = -compute1PID(input1);
          if(output1 < 0){
            digitalWrite(8, HIGH);
          }else{
            digitalWrite(8, LOW);
          }
          
          if (output1 < 0){
            output1 = -output1;
          }
          if (output1 > 127){
            output1 = 127;
          }
  
          analogWrite(10, output1);
          
            
  
          if (input - input1 > 10){
             //right motor is bigger than left motor
             setPoint = input;
          }
          else if(input1-input > 10){
            //left motor is bigger
            setPoint1 = input1;
          } else{
            setPoint = setPointOrigin;
            setPoint1 = setPoint1Origin;
          }

          if(output < 30 && output1 < 30){
            output = 0;
            output1 = 0;
            
            moveForward = 0;
            delay(1000);
           
          }

        }

        if(turning == 0 && moveForward ==0){
          analogWrite(9, 0); 
          analogWrite(10, 0);             
        }


} //end of loop





//PID calculations
double computePID(double inp){     
        error = setPoint - inp;                                // determine error
        float out = kp*error;       
        return out;                                      
}
double compute1PID(double inp1){     
        error1 = setPoint1 - inp1;                                // determine error        
        float out1 = kp1*error1;
        return out1;                                      
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
  encoder1PinALast = p;
} //end of encoderISR
