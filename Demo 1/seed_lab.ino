
// 1 ft forward is 510 encoder counts


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
  setPoint = 0; //set point at zero degrees
  setPoint1 = 0;

  encoder0Pos = 0;
  encoder1Pos = 0;
  
  
  setPoint = 5000;
  setPoint1 = 5000;
 
}


void loop(){
// created by the man the myth the legend, steven
//  if(millis() > 5000){
//    setPoint = 50;
//    setPoint1 = 50;
//  }else if(millis() > 6000){
//    setPoint = 100;
//    setPoint1 = 100;
//  }else if(millis() > 7000){
//    setPoint = 150;
//    setPoint1 = 150;
//  }else if(millis() > 8000){
//    setPoint = 200;
//    setPoint1 = 200;
//  }else if(millis() > 9000){
//    setPoint = 250;
//    setPoint1 = 250;
//  }else if(millis() > 10000){
//    setPoint = 300;
//    setPoint1 = 300;
//  }else if(millis() > 11000){
//    setPoint = 350;
//    setPoint1 = 350;
//  }else if(millis() > 12000){
//    setPoint = 400;
//    setPoint1 = 400;
//  }
//
//
//
//



  
        input = encoder0Pos;                //read from rotary encoder connected to A0
        output = -computePID(input);
        if(output < 0){
          //digitalWrite(8, HIGH);
          digitalWrite(7, LOW);
        }else{
          //digitalWrite(8, LOW);
          digitalWrite(7, HIGH);
        }
      analogWrite(9, -output);                //control the motor based on PID value
        


//      Serial.print(setPoint);
//      Serial.print(" ");
//      Serial.print(input);
//      Serial.print(" ");
//      Serial.println(output);
      //  analogWrite(9,127);
        input1 = encoder1Pos;                //read from rotary encoder connected to A0
        output1 = -compute1PID(input1);
        if(output1 < 0){
          digitalWrite(8, HIGH);
          //digitalWrite(7, LOW);
        }else{
          digitalWrite(8, LOW);
          //digitalWrite(7, HIGH);
        }

        output1 = output1*0.5;
        analogWrite(10, output1);

        
      Serial.print(setPoint);
      Serial.print(" ");
      Serial.print(input1);
      Serial.print(" ");
      Serial.println(output1);
      




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

double compute1PID(double inp1){     
        currentTime1 = millis();                //get current time
        elapsedTime1 = (double)(currentTime1 - previousTime1);        //compute time elapsed from previous computation

        error1 = setPoint1 - inp1;                                // determine error
        cumError1 += error1 * elapsedTime1;                // compute integral
        //rateError = (error - lastError)/elapsedTime;   // compute derivative
        
        float out1 = kp1*error1;// + ki*cumError; //+ kd*rateError;                //PID output     
        
                 

        lastError1 = error1;                                //remember current error
        previousTime1 = currentTime1;                        //remember current time

        return out1;                                        //have function return the PID output
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
