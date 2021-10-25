//Pinout
//encoderLeftPinA = 2;
//encoderLeftPinB = 6;
//encoderRightPinA = 3;
//encoderRightPinB = 5;


const int samplingTime = 10; //10ms
const unsigned long collectionTime = 10000; //2seconds
const byte encoderLeftPinA = 2;
const byte encoderRightPinA = 3;
const int encoderCounts = 3200;

unsigned long currentTime;
unsigned long loopTime;


int encoderLeftPinB = 6;
int encoderLeftPos = 0;
int encoderLeftPinALast = LOW;
int n = LOW;
int encoderLeftLast;
int encoderLeftDif;
float velocityLeft;

int encoderRightPinB = 5;
int encoderRightPos = 0;
int encoderRightPinALast = LOW;
int m = LOW;
int encoderRightLast;
int encoderRightDif;
float velocityRight;

volatile byte state = LOW;


void setup() {
  pinMode(encoderLeftPinA, INPUT);
  pinMode(encoderLeftPinB, INPUT);
  pinMode(encoderRightPinA, INPUT);
  pinMode(encoderRightPinB, INPUT);
  
  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(encoderLeftPinA), encoderLeftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRightPinA), encoderRightISR, CHANGE);

} //end of setup

void loop() {
  Serial.print(currentTime);
  Serial.print("  ");
  Serial.print(velocityLeft);
  Serial.print("  ");
  Serial.println(velocityRight);

  encoderLeftDif = encoderLeftPos - encoderLeftLast;         //UNIT: encoderCounts
  velocityLeft = (encoderLeftDif*628.2)/encoderCounts;    //UNIT: radians / second
  encoderLeftLast = encoderLeftPos;                      //UNIT: encoder Position

  encoderRightDif = encoderRightPos - encoderRightLast;         //UNIT: encoderCounts
  velocityRight = (encoderRightDif*628.2)/encoderCounts;    //UNIT: radians / second
  encoderRightLast = encoderRightPos;                      //UNIT: encoder Position
 

 
  if(currentTime >= collectionTime){
    Serial.println("End of Data");
    exit(0);
  }
  if(millis() > currentTime + samplingTime){
    Serial.println("Error");
  }
  while(millis() < currentTime + samplingTime){
  }
  currentTime = currentTime + samplingTime;
} //end of main

void encoderLeftISR(){
    n = digitalRead(encoderLeftPinA);
    if ((encoderLeftPinALast == LOW) && (n == HIGH)) {
      if (digitalRead(encoderLeftPinB) == LOW) {
        encoderLeftPos--;
      } else {
        encoderLeftPos++;
      }
    }
  encoderLeftPinALast = n;
} //end of encoderLeftISR

void encoderRightISR(){
    m = digitalRead(encoderRightPinA);
    if ((encoderRightPinALast == LOW) && (m == HIGH)) {
      if (digitalRead(encoderRightPinB) == LOW) {
        encoderRightPos--;
      } else {
        encoderRightPos++;
      }
    }
  encoderRightPinALast = m;
} //end of encoderRightISR
