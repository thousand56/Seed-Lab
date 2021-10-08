const int samplingTime = 10; //10ms
const unsigned long collectionTime = 2000; //2seconds
const byte encoder0PinA = 3;
const int encoderCounts = 3200;

unsigned long currentTime;
unsigned long loopTime;

int encoder0PinB = 4;
int encoder0Pos = 0;
int encoder0PinALast = LOW;
int n = LOW;
int encoderLast;
int encoderDif;
float velocity;

volatile byte state = LOW;


void setup() {
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);
  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(encoder0PinA), encoderISR, CHANGE);
} //end of setup

void loop() {
  Serial.print(currentTime);
  Serial.print("  ");
  Serial.println(velocity);

  encoderDif = encoder0Pos - encoderLast;         //UNIT: encoderCounts
  velocity = (encoderDif*628.2)/encoderCounts;    //UNIT: radians / second
  encoderLast = encoder0Pos;                      //UNIT: encoder Position
 
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
