//Parameters
const int pinIR = A0;
const int pinLED1 = 2;
const int pinLED2 = 3;

//Variables
int IRval  = 0;

void setup() {
  //Init Serial USB
  Serial.begin(9600);

  pinMode(pinIR, INPUT);
  pinMode(pinLED1, OUTPUT);
  pinMode(pinLED2, OUTPUT);
  digitalWrite(pinLED1, LOW);
  digitalWrite(pinLED2, LOW);
}

void loop() {
  IR_sharp();
}

void IR_sharp( ) {
  ////Read distance sensor
  IRval = analogRead(pinIR);
  delay(10);
  if (IRval > 220) {
    car_detected();
  }
}

void car_detected (){
  digitalWrite(pinLED1, HIGH);
  digitalWrite(pinLED2, HIGH);
  delay(1000);
  Serial.print('a');
  delay(3000);
  digitalWrite(pinLED1, LOW);
  digitalWrite(pinLED2, LOW);
  delay(20000);
}




