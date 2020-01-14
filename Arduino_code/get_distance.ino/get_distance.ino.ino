

int ledPin = 13;  // LED connected to digital pin 13
int inPin = 6;    // pushbutton connected to digital pin 6
int val = 0;      // variable to store the read value
float frequency = 0; //frequency of encoder pulses
float period = 0; // period of encoder pulses
float ppr = 120; // pulses per rotation
float rpm = 0; // rotations per minute
int ms_measured = 500;
long transitions = 0;
int prev = 0; // previous voltage of pulse (high or low)
long init_time = millis();
long rotations = 0;
float radius = 0.05715; //radius of wheel in meters
float circumference = 2*PI*radius; //circumference of wheel in meters
float distance = 0;


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);  // sets the digital pin 13 as output
  pinMode(inPin, INPUT);    // sets the digital pin 7 as input
  Serial.begin(115200);
}

void loop() {
  val = digitalRead(inPin);   // read the input pin
  if (val != prev){ //if the value of pulse is different than it used to be, count transition
      transitions += 1;
      }
  prev = val;
  frequency = transitions/2.0;
  period = 1.0/frequency;
  rpm = 60000/ms_measured/period/ppr;
  rotations = transitions/ppr;
  distance = rotations * circumference;
  if ((millis() - init_time) % 1000 == 0){
    Serial.print(distance);
    Serial.println(" meters");
  }
  
}
