// tests driving capability driving forward, driving backward, turning

#include <TinyGPS++.h>
#include <SoftwareSerial.h>

//define pins
#define backEncoder 3
#define frontEncoder 2
#define HCRXPin 4
#define HCTXPin 5
#define ATpin 6 // used to switch to AT mode
#define gpsRXPin 7
#define gpsTXPin 8
int frontMotor1=9;
int frontMotor2=10;
int backMotor1=11;
int backMotor2=12;
#define LEDpin 13

// define baud rates
#define GPSBaud 9600
#define HC12Baud 1200
#define monitorBaud 1200

// misc global
#define ppr 120 // pulses per rotation
#define radius 0.05715 //radius of wheel in meters
#define circumference 2*PI*radius //circumference of wheel in meters

// define global variables
double distance_to_goal, delta_dir, after_lat, after_lng, curr_dir;
double straight_lat, straight_lng, course1, course2;
double goal_dir, init_lat, init_lng, distance_covered, x, y, h;
double Goal_Lat = 40.246204, Goal_Lng = -111.646780;
String s,sa,sb;
bool writer;
int    polyCorners  = 4; // how many corners the polygon has
float  polyX[] = {38.14615, 38.14635, 38.14557, 38.14541 };  //latitudinal coordinates of corners
float  polyY[] = {-76.42668, -76.42617, -76.42608, -76.42661}; //longitudinal coordinates of corners
int ledPin = 13;  // LED connected to digital pin 13
int val = 0;      // variable to store the read value
double frequency = 0; //frequency of encoder pulses
double period = 0; // period of encoder pulses
double rpm = 0; // rotations per minute
int ms_measured = 500;
double transitions = 0;
int prev = 0; // previous voltage of pulse (high or low)
long init_time = millis();
double rotations = 0;
double distance = 0;

// define states
enum States {in_air, left, right, straight, wait, navigate, triangulate};
enum Orientations {upside, downside};
Orientations orientation = upside;
States state = in_air; 

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(gpsTXPin, gpsRXPin);
SoftwareSerial HC12(HCTXPin, HCRXPin); // HC-12 TX Pin, HC-12 RX Pin

void setup() {
  // setup serial communications
  Serial.begin(monitorBaud);             // Serial port to computer
  HC12.begin(HC12Baud);               // Serial port to HC12
  ss.begin(GPSBaud); //Serial port to gps
  
  pinMode(ATpin, OUTPUT);
  pinMode(frontMotor1, OUTPUT);
  pinMode(frontMotor2, OUTPUT);
  pinMode(backMotor1, OUTPUT);
  pinMode(backMotor2, OUTPUT);
  pinMode(frontEncoder, INPUT);
  pinMode(backEncoder, INPUT);
  pinMode(LEDpin, OUTPUT);
  
  digitalWrite(ATpin, HIGH); // For command mode set to low
  digitalWrite(frontMotor1, LOW);
  digitalWrite(frontMotor2, LOW);
  digitalWrite(backMotor1, LOW);
  digitalWrite(backMotor2, LOW);
  digitalWrite(LEDpin, HIGH);
  
  bool writer = false; 

  state = in_air;
  distance_to_goal = 100;

  attachInterrupt(digitalPinToInterrupt(frontEncoder), encoder_counter, RISING); 
}

void loop() {
//  drive_straight(5);
//  drive_left(5);
  drive_straight(1);
  smart_stop();
  drive_right(3);
  smart_stop();
  drive_straight(1);
  smart_stop();
  drive_left(5);
//  delay(2000);
//
//  stop_fun();
//  delay(2000);
//  
//  switch_motors;
//  delay(500);
//  drive_straight(10);
//  delay(2000);
//  drive_left(10);
//  delay(2000);
//  drive_straight(10);
//  delay(2000);
//  drive_right(10);
//  delay(2000);

  stop_fun(0);
  while(1);
//  delay(2000);
//
//  switch_motors();
//  delay(500);
}

void switch_motors(){
  int a = frontMotor1;
  int b = frontMotor2;
  int c = backMotor1;
  int d = backMotor2;
  frontMotor1 = b;
  frontMotor2 = a;
  backMotor1 = d;
  backMotor2 = c;
  Serial.print("Switching Motors!");
  switch (orientation){
    case upside:
      orientation = downside;
      Serial.print("Switching downside!");
      break;
    case downside:
      orientation = upside;
      Serial.print("Switching downside!");
      break;
  }
}

void smart_stop(){
  digitalWrite(frontMotor1, LOW);
  digitalWrite(frontMotor2, LOW);
  digitalWrite(backMotor1, LOW);
  digitalWrite(backMotor2, LOW);
  while (transitions != 0){
    transitions = 0;
    delay(100);
  }
}

void stop_fun(double dangle)
{
  digitalWrite(frontMotor1, LOW);
  digitalWrite(frontMotor2, LOW);
  digitalWrite(backMotor1, LOW);
  digitalWrite(backMotor2, LOW);
  Serial.print("Stopping!");
  if (dangle != 0){
    delay(dangle);
  }
}

void drive_left(double dangle)
{
  if (orientation == upside){
    digitalWrite(frontMotor2, LOW); 
    digitalWrite(backMotor2, LOW);
    Serial.print("same");
    Serial.print(orientation);
    analogWrite(frontMotor1, 1);
    analogWrite(backMotor1, 255);   
  }
  else{
    digitalWrite(frontMotor2, LOW); 
    digitalWrite(backMotor2, LOW);
    digitalWrite(backMotor1, LOW);
    Serial.print("something different");
    analogWrite(frontMotor1, 255);
    analogWrite(backMotor1, 1);
  }
  
  //check angle
  Serial.print("Turning Left: ");
  Serial.print(dangle);
  distance = 0;
  transitions = 0;
  while(distance < dangle){
    rotations = transitions/ppr;
    distance = rotations * circumference;
    Serial.println(distance); // you need this line otherwise transitions doesn't update!!!!
  }
}

void drive_right(double dangle)
{
  Serial.print("Turning Right: ");
  drive_left(dangle);
  Serial.print(dangle);
  // delay(3000);
  distance = 0;
  transitions = 0;
  while(distance < dangle/2){
    rotations = transitions/ppr;
    distance = rotations * circumference;
    Serial.println(distance); // you need this line otherwise transitions doesn't update!!!!
  }
  smart_stop();
  switch_motors();
  drive_straight(2);
}

void drive_straight(double dist)
{
  digitalWrite(frontMotor2, LOW); 
  digitalWrite(backMotor2, LOW);
  
  digitalWrite(frontMotor1, HIGH);
  digitalWrite(backMotor1, HIGH);
    
  //check distance
  Serial.print("Driving Straight: ");
  Serial.print(dist);
  // delay(3000);
  distance = 0;
  transitions = 0;
  while(distance < dist){
    rotations = transitions/ppr;
    distance = rotations * circumference;
    Serial.println(distance); // you need this line otherwise transitions doesn't update!!!!
  }
}

void encoder_counter() {
  transitions += 1;
}

//notes:

// PWM Must be one of 3, 5, 6, 9, 10, or 11

//Encoder

//  if ((millis() - init_time) % 1000 == 0){
//    Serial.print(distance);
//    Serial.println(" meters");
//  }
//  frequency = transitions/2.0;
//  period = 1.0/frequency;
//  rpm = 60000/ms_measured/period/ppr;

//Antenna
      // To set up antenna connect ground to set.
      // type AT into monitor and it will reply with OK
      // to change Baud Rate: AT+Bxxxx ex. AT+B9600
      // to change Communication Channel: AT+Cxxx ex. AT+C006

      // when you use the serial monitor to send custom messages it does a wierd thing where only the first three characters of a new line make it to the computer

//  //transmission code
//  while (HC12.available())  // If HC-12 has data
//  {       
//    Serial.write(HC12.read());      // Send the data to Serial monitor
//  }
//  while (Serial.available()) // If Serial monitor has data
//  {      
//    HC12.write(Serial.read());      // Send that data to HC-12
//  }

//GPS
      // Serial.println(gps.speed.mph()); // Speed in miles per hour (double)
      // Serial.println(gps.speed.mps()); // Speed in meters per second (double)
      // Serial.println(gps.speed.kmph()); // Speed in kilometers per hour (double)
      // Serial.println(gps.course.value()); // Raw course in 100ths of a degree (i32)
      // Serial.println(gps.course.deg()); // Course in degrees (double)
      // Serial.println(gps.satellites.value()); // Number of satellites in use (u32)
      // Serial.println(gps.hdop.value()); // Horizontal Dim. of Precision (100ths-i32)      


////////////////////////////////////////////////////////////////////////////////////
//Extra Functions
////////////////////////////////////////////////////////////////////////////////////

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

int sign(double x)
{
  int signOfX = (x > 0) - (x < 0);
  return signOfX;
}

//  Globals which should be set before calling this function:
//
//  int    polyCorners  =  how many corners the polygon has
//  float  polyX[]      =  horizontal coordinates of corners
//  float  polyY[]      =  vertical coordinates of corners
//  float  x, y         =  point to be tested
//
//  (Globals are used in this example for purposes of speed.  Change as
//  desired.)
//
//  The function will return true if the point x,y is inside the polygon, or
//  NO if it is not.  If the point is exactly on the edge of the polygon,
//  then the function may return true or false.
//
//  Note that division by zero is avoided because the division is protected
//  by the "if" clause which surrounds it.

// from: http://alienryderflex.com/polygon/

bool pointInPolygon() {

  int   i, j=polyCorners-1 ;
  bool  oddNodes=false     ;
  
  for (i=0; i<polyCorners; i++) {
    if ((polyY[i]< y && polyY[j]>=y
    ||   polyY[j]< y && polyY[i]>=y)
    &&  (polyX[i]<=x || polyX[j]<=x)) {
      oddNodes^=(polyX[i]+(y-polyY[i])/(polyY[j]-polyY[i])*(polyX[j]-polyX[i])<x); }
    j=i; }

  return oddNodes; }  
