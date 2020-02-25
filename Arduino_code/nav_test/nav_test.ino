// tests driving capability driving forward, driving backward, turning

#include <TinyGPS++.h>
#include <SoftwareSerial.h>

//define pins
#define backEncoder 3
#define frontEncoder 2
#define HCRXPin 4
#define HCTXPin 5
#define ATpin 6 // used to switch to AT modet
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
double Goal_Lat = 40.246187, Goal_Lng = -111.646674; //Manhole cover in parking lot
//double Goal_Lat = 40.246204, Goal_Lng = -111.646780; // sidewalk cover
//double Goal_Lat = 40.24642, Goal_Lng = -111.64638; // manhole in street by parking lot
String s,sa,sb,sc;
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
int count;

struct gps_lat_lng {
   double latitude;
   double longitude;
   double crs;
};

gps_lat_lng gps_state;

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

void send_state(String typer = "none", double x = 0.0, double y=0.0, double z=0.0);

void setup()
{
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
  count = 0;
  
  Serial.println(F("HelloWorld"));
  Serial.println();

  attachInterrupt(digitalPinToInterrupt(frontEncoder), encoder_counter, RISING); 
}

void loop()
{
  ss.listen();
  if (ss.available() > 0){
    if (gps.encode(ss.read()))
    {
      Serial.print(F("it is going into nav_code"));
      nav_code();
    }
  }
  if (distance_to_goal < 3)
  {
    winner();
  }
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
  }
}

void check_for_messages(){
  HC12.listen();
  while (HC12.available()) {        // Check if HC-12 has data
    byte c = HC12.read();
    s += char(c);
    writer = true;
  }
  ss.listen();
}

void winner()
{
  stop_fun(0);
  HC12.listen();
  while (1){
    Serial.print("You Did It");
    HC12.write("You did it!;");
    send_state("gps_coordinates",init_lat,init_lng,distance_to_goal);
  }
}

void nav_code()
{
  init_lat = gps.location.lat();
  init_lng = gps.location.lng();
  distance_to_goal =
  TinyGPSPlus::distanceBetween(
    gps.location.lat(),
    gps.location.lng(),
    Goal_Lat,
    Goal_Lng);
    //drive forward
    // delay(1000);
    after_lat = gps.location.lat();
    after_lng = gps.location.lng();
    curr_dir = gps.course.deg(); // TinyGPSPlus::courseTo(init_lat,init_lng,after_lat,after_lng);
    distance_to_goal = TinyGPSPlus::distanceBetween(after_lat,after_lng,Goal_Lat,Goal_Lng);
    goal_dir = TinyGPSPlus::courseTo(after_lat,after_lng,Goal_Lat,Goal_Lng);
    delta_dir = curr_dir-goal_dir; // turn left if positive, right if negative;
    if(abs(delta_dir) > 180) // test so the car always turns the least amount
    {
      delta_dir = (360-abs(delta_dir))*sign(delta_dir)*-1; //
    }
    Serial.print(distance_to_goal);
    Serial.print("\t");
    Serial.print(delta_dir);
    Serial.print("\t");
    Serial.print(after_lng);
    Serial.println();
    HC12.listen();
    send_state("directions_to_goal",distance_to_goal,delta_dir);
    check_for_messages();
    ss.listen();
    smartDelay(0);
    //change direction
    count = count +1;
    Serial.println(count);
    Serial.println();
    Serial.print("valid GPS? :: ");
    Serial.print("\t");
    Serial.print(gps.location.isValid());
    if (count > 10 && gps.location.isValid()) //slow down loop and check if gps location is valid                                                
    {
        if(delta_dir > 0)
        {
          drive_left(delta_dir, goal_dir);
        }
        else
        {
          if (delta_dir > -90){
            drive_right(delta_dir, goal_dir, curr_dir);
          }
          else{
            switch_motors();
            drive_left(delta_dir+90,goal_dir);
          }
        }
        //drive in direction
        drive_straight(distance_to_goal);
        count = 0;
    }
    //setup for next loop
    init_lat = after_lat;
    init_lng = after_lng;
    //printInt(distance_meters, gps.location.isValid(), 9);
    //Serial.println();
    //printFloat(dangle, gps.location.isValid(), 7, 2);
    //Serial.println();
}

void send_state(String typer, double x, double y, double z)
{
  // send two doubles seperated by a comma
  HC12.listen();
  sa = String(x,10);
  sb = String(y,10);
  sc = String(z,10);
  s = typer+','+sa+','+sb+','+sc;
  HC12.print(s);
  HC12.write(";");
  ss.listen();
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
  send_state("Switching_motors");
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

void stop_fun(double time_delay)
{
  digitalWrite(frontMotor1, LOW);
  digitalWrite(frontMotor2, LOW);
  digitalWrite(backMotor1, LOW);
  digitalWrite(backMotor2, LOW);
  Serial.print("Stopping!");
  if (time_delay != 0){
    delay(time_delay);
  }
}

void drive_left(double dangle, double goal_theta)
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
  dangle = dangle*5/90; //deg to meters
  //check angle
  send_state("Left_turn",dangle);
  Serial.print("Turning Left: ");
  Serial.print(dangle);
  distance = 0;
  transitions = 0;
  gps_state = get_gpsloc();
  while(distance < abs(dangle) ||  abs(gps_state.crs-goal_theta)>15){
    rotations = transitions/ppr;
    distance = rotations * circumference;
    Serial.println(distance); // you need this line otherwise transitions doesn't update!!!!
    gps_state = get_gpsloc();
  }
  smart_stop();
}

void drive_right(double dangle, double goal_theta, double curr_theta)
{
  Serial.print("Turning Right: ");
  send_state("Right_turn",dangle);
  drive_left(90, curr_theta-90);
  smart_stop();
  switch_motors();
  drive_left(90-abs(dangle), goal_theta);
  Serial.print(dangle);
  smart_stop();
}

struct gps_lat_lng get_gpsloc(){
  struct gps_lat_lng loc;
  int count = 0;
  ss.listen();
  while (count < 100){
    if (ss.available() > 0){
      if (gps.encode(ss.read()))
      {
        loc.latitude = gps.location.lat();
        loc.longitude =  gps.location.lng();
        loc.crs = gps.course.deg();
        count = 101;
      }
    }
    count = count +1; 
  }
  return loc;
}

void drive_straight(double dist)
{
  dist = dist-(delta_dir*5/90);
  if (dist > 5){
    dist = 5;
  }
  digitalWrite(frontMotor2, LOW); 
  digitalWrite(backMotor2, LOW);
  
  digitalWrite(frontMotor1, HIGH);
  digitalWrite(backMotor1, HIGH);
    
  //check distance
  Serial.print("Driving_Straight");
  Serial.print(dist);
  send_state("straight: ",dist);
  // delay(3000);
  distance = 0;
  transitions = 0;
  while(distance < dist){
    rotations = transitions/ppr;
    distance = rotations * circumference;
    Serial.println(distance); // you need this line otherwise transitions doesn't update!!!!
  }
  smart_stop();
}

void encoder_counter() {
  transitions += 1;
}

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

//notes:

//Antenna
      // To set up antenna connect ground to set.
      // type AT into monitor and it will reply with OK
      // to change Baud Rate: AT+Bxxxx ex. AT+B9600
      // to change Communication Channel: AT+Cxxx ex. AT+C006

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
