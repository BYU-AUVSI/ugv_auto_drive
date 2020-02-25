#include <TinyGPS++.h>
#include <SoftwareSerial.h>

//define pins
#define backEncoder 2
#define frontEncoder 3
#define HCRXPin 4
#define HCTXPin 5
#define ATpin 6 // used to switch to AT mode
#define gpsRXPin 7
#define gpsTXPin 8
int frontMotor1=9;
int frontMotor2=10;
int backMotor1=11;
int backMotor2=12;

// define baud rates
#define GPSBaud 9600
#define HC12Baud 9600
#define monitorBaud 1200

// define global variables
double distance_to_goal, delta_dir, after_lat, after_lng, curr_dir;
double straight_lat, straight_lng, course1, course2;
double goal_dir, init_lat, init_lng, distance_covered, x, y, h;
double Goal_Lat = 40.246204, Goal_Lng = -111.646780;
String s,sa,sb;
bool writer;
int    polyCorners  = 4; // how many cornersM the polygon has
float  polyX[] = {38.14615, 38.14635, 38.14557, 38.14541 };  //latitudinal coordinates of corners
float  polyY[] = {-76.42668, -76.42617, -76.42608, -76.42661}; //longitudinal coordinates of corners

// define states
enum States {in_air, left, right, straight, wait, navigate, triangulate};
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
  
  digitalWrite(ATpin, HIGH); // For command mode set to high
  digitalWrite(frontMotor1, LOW);
  digitalWrite(frontMotor2, LOW);
  digitalWrite(backMotor1, LOW);
  digitalWrite(backMotor2, LOW);
  
  bool writer = false; 

  state = in_air;
  distance_to_goal = 100;
}

void loop() {

  check_boundary_area();
  
  switch (state){
    case in_air:
        ss.listen();
        h = -1;
        if (ss.available() > 0) // if gps is working
        {
          if (gps.encode(ss.read())) //if you are able to encode the gps object
          {
            Serial.println("made it2");
            if (gps.altitude.isUpdated())
              h = gps.altitude.meters();
          }
        }
      check_for_messages();
      h = get_height();
      if (h < 10 && h != -1){
        state = triangulate;
      }
      break;
    case left:
      drive_left(delta_dir);
      break;
    case wait:
      Serial.print(F("stopping"));
      stop_fun();
      break;
    case navigate:
      ss.listen();
      if (ss.available() > 0) // if gps is working
      {
        if (gps.encode(ss.read())) //if you are able to encode the gps object
        {
          Serial.print(F("going into get_path"));
          get_path();
        }
      }
      break;
    case triangulate:
      ss.listen();
      if (ss.available() > 0) // if gps is working
      {
        if (gps.encode(ss.read())) //if you are able to encode the gps object
        {
          get_n_set_orientation;
        }
      }
  }
  
  if (distance_to_goal < 3)
  {
    state = wait;
    Serial.print("You Did It");
  }
  
}

void check_boundary_area(){
  ss.listen();
  if (ss.available() > 0) // if gps is working
  {
    if (gps.encode(ss.read())) //if you are able to encode the gps object
    {
      x = gps.location.lat();
      y = gps.location.lng();
      if (pointInPolygon()){
        stop_fun();
        state = wait;
        Serial.print("You went outside the boundaries");
        while(1);
      }   
    }
  }
}

void get_n_set_orientation(){
  init_lat = gps.location.lat();
  init_lng = gps.location.lng();
  drive_straight(5);
  straight_lat = gps.location.lat();
  straight_lng = gps.location.lng();
  drive_left(90);
  drive_straight(5);
  stop_fun();
  after_lat = gps.location.lat();
  after_lng = gps.location.lng();
  course1 = TinyGPSPlus::courseTo(init_lat,init_lng,straight_lat,straight_lng);
  course2 = TinyGPSPlus::courseTo(straight_lat,straight_lng,after_lat,after_lng);
  if (course1 <= 90 && course2 > 90){
    if (course2 > 225){
      //keep motors the way they are
      state = navigate;
    }
    else{
      //switch motors
      switch_motors();
    }
  }
  else{
    if (course1 <= 180 && (course2 > 180 || course2 < 90)){
      if (course2 < 90 || course2 > 315){
        // keep motors the way they are
        state = navigate; 
      }
      else{
        //switch motors
        switch_motors();
      }
    }
    else{
      if (course1 <= 270 && (course2 > 270 || course2 < 180)){
        if (course2 < 180 || course2 > 45){
          //keep motors the way they are
          state = navigate;
        }
        else{
          //switch motors
          switch_motors();
        }
      }
      else{
        if (course1 > 270 && course2 < 270){
          if (course2 < 270 || course2 > 135){
            //keep motors the way they are
            state = navigate;
          }
          else{
            //switch motors
            switch_motors();
          }
        }
      }
    }
  }
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
}

double get_height(){
  ss.listen();
  h = -1;
  if (ss.available() > 0) // if gps is working
  {
    if (gps.encode(ss.read())) //if you are able to encode the gps object
    {
      Serial.print("made it2");
      if (gps.altitude.isUpdated())
        h = gps.altitude.meters();
    }
  }
  return h;
}

void check_for_messages(){
  HC12.listen();
  while (HC12.available()) {        // Check if HC-12 has data
    byte c = HC12.read();
    s += char(c);
    writer = true;
  }
  ss.listen();
  smartDelay(0);
}

void send_gpsloc(double x, double y)
{
  // send two doubles seperated by a comma
  HC12.listen();
  sa = String(x,10);
  sb = String(y,10);
  s = sa+','+sb;
  HC12.print(s);
  HC12.write(";");
}

void stop_fun()
{
  digitalWrite(frontMotor1, LOW);
  digitalWrite(frontMotor2, LOW);
  digitalWrite(backMotor1, LOW);
  digitalWrite(backMotor2, LOW);
}

void get_path()
{
  init_lat = gps.location.lat();
  init_lng = gps.location.lng();
  distance_to_goal =
  TinyGPSPlus::distanceBetween(
    gps.location.lat(),
    gps.location.lng(),
    Goal_Lat,
    Goal_Lng);
  int count = 0;
  //drive forward
  // delay(1000);
  after_lat = gps.location.lat();
  after_lng = gps.location.lng();
  curr_dir = gps.course.deg();
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
  HC12.write("Hello World");
  ss.listen();
  smartDelay(0);
  //change direction
  count = count +1;
  if (count > 10)
  {
      if(delta_dir > 0)
      {
        drive_left(delta_dir);
      }
      else
      {
        drive_right(delta_dir);
      }
      //drive in direction
      drive_straight(distance_to_goal);
  }
  //setup for next loop
  init_lat = after_lat;
  init_lng = after_lng;
  send_gpsloc(init_lat,init_lng);
}

void drive_left(double dangle)
{
  digitalWrite(frontMotor1, LOW); 
  digitalWrite(backMotor1, LOW);
  
  digitalWrite(frontMotor2, HIGH);
  digitalWrite(backMotor2, HIGH);
  
  //check angle
  Serial.print("Turning Left: ");
  Serial.print(dangle);
  // delay(3000);
}

void drive_right(double dangle)
{
  digitalWrite(frontMotor2, LOW); 
  digitalWrite(backMotor2, LOW);
  
  digitalWrite(frontMotor1, HIGH);
  digitalWrite(backMotor1, HIGH);
  
  //check angle 
  Serial.print("Turning Right: ");
  Serial.print(dangle);
  // delay(3000);
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
}

//notes:

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
