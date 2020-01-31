#include <TinyGPS++.h>
#include <SoftwareSerial.h>

//define pins
#define gpsRXPin 2
#define gpsTXPin 3
#define HCRXPin 4
#define HCTXPin 5
#define ATpin 6 // used to switch to AT mode
#define frontMotor1 7
#define frontMotor2 8
#define backMotor1 9
#define backMotor2 10
#define frontEncoder 11
#define backEncoder 12

// define baud rates
#define GPSBaud 9600
#define HC12Baud 1200
#define monitorBaud 1200

// define global variables
double distance_to_goal, delta_dir, after_lat, after_lng, curr_dir;
double goal_dir, init_lat, init_lng, distance_covered, x, y;
double Goal_Lat = 40.246204, Goal_Lng = -111.646780;
String s,sa,sb;
bool writer;

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
}

void loop() {

  HC12.listen();
  while (HC12.available()) {        // Check if HC-12 has data
    byte c = HC12.read();
    s += char(c);
    writer = true;
  }

  ss.listen();
  if (ss.available() > 0) // if gps is working
  {
      if (gps.encode(ss.read())) //if you are able to encode the gps object
    {
      Serial.print(F("it is going into get_path"));
      get_path();
    }
  }

  if (distance_to_goal < 3)
  {
    stop_fun();
  }

  

  
  
}

void send_gps(double x, double y)
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
  while (1){
    Serial.print("You Did It");
  }
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
  send_gps(init_lat,init_lng);
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

  
