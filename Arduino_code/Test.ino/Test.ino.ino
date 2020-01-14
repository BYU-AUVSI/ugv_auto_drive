#include <TinyGPS++.h>
#include <SoftwareSerial.h>

double distance_to_goal;
double delta_dir;

static const int RXPin = 2, TXPin = 3; // according to the gps
static const uint32_t GPSBaud = 9600;
static const uint32_t HC12Baud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(TXPin, RXPin);
SoftwareSerial HC12(5, 6); // HC-12 TX Pin, HC-12 RX Pin
const double Goal_Lat = 40.246204, Goal_Lng = -111.646780;

void setup()
{
  digitalWrite(9, LOW);   // start with everything off
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);   
  digitalWrite(12, LOW);   

  Serial.begin(115200);
  ss.begin(GPSBaud);
  HC12.begin(HC12Baud);               // Serial port to HC12

  Serial.println(F("HelloWorld"));
  Serial.println();
}

void loop()
{
  ss.listen();
  while (ss.available() > 0)
  {
      if (gps.encode(ss.read()))
    {
      Serial.print(F("it is going into nav_code"));
      nav_code();
    }
  }

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
  }
}

void nav_code()
{
  double after_lat;
  double after_lng;
  double curr_dir;
  double goal_dir;
  double init_lat = gps.location.lat();
  double init_lng = gps.location.lng();
  double distance_covered;
  distance_to_goal =
  TinyGPSPlus::distanceBetween(
    gps.location.lat(),
    gps.location.lng(),
    Goal_Lat,
    Goal_Lng);
  int count = 0;
  while(distance_to_goal > 3)
    {
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
      Serial.print(after_lat);
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
    }
    Serial.print("You Did It");
    //printInt(distance_meters, gps.location.isValid(), 9);
    //Serial.println();
    //printFloat(dangle, gps.location.isValid(), 7, 2);
    //Serial.println();
}

void drive_left(double dangle)
{
  digitalWrite(12, LOW);  
  digitalWrite(10, LOW); 
  digitalWrite(11, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(9, HIGH);   // turn the LED on (HIGH is the voltage level)
  //check angle
  Serial.print("Turning Left: ");
  Serial.print(dangle);
  // delay(3000);
}

void drive_right(double dangle)
{
  digitalWrite(9, LOW);  
  digitalWrite(11, LOW); 
  digitalWrite(10, HIGH);
  digitalWrite(12, HIGH); 
  //check angle 
  Serial.print("Turning Right: ");
  Serial.print(dangle);
  // delay(3000);
}

void drive_straight(double dist)
{
  digitalWrite(12, LOW);  
  digitalWrite(9, LOW); 
  digitalWrite(11, HIGH);   
  digitalWrite(10, HIGH);   
  //check distance
  Serial.print("Driving Straight: ");
  Serial.print(dist);
  // delay(3000);
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


  

