//#include <TinyGPS++.h>
//#include <SoftwareSerial.h>
//
//double distance_to_goal;s
//double delta_dir;
//
//static const int RXPin = 2, TXPin = 3; // according to the gps
//static const uint32_t GPSBaud = 9600;
//static const uint32_t HC12Baud = 9600; //9600
//double after_lat;
//double after_lng;
//double curr_dir;
//double goal_dir;
//double init_lat;
//double init_lng;
//double distance_covered;
//
//// The TinyGPS++ object
//TinyGPSPlus gps;
//
//// The serial connection to the GPS device
//SoftwareSerial ss(TXPin, RXPin);
//SoftwareSerial HC12(3, 4); // HC-12 TX Pin, HC-12 RX Pin
//const double Goal_Lat = 40.246204, Goal_Lng = -111.646780;
#define ATpin 5 // used to switch to AT mode
#include <SoftwareSerial.h>
SoftwareSerial HC12(4, 3); // HC-12 TX Pin, HC-12 RX Pin
double x,y;
String s,sa,sb;
bool writer;

void setup() {
  Serial.begin(9600);//1200);             // Serial port to computer
  HC12.begin(9600);//1200);               // Serial port to HC12
  x = -111.100239;
  y = 42.0213923;
  pinMode(ATpin, OUTPUT);
  digitalWrite(ATpin, LOW); // AT Mode
  bool writer = false;
}
void loop() {
    while (HC12.available())  // If HC-12 has data
  {       
    Serial.write(HC12.read());      // Send the data to Serial monitor
  }
  while (Serial.available()) // If Serial monitor has data
  {      
    HC12.write(Serial.read());      // Send that data to HC-12
  }

//  while (HC12.available()) {        // If HC-12 has data
//    byte c = HC12.read();
//    s += char(c);
//    writer = true;
//  }
//
//  if (s.endsWith(";") && writer == true) {
//      HC12.print(s);
//      Serial.print(s);
//      s = "";
//      writer = false;
//    }
//  x = x+1.02;
//  y = y+1;
//  sa = String(x,10);
//  sb = String(y,10);
//  s = sa+','+sb;
//  HC12.print(s);
//  HC12.write(";");
//  Serial.println(x);
//  delay(1000);
//  
//  while (Serial.available()) // If Serial monitor has data
//  {      
//    HC12.write(Serial.read());      // Send that data to HC-12
//  }
//  delay(1000);
//  while (Serial.available()) {      // If Serial monitor has data
//    //char c = Serial.read();  //gets one byte from serial buffer
//    //HC12.write(b'hello');      // Send that data to HC-12
//    //Serial.print(Serial.available())
//    //HC12.write(10);
//  }
}


//void setup()
//{
////  distance_to_goal = 100;
////  digitalWrite(9, LOW);   // start with everything off
////  digitalWrite(10, LOW);
////  digitalWrite(11, LOW);   
////  digitalWrite(12, LOW);   
//
//  Serial.begin(115200);
//  ss.begin(GPSBaud);
//  HC12.begin(HC12Baud);               // Serial port to HC12
// 
//  Serial.println(F("HelloWorld"));
//  Serial.println();
//  HC12.listen();
//}
//
//void loop()
//{
////  HC12.write("Hello World");
//  while (HC12.available()) {        // If HC-12 has data
//    Serial.print(HC12.read());      // Send the data to Serial monitor
//    //Serial.print("I recieved Stuff");
//  }
//  while (Serial.available()) {      // If Serial monitor has data
//    HC12.write(Serial.read());      // Send that data to HC-12
//    //Serial.print("I sent stuff");
//  }
//}
//
//void winner()
//{
//  while (1){
//    Serial.print("You Did It");
//  }
//}
//
//void nav_code()
//{
//  init_lat = gps.location.lat();
//  init_lng = gps.location.lng();
//  distance_to_goal =
//  TinyGPSPlus::distanceBetween(
//    gps.location.lat(),
//    gps.location.lng(),
//    Goal_Lat,
//    Goal_Lng);
//  int count = 0;
//    //drive forward
//    // delay(1000);
//    after_lat = gps.location.lat();
//    after_lng = gps.location.lng();
//    curr_dir = gps.course.deg(); // TinyGPSPlus::courseTo(init_lat,init_lng,after_lat,after_lng);
//    distance_to_goal = TinyGPSPlus::distanceBetween(after_lat,after_lng,Goal_Lat,Goal_Lng);
//    goal_dir = TinyGPSPlus::courseTo(after_lat,after_lng,Goal_Lat,Goal_Lng);
//    delta_dir = curr_dir-goal_dir; // turn left if positive, right if negative;
//    if(abs(delta_dir) > 180) // test so the car always turns the least amount
//    {
//      delta_dir = (360-abs(delta_dir))*sign(delta_dir)*-1; //
//    }
//    Serial.print(distance_to_goal);
//    Serial.print("\t");
//    Serial.print(delta_dir);
//    Serial.print("\t");
//    Serial.print(after_lng);
//    Serial.println();
//    HC12.listen();
//    HC12.write("Hello World");
//    ss.listen();
//    smartDelay(0);
//    //change direction
//    count = count +1;
//    if (count > 10)
//    {
//        if(delta_dir > 0)
//        {
//          drive_left(delta_dir);
//        }
//        else
//        {
//          drive_right(delta_dir);
//        }
//        //drive in direction
//        drive_straight(distance_to_goal);
//    }
//    //setup for next loop
//    init_lat = after_lat;
//    init_lng = after_lng;
//    //printInt(distance_meters, gps.location.isValid(), 9);
//    //Serial.println();
//    //printFloat(dangle, gps.location.isValid(), 7, 2);
//    //Serial.println();
//}
//
//void drive_left(double dangle)
//{
//  digitalWrite(12, LOW);  
//  digitalWrite(10, LOW); 
//  digitalWrite(11, HIGH);   // turn the LED on (HIGH is the voltage level)
//  digitalWrite(9, HIGH);   // turn the LED on (HIGH is the voltage level)
//  //check angle
//  Serial.print("Turning Left: ");
//  Serial.print(dangle);
//  // delay(3000);
//}
//
//void drive_right(double dangle)
//{
//  digitalWrite(9, LOW);  
//  digitalWrite(11, LOW); 
//  digitalWrite(10, HIGH);
//  digitalWrite(12, HIGH); 
//  //check angle 
//  Serial.print("Turning Right: ");
//  Serial.print(dangle);
//  // delay(3000);
//}
//
//void drive_straight(double dist)
//{
//  digitalWrite(12, LOW);  
//  digitalWrite(9, LOW); 
//  digitalWrite(11, HIGH);   
//  digitalWrite(10, HIGH);   
//  //check distance
//  Serial.print("Driving Straight: ");
//  Serial.print(dist);
//  // delay(3000);
//}
//
//static void printFloat(float val, bool valid, int len, int prec)
//{
//  if (!valid)
//  {
//    while (len-- > 1)
//      Serial.print('*');
//    Serial.print(' ');
//  }
//  else
//  {
//    Serial.print(val, prec);
//    int vi = abs((int)val);
//    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
//    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
//    for (int i=flen; i<len; ++i)
//      Serial.print(' ');
//  }
//  smartDelay(0);
//}
//
//static void printInt(unsigned long val, bool valid, int len)
//{
//  char sz[32] = "*****************";
//  if (valid)
//    sprintf(sz, "%ld", val);
//  sz[len] = 0;
//  for (int i=strlen(sz); i<len; ++i)
//    sz[i] = ' ';
//  if (len > 0) 
//    sz[len-1] = ' ';
//  Serial.print(sz);
//  smartDelay(0);
//}
//
//static void smartDelay(unsigned long ms)
//{
//  unsigned long start = millis();
//  do 
//  {
//    while (ss.available())
//      gps.encode(ss.read());
//  } while (millis() - start < ms);
//}
//
//int sign(double x)
//{
//  int signOfX = (x > 0) - (x < 0);
//  return signOfX;
//}

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


  
