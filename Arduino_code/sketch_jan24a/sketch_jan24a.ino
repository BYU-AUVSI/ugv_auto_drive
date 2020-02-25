

/*********************************
Simple demonstration of HC-12 433mHz Radio
Connect Vcc to Arduino 5volts
Connect Gnd to ArduinoGnd
Connect SET to Arduino pin 10 (Take LOW to send AT Commands)

To switch HC-12 to AT Mode, take AT PIN LOW
To return to Normal Mode take ATPIN HIGH

*********************************/

#include <SoftwareSerial.h>
#define ATpin 5 // used to switch to AT mode
#define LEDpin 13
SoftwareSerial HC12Serial(3,4); // RX,TX --> GOES TO TX,RX OF DEVICE

void setup() {
Serial.begin(9600);
//Serial.begin(4800);
Serial.setTimeout(20);
pinMode(ATpin, OUTPUT);
//digitalWrite(ATpin, HIGH); // normal mode (Alternate between High and Low, Low mode is for programming, normal mode is for functioning)
digitalWrite(ATpin, LOW); // AT Mode
//pinMode(LEDpin, OUTPUT);
//digitalWrite(LEDpin, LOW);
HC12Serial.begin(9600);
HC12Serial.setTimeout(20);
}

void loop() {
if(Serial.available() > 0){ // Read sent TO HC-12
String input = Serial.readString();
HC12Serial.println(input);
}
if(HC12Serial.available() > 0){ // send data FROM HC-12
String input = HC12Serial.readString();
Serial.println(input);
}
delay(3000);
}

/*****************

• AT Test command.  Send command "AT" to the module, and the module returns "OK".

• AT+Bxxxx  Change the serial port baud rate. The baud rate can be set to 1200bps, 2400bps, 4800bps, 9600bps, 19,200bps, 38,400bps, 57,600bps, or 115,200bps.
The default value is 9600bps. e.g: To set the serial port baud rate of the module to 19,200bps, send command "AT+B19200" to the module, and the module will return "OK+B19200".
After exiting from command mode, the module will begin to communicate at 19,200bps.

• AT+Cxxx  Change wireless communication channel, selectable from 001 to 127 (for wireless channels exceeding 100, the communication distance cannot be guaranteed).
The default value for the wireless channel is 001, with a working frequency of 433.4MHz.
The channel stepping is 400KHz, and the working frequency of channel 100 is 473.0MHz.
e.g: To set the module to work on channel 21, send command "AT+C021" to the module, and the module will return "OK+C021".
After exiting from command mode, the module will work on channel 21, with a working frequency of 441.4MHz.
Note: As the wireless receiving sensitivity of the HC-12 module is relatively high, when the serial port baud rate is greater than 9600bps five adjacent channels should be staggered for use.
Even when the serial port baud rate is not greater than 9600bps, over short distances (less than 10m) also five adjacent channels should be staggered for use.

• AT+FUx  Change the serial port transparent transmission mode of the module.
Four modes are available, namely FU1, FU2, FU3, and FU4. Only when the serial port speed, channel, and transparent transmission mode of two modules is set to be the same, can normal wireless communications occur.
For more details, please see the above section "Wireless Serial Port Transparent Transmission". e.g: Send command "AT+FU1" to the module, and the module returns "OK+FU1".

• AT+Px  Set the transmitting power of the module, with x selectable from 1 to 8.
The corresponding transmitting power of the module is as shown below: x value 1 2 3 4 5 6 7 8
Transmitting power of module -1 dBm (0.8mW) 2 dBm (1.6mW) 5 dBm (3.2mW) 8 dBm (6.3mW) 11 dBm (12mW) 14 dBm (25mW) 17 dBm (50mW) 20 dBm (100mW)
The default value is 8, and the higher the transmitting power, the further the possible wireless communication distance.
When the transmitting power level is set to 1, the transmitting power is at the minimum.
Generally speaking, every time the transmitting power is reduced by 6dB, the communication distance will be reduced by half.
e.g: Send command "AT+P5" to the module, and the module returns "OK+P5".
After exiting from command mode, the transmitting power of the module will be set to 11dBm.

• AT+Ry  Obtain a single parameter from the module, where y is any letter among B, C, F, and P, respectively representing:
baud rate, communication channel, serial port transparent transmission mode, and transmitting power.
Example 1: Send command "AT+RB" to the module, and if the module returns "OK+B9600" it is confirmed that the serial port baud rate of the module is 9600bps.
Example 2: Send command "AT+RC" to the module, and if the module returns "OK+RC001" it is confirmed that the communication channel of the module is 001.
Example 3: Send command "AT+RF" to the module, and if the module returns "OK+FU3" it is confirmed that the module is working in serial port transparent transmission mode FU3.
Example 4: Send command "AT+RP" to the module, and if the module returns "OK+RP:+20dBm" it is confirmed that the transmitting power of module is set to 20dBm (100mW).

• AT+RX  Obtain all parameters from the module.
Returns serial port transparent transmission mode, serial port baud rate, communication channel, and transmitting power in that order.
e.g: Send command "AT+RX" to the module, and the module returns "OK+FU3\r\n OK+B9600\r\n OK+C001\r\n OK+RP:+20dBm\r\n". ("\r\n" means return\newline)

• AT+Udps Set data bits (d), parity (p), and stop bits (s) for serial port communication.
For parity, N means none, O means odd check, and E means even check.
For stop bits, 1 means one stop bit, 2 means two stop bits, and 3 means 1.5 stop bits.
e.g: To set the serial port format to eight data bits, odd parity, and one stop bit, send command "AT+U8O1" to the module.
The module will return "OK+U8O1".

• AT+V  Request firmware version information from the module. e.g: Send command "AT+V" to the module, and the module returns "HC-12_V2.3".

• AT+SLEEP  After receiving this command, the module will enter sleep mode upon exiting from command mode, with a working current of about 22uA.
This mode doesn't allow serial port data transmission.
Upon entering command mode again the module will exit from sleep mode automatically.
e.g: When wireless data transmission is not needed, to save power send command "AT+SLEEP" to the module, and the module will return "OK+SLEEP".
Upon exit from command mode the working current will drop to about 22uA.

• AT+DEFAULT  Set serial port baud rate and configuration, communication channel, power, and serial port transparent transmission mode back to the factory default values.
e.g: Send command "AT+DEFAULT" to the module, and the module returns "OK+DEFAULT", with the factory default values restored.
The factory default serial port baud rate is 9600bps, 8 data bits, no parity, 1 stop bit, communication channel is 001, transmitting power is 20dBm, and serial port transparent transmission mode is FU3.

• AT+UPDATE  Puts the module in the state of waiting for a software update.
After receiving this command the module will not respond to any further AT commands until power has been cycled.
(AT Commands from HC-12 Wireless Serial Port Communication Module User Manual version 2.3B)

 *******************/
