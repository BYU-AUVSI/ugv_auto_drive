# UGV Autonomous Drive
2019-2020:
Advice for next year's team
https://youtu.be/oaVgXmCSSKs

Changed to Arduino Nano for simplicity of use. Using HC-12 as antenna (need to check if actually compliant with competition guidelines, 2019 team said something about not being FCC compliant), and a UBLOX NEO-M8N for GPS signal. We are using the Arduino TinyGPS++ and the pre-installed SoftwareSerial libraries.

HC-12 antenna settings:
- to change settings connect ground to "set" pin on antenna
- Arduino monitor must be on same baud rate as communication
- send "AT" to HC-12 and antenna will return with "OK"
- "AT+RX" will return all of the current settings
- "AT+Bxxxxx" will set baud rate
- "AT+FUx" will set power/range settings
- "AT+Cxxxx" will set channel
Current Settings:
- FU4 (high power and long distance, max of 1200 baud rate. The default is FU3, )
- baud rate: 1200 (default is 9600)

2018-19:
Autonomous drive software for UGV, written for OpenPilot Revolution using the airbourne_f4 library for hardware abstraction.
