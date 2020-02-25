#Python3
#!/usr/bin/env python

import serial
import rospy
import time

# Wait for 5 seconds

from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    ser = serial.Serial()
    ser.baudrate = 1200 # 9600
    while not rospy.is_shutdown():
    ####### Code for communication
        for i in range(256):
        try:
            ser.port = '/dev/ttyUSB'+str(i)
            ser.open()
            if ser.is_open:
                break
        except serial.SerialException:
            pass

        ser.write(b'Hello World')
        x = 0
        while True:
            x = x+1
            ser.write(x)
            ser.write(b'hello;')
            s = ser.read_until(b';')
            print(s)

        ser.write(b'straight,1,;')

        while True:
            s = ser.read_until(b';')
            print(s)

    ser.close()
    ### end of code for communication


 if __name__ == '__main__':
     try:
        talker()
     except rospy.ROSInterruptException:
        pass


"""Notes

#url for serial port if needed:  ftdi://0x10c4:0xea60

For Testing:
This code will write consecutive integers to the hc12 and the hc12 should send the same thing back
---------------
while True:
    x = x+1
    stringer = str(x)
    ser.write(bytes(stringer,'utf-8'))
    ser.write(b';')
    s = ser.read_until(b';')
    print(s)
----------------

"""
