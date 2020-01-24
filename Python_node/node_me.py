#ftdi://0x10c4:0xea60
#pyusb tutorial
# import usb.core
# import usb.util
#
# # find our device
# dev = usb.core.find(idVendor=0x10c4, idProduct=0xea60)
#
# # was it found?
# if dev is None:
#     raise ValueError('Device not found')
#
# #need to run this section otherwise you get busy error
# reattach = False
# if dev.is_kernel_driver_active(0):
#     reattach = True
#     dev.detach_kernel_driver(0)
#
# # set the active configuration. With no arguments, the first
# # configuration will be the active one
# dev.set_configuration()
#
# # get an endpoint instance
# cfg = dev.get_active_configuration()
# intf = cfg[(0,0)]
#
# ep = usb.util.find_descriptor(
#     intf,
#     # match the first OUT endpoint
#     custom_match = \
#     lambda e: \
#         usb.util.endpoint_direction(e.bEndpointAddress) == \
#         usb.util.ENDPOINT_OUT)
#
# assert ep is not None
#
# # write the data
# ep.write('test')

# ## Might possible need this part??????
# # This is needed to release interface, otherwise attach_kernel_driver fails
# # due to "Resource busy"
# usb.util.dispose_resources(dev)
#
# # It may raise USBError if there's e.g. no kernel driver loaded at all
# if reattach:
#     dev.attach_kernel_driver(0)

#### Round 2

import serial
ser = serial.Serial()
ser.baudrate = 9600
ser.port = '/dev/ttyUSB1'
ser.open()
ser.is_open

ser.write(b'hello')

#ser.read(100)

while True:
    s = ser.read_until(b';')
    print(s)



ser.close()
ser.is_open
