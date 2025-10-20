import usb.core
import usb.util
import sys

dev=usb.core.find(idVendor=0x2560, idProduct=0xc128) #lsusb to list attached devices in case your id's are different.
#print(dev) #uncomment to see the configuration tree.
#Follow the tree: dev[0] = configuration 1.
#interfaces()[2] = HID interface
#0x06 = Interrupt OUT. (Endpoint)

if dev is None:
    raise ValueError('Device not found')
cfg=-1

i = dev[0].interfaces()[2].bInterfaceNumber

cfg = dev.get_active_configuration()
intf = cfg[(2,0)]

if dev.is_kernel_driver_active(i):
    try:
        reattach = True
        dev.detach_kernel_driver(i)
        #print("eh") #debug making sure it got in here.
    except usb.core.USBError as e:
        sys.exit("Could not detach kernel driver from interface({0}): {1}".format(i, str(e)))


print(dev) #not needed, just helpful for debug
msg = [0] * 64
#msg = [0xA0, 0xc1, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00] #gotta send a 64 byte message.
msg[0] = 0xA8 #these command sets are in the qtcam source code.
msg[1] = 0x1c
msg[2] = 0x01 # 01= ext trigger. 00 = Master mode.
msg[3] = 0x00


dev.write(0x6,msg,1000) 
#wham bam, thank you ma'am. Just write msg to the 0x06 endpoint. 
#The 1000 is a timeout in ms. That should go back down, but it's an artifact of debugging. 
#Also, for debugging, I recommend using wireshark set to capture usb packets while you're sending them from qtcam.