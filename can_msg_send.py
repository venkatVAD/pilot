#! /usr/bin/env python

import can
import random
import time

bus=can.interface.Bus(bustype="socketcan",channel="vcan0",bitrate=250000)

while True:
    p=random.randint(1,255)
    q=random.randint(1,255)
    r=random.randint(1,255)
    s=random.randint(1,255)

    a=random.randint(1,255)
    b=random.randint(1,255)
    c=random.randint(1,255)
    d=random.randint(1,255)

    msg1=can.Message(arbitration_id=0x201, data=[a,b,c,d,p,q,r,s],is_extended_id=False)
    msg2=can.Message(arbitration_id=0x202, data=[a,b,c,d,p,q,r,s],is_extended_id=False)
    msg3=can.Message(arbitration_id=0x203, data=[a,b,c,d,p,q,r,s],is_extended_id=False)
    msg4=can.Message(arbitration_id=0x204, data=[a,b,c,d,p,q,r,s],is_extended_id=False)
    msg5=can.Message(arbitration_id=0x292, data=[a,b,c,d,p,q,r,s],is_extended_id=False)
    try:
        bus.send(msg1,0.2)
        # print("Speed of the Motor is sent {}".format(bus.channel_info))
        time.sleep(0.1)

        bus.send(msg2,0.2)
        # print("Speed of the Motor is sent {}".format(bus.channel_info))
        time.sleep(0.1)

        bus.send(msg3,0.2)
        # print("Speed of the Motor is sent {}".format(bus.channel_info))
        time.sleep(0.1)

        bus.send(msg4,0.2)
        # print("Speed of the Motor is sent {}".format(bus.channel_info))
        time.sleep(0.1)

        bus.send(msg5,0.2)
        # print("Speed of the Motor is sent {}".format(bus.channel_info))
        time.sleep(0.1)
        
    except:
        pass
