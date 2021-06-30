#!/usr/bin/env python

from tabulate import tabulate
import can
import struct
import curses
import subprocess
import time
import datetime


class Dataa:
    def __init__(self):
        self.bus=can.interface.Bus(bustype="socketcan",channel="can0",bitrate=250000)
        self.avg_Stater_Crnt=0
        self.avgMtr_PhaseV=0
        self.tgt_torq=0
        self.mtractl_torq=0
        self.propspeedlmt=0
        self.MotorRPM=0
        self.calbattery_crnt=0
        self.ctrlcapctr_v=0
        self.trottleip=0
        self.mtr_temp=0
        self.ctrlr_temp=0
        self.dis_traveld=0
        self.drive_dirctn=0
        self.forward=0
        self.reverse=0
        self.seat_s=0
        self.sternAngle=0
      
    def twos_comp(self, val, bits):
        if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
            val = val - (1 << bits)        # compute negative value
        return val 

    def to_little(self,val):
        little_hex = bytearray.fromhex(val)
        little_hex.reverse()
        # print("Byte array format:", little_hex)
        str_little = ''.join(format(x, '02x') for x in little_hex)
        return str_little
        
    def spe(self):
       #Every Can id except '0x292' is  little endian and swap to get correct value.
       #for '0x292' not in little endian, so no need to swap.
        self.message=self.bus.recv()
        
        if self.message.arbitration_id==0x201:
            self.can_info_torq={'data':None,'time':None}
            self.time_at_201=rospy.get_time()
            self.can_info_torq['data']=self.mtractl_torq
            self.can_info_torq['time']=self.time_at_201
            
            self.a=bytearray(self.message.data)
            self.he=self.a.hex() #converting bytearray to hex format

            ccd=self.to_little(self.he[0:4])
            self.avg_Stater_Crnt=int(ccd,16)
            self.avg_Stater_Crnt= self.twos_comp(self.avg_Stater_Crnt,16)

            ab=self.to_little(self.he[4:8])
            self.avgMtr_PhaseV=int(ab,16)
            self.avgMtr_PhaseV= self.twos_comp(self.avgMtr_PhaseV,16)*0.0625 

            de=self.to_little(self.he[12:16])
            self.tgt_torq=int(de,16)
            self.tgt_torq= self.twos_comp(self.tgt_torq,16)*0.0625

            ef= self.to_little(self.he[8:12])
            self.mtractl_torq=int(ef,16)
            self.mtractl_torq= self.twos_comp(self.mtractl_torq,16)*0.0625 

        elif self.message.arbitration_id==0x292:    
            self.steering=bytearray(self.message.data)
            self.he= self.steering.hex()

            op = self.he[2:6] #Not little endian. No need to swap
            self.sternAngle= (int(op,16))/4

        elif self.message.arbitration_id==0x202:
            self.a=bytearray(self.message.data)
            self.he=self.a.hex() #converting bytearray to hex format

            fg= self.to_little(self.he[0:8])
            self.propspeedlmt=int(fg,16)
            self.propspeedlmt= self.twos_comp(self.propspeedlmt,32)
            
            gh= self.to_little(self.he[8:])
            self.MotorRPM=int(gh,16)
            self.MotorRPM= self.twos_comp(self.MotorRPM,32)

        elif self.message.arbitration_id==0x203:
            self.a=bytearray(self.message.data)
            self.he=self.a.hex() #converting bytearray to hex format

            hi=self.to_little(self.he[0:4])
            self.calbattery_crnt=int(hi,16)
            self.calbattery_crnt=self.twos_comp(self.calbattery_crnt,16)*0.0625

            ij=self.to_little(self.he[4:8])
            self.ctrlcapctr_v=int(ij,16)*0.0625

            jk=self.to_little(self.he[8:12])
            self.trottleip=int(jk,16)
            self.trottleip=self.twos_comp(self.trottleip,16)*0.00390625

        elif self.message.arbitration_id==0x204:
            self.a=bytearray(self.message.data)
            self.he=self.a.hex() #converting bytearray to hex format

            kl=self.to_little(self.he[0:4])
            self.mtr_temp=int(kl,16)
            self.mtr_temp=self.twos_comp(self.mtr_temp,16)

            self.ctrlr_temp=int(self.he[4:6],16)
            # self.ctrlr_temp1=int(self.he[4],16)
            # self.ctrlr_temp2=int(self.he[5],16)
            # self.ctrlr_temp=self.ctrlr_temp2+self.ctrlr_temp1
            self.ctrlr_temp=self.twos_comp(self.ctrlr_temp,8)

            lm=self.to_little(self.he[6:14])
            self.dis_traveld=int(lm,16)
            self.dis_traveld=self.dis_traveld*0.00390625

            mn=self.to_little(self.he[14:16])
            self.drive_dirctn=int(mn,16)
            if self.drive_dirctn==0:
                self.forward=0
                self.reverse=0
                self.seat_s=0
            elif self.drive_dirctn==1:
                self.forward=1
                self.reverse=0
                self.seat_s=0
            elif self.drive_dirctn==2:
                self.forward=0
                self.reverse=1
                self.seat_s=0
            elif self.drive_dirctn==4:
                self.forward=0
                self.reverse=0
                self.seat_s=1
            elif self.drive_dirctn==5:
                self.forward=1
                self.reverse=0
                self.seat_s=1
            elif self.drive_dirctn==6:
                self.forward=0
                self.reverse=1
                self.seat_s=1
            else:
                pass

        # else:
        #     print("Dont worry")

    def table1(self):
        while True:
            self.spe()
            file= open("mytext.txt","w")
            self.table=[["Name", "Value"],["AverageMotorStatorCurrent", self.avg_Stater_Crnt],["AverageMotorPhaseVoltage", self.avgMtr_PhaseV],["Target Trque", self.tgt_torq],["Motor Actual Torque", self.mtractl_torq],["Steering Angle", self.sternAngle],["Proportional Speed Limit", self.propspeedlmt],["Motor RPM", self.MotorRPM],["Calculated Battery Current",self.calbattery_crnt],["Controller Capacitor Voltage",self.ctrlcapctr_v],["Throttle input",self.trottleip],["Motor Temp",self.mtr_temp],["Controller Temperature", self.ctrlr_temp],["Distance travelled", self.dis_traveld],["Forward switch",self.forward],["Reverse Switch", self.reverse],["Seat Switch", self.seat_s]]
            a=tabulate(self.table, headers="firstrow", tablefmt="pretty")
            file.write(str(a))
            file.close()
            b=str(a)
            print(b)
            # return a

    def valuess(self):
        # while True:
        self.spe()
        
        self.values_to_add= ["AverageMotorStatorCurrent", self.avg_Stater_Crnt],["AverageMotorPhaseVoltage", self.avgMtr_PhaseV],["Target Trque", self.tgt_torq],["Motor Actual Torque",self.mtractl_torq],["Steering Angle", self.sternAngle],["Proportional Speed Limit", self.propspeedlmt],["Motor RPM", self.MotorRPM],["Calculated Battery Current",self.calbattery_crnt],["Controller Capacitor Voltage",self.ctrlcapctr_v],["Throttle input",self.trottleip],["Motor Temp",self.mtr_temp],["Controller Temperature",self.ctrlr_temp],["Distance travelled", self.dis_traveld],["Forward switch",self.forward],["Reverse Switch",self.reverse],["Seat Switch",self.seat_s]
        # for i in range(len(self.valuess)):
            # print('\n'.join(map(str, self.values_to_add)))   
        return self.values_to_add
        

    def valuess1(self):
        # while True:
        self.spe()
        self.values_to_add= [self.avg_Stater_Crnt,self.avgMtr_PhaseV,self.tgt_torq, self.mtractl_torq,self.sternAngle,self.propspeedlmt, self.MotorRPM,self.calbattery_crnt,self.ctrlcapctr_v, self.trottleip,self.mtr_temp, self.ctrlr_temp,self.dis_traveld,self.forward,self.reverse,self.seat_s]
        # for i in self.values_to_add:
        return self.values_to_add    


# A= Dataa()
# A.valuess1() 
# A.valuess()  
# A.terminal_update()
# while True:
#     a=A.valuess()

#     print(a)
# A.spe() 
# while True:
# A.table1()
