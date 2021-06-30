try: 
    import rospy
    import math
    from threading import Thread
    import can
    from time import time,sleep
except Exception as e:
    print('No moduled named '+e)

 

class Main_variables():
    def __init__(self):
        '''
        A class to have all the declaration of vehicle releated variables
        
        '''
        self.return_val={'status' : None ,"msg" : None,'time' : None}
        self.avg_Stater_Crnt={'status' : None ,"data" : None,'time' : None}
        self.avgMtr_PhaseV={'status' : None ,"data" : None,'time' : None}
        self.tgt_torq={'status' : None ,"data" : None,'time' : None}
        self.mtractl_torq={'status' : None ,"data" : None,'time' : None}
        self.propspeedlmt={'status' : None ,"data" : None,'time' : None}
        self.MotorRPM={'status' : None ,"data" : None,'time' : None}
        self.calbattery_crnt={'status' : None ,"data" : None,'time' : None}
        self.ctrlcapctr_v={'status' : None ,"data" : None,'time' : None}
        self.trottleip={'status' : None ,"data" : None,'time' : None}
        self.mtr_temp={'status' : None ,"data" : None,'time' : None}
        self.ctrlr_temp={'status' : None ,"data" : None,'time' : None}
        self.dis_traveld={'status' : None ,"data" : None,'time' : None}
        
        self.sternAngle={'status' : None ,"data" : None,'time' : None}
        self.avg_Stater_Crntp={'status' : None ,"data" : None,'time' : None}
        self.batterycurrent={'status' : None ,"data" : None,'time' : None}
        self.drive_dirctn={'status' : None ,"data" : {'forward' : None, 'reverse': None, 'seat_s' : None},'time' : None}
        self.batteryvoltage={'status' : None ,"data" : None,'time' : None}
        self.batterysoc={'status' : None ,"data" : None,'time' : None}
        self.batteryamphours={'status' : None ,"data" : None,'time' : None}
        self.batteryhighesttemperature={'status' : None ,"data" : None,'time' : None}
        self.batterylowesttemperature={'status' : None ,"data" : None,'time' : None}
        self.batteryaveragetemperature={'status' : None ,"data" : None,'time' : None}
        
        
        self.torque={'status' : None ,"data" : None,'time' : None}
        self.motor_duty={'status' : None ,"data" : None,'time' : None}
        self.current={'status' : None ,"data" : None,'time' : None}
        self.supplyvoltage={'status' : None ,"data" : None,'time' : None}
        self.switch_position={'status' : None ,"data" : None,'time' : None}
        self.box_temperature={'status' : None ,"data" : None,'time' : None}
        self.torqueA={'status' : None ,"data" : None,'time' : None}
        self.torqueB={'status' : None ,"data" : None,'time' : None}
        

class Can_connection():
    def __init__(self,can_channel):
        self.can_channel=can_channel
    def can_connection_check(self):
        return_val={'status':None,"msg":None,'time':None}
        try: 
            self.bus = can.interface.Bus(bustype="socketcan", channel=self.can_channel, bitrate= 250000)
            return_val['status']=True
            return_val['msg']='succesfully connected'
            # return_val['time']=rospy.get_time()
        except Exception as e:
            return_val['status']=False
            return_val['msg']='Error --' + str(e)
            # return_val['time']=rospy.get_time()
        return return_val


    def to_little(self,val):
        little_hex = bytearray.fromhex(val)
        little_hex.reverse()
        # print("Byte array format:", little_hex)
        str_little = ''.join(format(x, '02x') for x in little_hex)
        return str_little

    def twos_comp(self, val, bits):
        if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
            val = val - (1 << bits)        # compute negative value
        return val 


        

        

class Motor(Thread,Main_variables,Can_connection):
    def __init__(self,can_channel='vcan0'):
        '''
        input:
        can_channel - can0 or can1 or vcan0
        output:
        vehicle info about all the motor related stuff
        '''
        self.can_channel=can_channel
        self.msg1,self.msg2=0,0
        Thread.__init__(self)
        Main_variables.__init__(self)
        Can_connection.__init__(self,can_channel=self.can_channel)
    def __call__(self):
        if 'can' not in self.can_channel:
            self.return_val['status'] = False
            self.return_val['msg']='Invalid can_channel name, Try entering aproprate can channel'
            self.return_val['time']=time()
            return self.return_val
        
        val=self.can_connection_check() 
        if val['status'] == False:
            self.return_val['status'] = False
            self.return_val['msg'] = val['msg']
            self.return_val['time'] = time()
            return  self.return_val
        
        Thread(target=self.f1).start()
        Thread(target=self.f2).start()
        Thread(target=self.f3).start()
        Thread(target=self.f4).start()
        Thread(target=self.f5).start()
        print('Threads started')
        
    
    def f1(self):
        while True:
            
            msg=self.bus.recv()
            if msg.arbitration_id==0x292:
                steering=bytearray(msg.data)
                he= steering.hex()
                op = he[2:6]
                self.angle_feedback= int(op,16)/4
                self.sternAngle['status']=True
                self.sternAngle['data']=self.angle_feedback
                self.sternAngle['time']=time()

    
    def f2(self):
        while True:
           msg2=self.bus.recv()
           if msg2.arbitration_id==0x202:
                a=bytearray(msg2.data)
                he=a.hex() #converting bytearray to hex format

                fg= self.to_little(he[0:8])
                self.propspeedlmt_val=int(fg,16)
                self.propspeedlmt_val= self.twos_comp(self.propspeedlmt_val,32)
                self.propspeedlmt['status'] = True
                self.propspeedlmt['data'] = self.propspeedlmt_val
                self.propspeedlmt['time'] = time()

                gh= self.to_little(he[8:])
                self.MotorRPM_val=int(gh,16)
                self.MotorRPM_val= self.twos_comp(self.MotorRPM_val,32)
                self.MotorRPM['status'] = True
                self.MotorRPM['data'] = self.MotorRPM_val
                self.MotorRPM['time'] = time()

               
    def f3(self):
        while True:
            
            msg3=self.bus.recv()
            if msg3.arbitration_id==0x201:             
                a=bytearray(msg3.data)
                he=a.hex() #converting bytearray to hex format
                ccd=self.to_little(he[0:4])
                self.avg_Stater_Crnt_val=int(ccd,16)
                self.avg_Stater_Crnt_val= self.twos_comp(self.avg_Stater_Crnt_val,16)
                self.avg_Stater_Crntp['status'] = True
                self.avg_Stater_Crntp['data'] = self.avg_Stater_Crnt_val
                self.avg_Stater_Crntp['time'] = time()

                ab=self.to_little(he[4:8])
                self.avgMtr_PhaseV_val=int(ab,16)
                self.avgMtr_PhaseV_val= self.twos_comp(self.avgMtr_PhaseV_val,16)*0.0625 
                self.avgMtr_PhaseV['status'] = True
                self.avgMtr_PhaseV['data'] = self.avgMtr_PhaseV_val
                self.avgMtr_PhaseV['time'] =time()

                de=self.to_little(he[12:16])
                self.tgt_torq_val=int(de,16)
                self.tgt_torq_val= self.twos_comp(self.tgt_torq_val,16)*0.0625
                self.tgt_torq['status'] = True
                self.tgt_torq['data'] = self.tgt_torq_val
                self.tgt_torq['time'] = time()

                ef= self.to_little(he[8:12])
                self.mtractl_torq_val=int(ef,16)
                self.mtractl_torq_val= self.twos_comp(self.mtractl_torq_val,16)*0.0625 
                self.mtractl_torq['status'] = True
                self.mtractl_torq['data'] = self.mtractl_torq_val
                self.mtractl_torq['time'] =time()

           
    def f4(self):
        while True:
            
            msg4=self.bus.recv()
            if msg4.arbitration_id==0x203:
                a=bytearray(msg4.data)
                he=a.hex() #converting bytearray to hex format

                hi=self.to_little(he[0:4])
                self.calbattery_crnt_val=int(hi,16)
                self.calbattery_crnt_val=self.twos_comp(self.calbattery_crnt_val,16)*0.0625
                self.calbattery_crnt['status'] = True
                self.calbattery_crnt['data'] = self.calbattery_crnt_val
                self.calbattery_crnt['time'] =time()


                ij=self.to_little(he[4:8])
                self.ctrlcapctr_v_val=int(ij,16)*0.0625
                self.ctrlcapctr_v['status'] = True
                self.ctrlcapctr_v['data'] = self.ctrlcapctr_v_val
                self.ctrlcapctr_v['time'] =time()


                jk=self.to_little(he[8:12])
                self.trottleip_val=int(jk,16)
                self.trottleip_val=self.twos_comp(self.trottleip_val,16)*0.00390625
                self.trottleip['status'] = True
                self.trottleip['data'] = self.trottleip_val
                self.trottleip['time'] =time()
    def f5(self):
        while True:
            msg5=self.bus.recv()
            if msg5.arbitration_id==0x204:
                # print("------------------------------------------------------------------")

                a=bytearray(msg5.data)
                he=a.hex() #converting bytearray to hex format avg_Stater_Crnt

                kl=self.to_little(he[0:4])
                self.mtr_temp_val=int(kl,16)
                self.mtr_temp_val=self.twos_comp(self.mtr_temp_val,16)
                self.mtr_temp['status'] = True
                self.mtr_temp['data'] = self.mtr_temp_val
                self.mtr_temp['time'] =time()

                self.ctrlr_temp_val=int(he[4:6],16)
                self.ctrlr_temp_val=self.twos_comp(self.ctrlr_temp_val,8)
                self.ctrlr_temp['status'] = True
                self.ctrlr_temp['data'] = self.ctrlr_temp_val
                self.ctrlr_temp['time'] =time()

                lm=self.to_little(he[6:14])
                self.dis_traveld_val=int(lm,16)
                self.dis_traveld_val=self.dis_traveld_val*0.00390625
                self.dis_traveld['status'] = True
                self.dis_traveld['data'] = self.dis_traveld_val
                self.dis_traveld['time'] =time()


                mn=self.to_little(he[14:16])
                self.drive_dirctn_val=int(mn,16)
                self.drive_dirctn['status']=True
                if self.drive_dirctn_val==0:
                    self.drive_dirctn['data']['forward']=0
                    self.drive_dirctn['data']['reverse']=0
                    self.drive_dirctn['data']['seat_s']=0
                    self.drive_dirctn['time']=time()

                   
                elif self.drive_dirctn_val==1:
                    self.drive_dirctn['data']['forward']=1
                    self.drive_dirctn['data']['reverse']=0
                    self.drive_dirctn['data']['seat_s']=0
                    self.drive_dirctn['time']=time()
            
                elif self.drive_dirctn_val==2:
                    self.drive_dirctn['data']['forward']=0
                    self.drive_dirctn['data']['reverse']=1
                    self.drive_dirctn['data']['seat_s']=0
                    self.drive_dirctn['time']=time()
                 
                elif self.drive_dirctn_val==4:
                    self.drive_dirctn['data']['forward']=0
                    self.drive_dirctn['data']['reverse']=0
                    self.drive_dirctn['data']['seat_s']=1
                    self.drive_dirctn['time']=time()
                
                elif self.drive_dirctn_val==5:
                    self.drive_dirctn['data']['forward']=1
                    self.drive_dirctn['data']['reverse']=0
                    self.drive_dirctn['data']['seat_s']=1
                    self.drive_dirctn['time']=time()

                elif self.drive_dirctn_val==6:
                    self.drive_dirctn['data']['forward']=0
                    self.drive_dirctn['data']['reverse']=1
                    self.drive_dirctn['data']['seat_s']=1
                    self.drive_dirctn['time']=time()

                else:
                    pass

        
        





class Battery(Thread,Main_variables,Can_connection):
    def __init__(self,can_channel='vcan0'):
        '''
        input:
        can_channel - can0 or can1 or vcan0
        output:
        vehicle info about all the motor related stuff
        '''
        self.can_channel=can_channel
        self.msg1,self.msg2=0,0
        Thread.__init__(self)
        Main_variables.__init__(self)
        Can_connection.__init__(self,can_channel=self.can_channel)
    def __call__(self):
        if 'can' not in self.can_channel:
            self.return_val['status'] = False
            self.return_val['msg']='Invalid can_channel name, Try entering aproprate can channel'
            self.return_val['time']=time()
            return self.return_val
        
        val=self.can_connection_check() 
        if val['status'] == False:
            self.return_val['status'] = False
            self.return_val['msg'] = val['msg']
            self.return_val['time'] = time()
            return  self.return_val
        
        Thread(target=self.batteryf1).start()
        Thread(target=self.batteryf2).start()
        Thread(target=self.batteryf3).start()
        
        print('Threads started')
    def batteryf1(self):
        while True:
            
            r=self.bus.recv()
            if r.arbitration_id==0x182:
                self.a=bytearray(r.data)
                self.he=self.a.hex() #converting bytearray to hex format

                aq= self.to_little(self.he[0:4])
                self.batterycurrent=int(aq,16)
                self.batterycurrent= self.twos_comp(self.batterycurrent,16)*0.1
                
                self.batterycurrent['status']=True
                self.batterycurrent['data']=self.batterycurrent
                self.batterycurrent['time']=time()
                ar= self.to_little(self.he[4:8])
                self.batteryvoltage=int(ar,16)*0.1
                self.batteryvoltage['status']=True
                self.batteryvoltage['data']=self.batteryvoltage
                self.batteryvoltage['time']=time()

    def batteryf2(self):
        while True:
            msg6=self.bus.recv()
            if msg6.arbitration_id==0x183:
                self.a=bytearray(msg6.data)
                self.he=self.a.hex() #converting bytearray to hex format

                _as= self.to_little(self.he[0:2])
                self.batterysoc=int(_as,16)*0.5
                self.batterysoc['status']=True
                self.batterysoc['data']=self.batterysoc
                self.batterysoc['time']=time()
                at= self.to_little(self.he[2:6])
                self.batteryamphours=int(at,16)*0.1
                self.batteryamphours['status']=True
                self.batteryamphours['data']=self.batteryamphours
                self.batteryamphours['time']=time()

                au= self.to_little(self.he[6:8])
                self.batteryhighesttemperature=int(au,16)
                self.batteryhighesttemperature=self.twos_comp(self.batteryhighesttemperature,8)*1
                self.batteryhighesttemperature['status']=True
                self.batteryhighesttemperature['data']=self.batteryhighesttemperature
                self.batteryhighesttemperature['time']=time()
                av=self.to_little(self.he[8:10])
                self.highesttemperaturethermistorid=int(av,16)*1
                
                aw=self.to_little(self.he[10:12])
                self.batterylowesttemperature=int(aw,16)
                self.batterylowesttemperature=self.twos_comp(self.batterylowesttemperature,8)*1
                self.batterylowesttemperature['status']=True
                self.batterylowesttemperature['data']=self.batterylowesttemperature
                self.batterylowesttemperature['time']=time()
                ax=self.to_little(self.he[12:14])
                self.highestlowestthermistorid=int(ax,16)*1

                ay=self.to_little(self.he[14:16])
                self.batteryaveragetemperature=int(ay,16)
                self.batteryaveragetemperature=self.twos_comp(self.batteryaveragetemperature,8)*1
                self.batteryaveragetemperature['status']=True
                self.batteryaveragetemperature['data']=self.batteryaveragetemperature
                self.batteryaveragetemperature['time']=time()
    def batteryf3(self):
        while True:
            msg7=self.bus.recv()
            if msg7.arbitration_id==0x184:
                self.a=bytearray(msg7.data)
                self.he=self.a.hex()

                az= self.to_little(self.he[0:2])   
                self.bmstemperature=int(az,16)
                self.bmstemperature=self.twos_comp(self.bmstemperature,8)*1
                self.bmstemperature['status']=True
                self.bmstemperature['data']=self.bmstemperature
                self.bmstemperature['time']=time()
                ba= self.to_little(self.he[2:4])
                self.batteryadaptivesoc=int(ba,16)*0.5
                self.batteryadaptivesoc['status']=True
                self.batteryadaptivesoc['data']=self.batteryadaptivesoc
                self.batteryadaptivesoc['time']=time()
                bb=self.to_little(self.he[4:8])
                self.batteryadaptiveamphours=int(bb,16)*0.1
                self.batteryadaptiveamphours['status']=True
                self.batteryadaptiveamphours['data']=self.batteryadaptiveamphours
                self.batteryadaptiveamphours['time']=time()
                bc=self.to_little(self.he[8:12])
                self.auxilarybatteryvoltage=int(bc,16)*0.1
                self.batteryaveragetemperature['status']=True
                self.batteryaveragetemperature['data']=self.batteryaveragetemperature
                self.batteryaveragetemperature['time']=time()


    
class Steering(Thread,Main_variables,Can_connection):
    def __init__(self,can_channel='vcan0'):
        '''
        input:
        can_channel - can0 or can1 or vcan0
        output:
        vehicle info about all the motor related stuff
        '''
        self.can_channel=can_channel
        self.msg1,self.msg2=0,0
        Thread.__init__(self)
        Main_variables.__init__(self)
        Can_connection.__init__(self,can_channel=self.can_channel)
    def __call__(self):
        if 'can' not in self.can_channel:
            self.return_val['status'] = False
            self.return_val['msg']='Invalid can_channel name, Try entering aproprate can channel'
            self.return_val['time']=time()
            return self.return_val
        
        val=self.can_connection_check() 
        if val['status'] == False:
            self.return_val['status'] = False
            self.return_val['msg'] = val['msg']
            self.return_val['time'] = time()
            return  self.return_val

        Thread(target=self.steeringf1).start()
        
        

    def steeringf1(self):
        while True:
            msg8=self.bus.recv()
            if msg8.arbitration_id==0x290:
                self.steering=bytearray(self.r.data)
                self.he= self.steering.hex()

                oq= self.he[0:2]
                self.torque= int(oq,16)
                self.torque['status']=True
                self.torque['data']=self.torque
                self.torque['time']=time()


                os= self.he[2:4]
                self.motor_duty = int(os,16)
                self.motor_duty['status']=True
                self.motor_duty['data']=self.motor_duty
                self.motor_duty['time']=time()

                ot= self.he[4:6]
                self.current = int(ot,16)
                self.current['status']=True
                self.current['data']=self.current
                self.current['time']=time()

                ou = self.he[6:8]
                self.supplyvoltage = int(ou,16)
                self.supplyvoltage['status']=True
                self.supplyvoltage['data']=self.supplyvoltage
                self.supplyvoltage['time']=time()

                ov = self.he[8:10]
                self.switch_position = int(ov,16)
                self.switch_position['status']=True
                self.switch_position['data']=self.switch_position
                self.switch_position['time']=time()


                ow = self.he[10:12]
                self.box_temperature = int(ow,16)
                self.box_temperature['status']=True
                self.box_temperature['data']=self.box_temperature
                self.box_temperature['time']=time()


                ox = self.he[12:14]
                self.torqueA = int(ox,16)
                self.torqueA['status']=True
                self.torqueA['data']=self.torqueA
                self.torqueA['time']=time()



                oy = self.he[14:16]
                self.torqueB = int(oy,16)
                self.torqueB['status']=True
                self.torqueB['data']=self.torqueB
                self.torqueB['time']=time()




    
class accessaries(Thread):
    pass

if __name__=="__main__":
    print("f")
    m=Motor()
    b=Battery()
    s=Steering()
    print(m())
    print(b())
    print(s())
    i=0
    while True:
        sleep(0.2)
        #  print('values I = '+str(m.msg1.Timestamp:)+" J ="+str(m.msg2))
        print('RPM -',m.MotorRPM)
        print("ANGLE -",m.sternAngle)
        print('avg_Stater_Crnt -',m.avg_Stater_Crntp)
        print('avgMtr_PhaseV -',m.avgMtr_PhaseV)
        print('drive_dirctn -',m.drive_dirctn)
        print('mtr_temp -',m.mtr_temp)
        
        i+=1
        # if i >10:
        #     break
        

        # self.avg_Stater_Crnt={'status' : None ,"data" : None,'time' : None}
        # self.avgMtr_PhaseV={'status' : None ,"data" : None,'time' : None}
        # self.tgt_torq

        
    # m=Thread()
    # print(dir(m))
    # print('b')
    
    
