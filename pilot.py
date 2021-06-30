try: 
    import rospy
    from vehicle_info import Motor,Battery,steerig
    import time
except Exception as e:
    print(e)
    exit()

m=Motor('vcan0')

print(m())


class Motor_control():
    def __init__(self,can_channel='vcan0',mode=''):
        '''
        inputs: 
          can_channel = channel on which can bus is runnig
          mode        = whether voltage control or rpm control

        '''
        pass
    def sanity_check(self):
        '''
        Inputs  : None
        fun     : It check all the nessesary conditions, if every thing is okay then it would replace dictionary with True in it else False along with what caused to get failed 
        Output  : An usal dic , {status: Bool,
                                 msg    : messege
                                 time   : time at this fuction had excecuted},  if status True then we can do other operaions else we can not.
           
        '''
        pass
    def trottle_send(self,value):
        '''
        Input   : Value to be sent to motor controller or DTU, it can be RPM or voltage, depnding on which mode we are in
        fun     : 
        '''


while True:
    time.sleep(0.1)
    print("----------------------------------------")

    print('RPM -',m.MotorRPM)
    print("ANGLE -",m.sternAngle)

    print("return_val - ",m.return_val)
    print("avg_Stater_Crnt - ",m.avg_Stater_Crnt)
    print("avgMtr_PhaseV - ",m.avgMtr_PhaseV)
    print("tgt_torq - ",m.tgt_torq)
    print("mtractl_torq - ",m.mtractl_torq)
    print("propspeedlmt - ",m.propspeedlmt)
    print("MotorRPM - ",m.MotorRPM)

    print("calbattery_crnt - ",m.calbattery_crnt)
    print("trottleip - ",m.trottleip)
    print("mtr_temp - ",m.mtr_temp)
    print("ctrlr_temp - ",m.ctrlr_temp)
    print("dis_traveld - ",m.dis_traveld)
    print("drive_dirctn - ",m.drive_dirctn)
    print("avg_Stater_Crntp - ",m.avg_Stater_Crntp)
    
