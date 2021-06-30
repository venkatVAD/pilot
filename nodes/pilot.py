try: 
    import rospy
    from threading import Thread
    from vehicle_info import Motor,Battery,Steering,Can_connection
    from time import time,sleep
    import can
except Exception as e:
    print(e)
    exit()


class Parameters():
    def __init__(self):
        '''
        inputs: 
          can_channel = channel on which can bus is runnig
          mode        = whether voltage control or rpm control
        '''
        self.avg_starter_Crnt_min=rospy.get_param('vechile_info/Motor/avg_starter_Crnt/avg_starter_Crnt_min','avg_starter_Crnt_min')
        self.avg_starter_Crnt_max=rospy.get_param('vechile_info/Motor/avg_starter_Crnt/avg_starter_Crnt_max','avg_starter_Crnt_max')

        self.avg_starter_Crnt_min=rospy.get_param('vechile_info/Motor/avgMtr_phaseV/avgMtr_phaseV_min','avgMtr_phaseV_min')
        self.avg_starter_Crnt_max=rospy.get_param('vechile_info/Motor/avgMtr_phaseV/avgMtr_phaseV_max','avgMtr_phaseV_max')

        self.tgt_torq_min=rospy.get_param('vechile_info/Motor/tgt_torq/tgt_torq_min','tgt_torq_min')
        self.tgt_torq_max=rospy.get_param('vechile_info/Motor/tgt_torq/tgt_torq_max','tgt_torq_max')

        self.mtractl_torq_min=rospy.get_param('vechile_info/Motor/mtractl_torq/mtractl_torq_min','mtractl_torq_min')
        self.mtractl_torq_max=rospy.get_param('vechile_info/Motor/mtractl_torq/mtractl_torq_max','mtractl_torq_max')

        self.propspeedlmt_min=rospy.get_param('vechile_info/Motor/propspeedlmt/propspeedlmt_min','propspeedlmt_min')
        self.propspeedlmt_max=rospy.get_param('vechile_info/Motor/propspeedlmt/propspeedlmt_max','propspeedlmt_max')

        self.MotorRPM_min=rospy.get_param('vechile_info/Motor/MotorRPM/MotorRPM_min','MotorRPM_min')
        self.MotorRPM_max=rospy.get_param('vechile_info/Motor/MotorRPM/MotorRPM_max','MotorRPM_max')

        self.calbattery_crnt_min=rospy.get_param('vechile_info/Motor/calbattery_crnt/calbattery_crnt_min','calbattery_crnt_min')
        self.calbattery_crnt_max=rospy.get_param('vechile_info/Motor/calbattery_crnt/calbattery_crnt_max','calbattery_crnt_max')

        self.ctrlcapctr_v_min=rospy.get_param('vechile_info/Motor/ctrlcapctr_v/ctrlcapctr_v_min','ctrlcapctr_v_min')
        self.ctrlcapctr_v_max=rospy.get_param('vechile_info/Motor/ctrlcapctr_v/ctrlcapctr_v_max','ctrlcapctr_v_max')

        self.trottleip_min=rospy.get_param('vechile_info/Motor/trottleip/trottleip_min','trottleip_min')
        self.trottleip_max=rospy.get_param('vechile_info/Motor/trottleip/trottleip_max','trottleip_max')

        self.mtr_temp_min=rospy.get_param('vechile_info/Motor/mtr_temp/mtr_temp_min','mtr_temp_min')
        self.mtr_temp_max=rospy.get_param('vechile_info/Motor/mtr_temp/mtr_temp_max','mtr_temp_max')

        self.ctrlr_tmrp_min=rospy.get_param('vechile_info/Motor/ctrlr_tmrp/ctrlr_tmrp_min','ctrlr_tmrp_min')
        self.ctrlr_tmrp_max=rospy.get_param('vechile_info/Motor/ctrlr_tmrp/ctrlr_tmrp_max','ctrlr_tmrp_max')

        self.dis_traveld=rospy.get_param('vechile_info/Motor/dis_traveld','dis_traveld')

        self.avg_starter_Crntp_min=rospy.get_param('vechile_info/Motor/Battery/avg_starter_Crntp_min','avg_starter_Crntp_min')
        self.avg_starter_Crntp_max=rospy.get_param('vechile_info/Motor/Battery/avg_starter_Crntp_max','avg_starter_Crntp_max')

        self.batterycurrent_min=rospy.get_param('vechile_info/Motor/Battery/batterycurrent_min','batterycurrent_min')
        self.batterycurrent_max=rospy.get_param('vechile_info/Motor/Battery/batterycurrent_max','batterycurrent_max')

        self.drive_dirctn_min=rospy.get_param('vechile_info/Motor/Battery/drive_dirctn_min','drive_dirctn_min')
        self.drive_dirctn_max=rospy.get_param('vechile_info/Motor/Battery/drive_dirctn_max','drive_dirctn_max')

        self.batteryvoltage_min=rospy.get_param('vechile_info/Motor/Battery/batteryvoltage_min','batteryvoltage_min')
        self.batteryvoltage_max=rospy.get_param('vechile_info/Motor/Battery/batteryvoltage_max','batteryvoltage_max')

        self.batterysoc_min=rospy.get_param('vechile_info/Motor/Battery/batterysoc_min','batterysoc_min')
        self.batterysoc_max=rospy.get_param('vechile_info/Motor/Battery/batterysoc_max','batterysoc_max')

        self.batteryamphours_min=rospy.get_param('vechile_info/Motor/Battery/batteryamphours_min','batteryamphours_min')
        self.batteryamphours_max=rospy.get_param('vechile_info/Motor/Battery/batteryamphours_max','batteryamphours_max')

        self.batteryhighesttemperature_min=rospy.get_param('vechile_info/Motor/Battery/batteryhighesttemperature_min','batteryhighesttemperature_min')
        self.batteryhighesttemperature_max=rospy.get_param('vechile_info/Motor/Battery/batteryhighesttemperature_max','batteryhighesttemperature_max')

        self.batterylowesttemperature_min=rospy.get_param('vechile_info/Motor/Battery/batterylowesttemperature_min','batterylowesttemperature_min')
        self.batterylowesttemperature_max=rospy.get_param('vechile_info/Motor/Battery/batterylowesttemperature_max','batterylowesttemperature_max')

        self.batteryaveragetemperature_min=rospy.get_param('vechile_info/Motor/Battery/batteryaveragetemperature_min','batteryaveragetemperature_min')
        self.batteryaveragetemperature_max=rospy.get_param('vechile_info/Motor/Battery/batteryaveragetemperature_max','batteryaveragetemperature_max')

        self.batteryaveragetemperature_min=rospy.get_param('vechile_info/Motor/Battery/batteryaveragetemperature_min','batteryaveragetemperature_min')
        self.batteryaveragetemperature_max=rospy.get_param('vechile_info/Motor/Battery/batteryaveragetemperature_max','batteryaveragetemperature_max')

        self.torque_min=rospy.get_param('vechile_info/steering/torque/torque_min','torque_min')
        self.torque_max=rospy.get_param('vechile_info/steering/torque/torque_max','torque_max')

        self.motor_duty_min=rospy.get_param('vechile_info/steering/motor_duty/motor_duty_min','motor_duty_min')
        self.motor_duty_max=rospy.get_param('vechile_info/steering/motor_duty/motor_duty_max','motor_duty_max')

        self.current_min=rospy.get_param('vechile_info/steering/current/current_min','current_min')
        self.current_max=rospy.get_param('vechile_info/steering/current/current_max','current_max')

        self.supplyvoltage_min=rospy.get_param('vechile_info/steering/supplyvoltage/supplyvoltage_min','supplyvoltage_min')
        self.supplyvoltage_max=rospy.get_param('vechile_info/steering/supplyvoltage/supplyvoltage_max','supplyvoltage_max')

        self.switch_position_min=rospy.get_param('vechile_info/steering/switch_position/switch_position_min','switch_position_min')
        self.switch_position_max=rospy.get_param('vechile_info/steering/switch_position/switch_position_max','switch_position_max')

        self.box_temperature_min=rospy.get_param('vechile_info/steering/box_temperature/box_temperature_min','box_temperature_min')
        self.box_temperature_max=rospy.get_param('vechile_info/steering/box_temperature/box_temperature_max','box_temperature_max')

        self.torqueA_min=rospy.get_param('vechile_info/steering/torqueA/torqueA_min','torqueA_min')
        self.torqueA_max=rospy.get_param('vechile_info/steering/torqueA/torqueA_max','torqueA_max')

        self.torqueB_min=rospy.get_param('vechile_info/steering/torqueB/torqueB_min','torqueB_min')
        self.torqueB_max=rospy.get_param('vechile_info/steering/torqueB/torqueB_max','torqueB_max')


class Motor_control(Parameters,Motor,Battery,Steering,Can_connection):
    def __init__(self,can_channel='vcan0',mode='RPM',direction ='F' ):
        """
        input : 
           can channel 
           mode -> mode of control
                RPM     -> we would send RPM command to vehicle 
                VOLTAGE -> we would send voltage command to vehicle
                SPEED   -> we would send speed command to vehilcle (m/s)
            direction -> direction whether forward or reverse or both
                F -> Forward
                R -> Reverse
                B -> Both
                 
        """
        # super.__init__(self)
        Parameters.__init__(self)
        # Motor.__init__(self)


        
        self.can_channel = can_channel
        
        self.mode = mode 

        

        self.sanity_check_result = {'status' : True ,"msg" : None,'time' : time()}
        self.trottle_in = {'data' : None , 'time' : time()}
        self.failed_res =  {'status' : None ,"msg" : None,'time' : None}
        self.trottle_log = {'status' : None ,"msg" : None,'time' : None}
        self.modes_list=["RPM","SPEED","VOLTAGE"]
        self.direc_list = ['F','R','B']
        self.trottle_id=0X510
        self.initial = True

    def __call__(self):
        if 'can' not in self.can_channel:
            self.failed_res['status'] = False
            self.failed_res['msg'] = 'can channel is not approiate please provide proper name, examples - can0, vcan0'
            self.failed_res['time']=time()
            return self.failed_res
        
        mode_cheak=False
        for m in self.modes_list:
            if m == self.mode:
                mode_cheak=True
        if not mode_cheak:
            self.failed_res['status'] = False
            self.failed_res['msg'] = 'The mode you have specified is invalid enter -> RPM , SPEED, VOLTAGE'
            self.failed_res['time']=time()
            return self.failed_res
        self.can_connection=Can_connection(self.can_channel)
        can_status= self.can_connection.can_connection_check()
        if not can_status['status']:
            self.failed_res['status'] = False
            self.failed_res['msg'] = can_status['msg']
            self.failed_res['time']=time()
            return self.failed_res

        self.v_info=Motor(self.can_channel)
        ret_val=self.v_info()
        if ret_val['status'] == False:
            self.failed_res['status'] = False
            self.failed_res['msg'] = ret_val['msg'] 
            self.failed_res['time']=time()
            return self.failed_res
        else:
            print(" Succesfully lanched vehicle information")
        print("RPM", self.v_info.MotorRPM)

        Thread(target=self.sanity_check).start()
        rospy.Timer(rospy.Duration(0.1),self.timer)
        self.failed_res['status'] = True
        self.failed_res['msg'] = 'Started sanity check on vehicle'
        self.failed_res['time']=time()
        return self.failed_res
        


    def timer(self,event):
        print("RPM", self.v_info.MotorRPM)
       

        if self.initial :
            on_st = self.on_start()
            if  on_st:
                self.initial = False
            else:
                print("motor is moving not starting trottle commands ")
            self.logger(self.send_rpm(-1))
            return 0
        if self.sanity_check_result['status'] == False or self.sanity_check_result['status'] == None:
            print(" sanity_check failed")
            print(self.sanity_check_result['msg'])
            self.logger(self.send_rpm(-1))
            return 0
        else:
            
            now = time()-self.trottle_in['time']
            now = int(now)* 1000
            
            if now < 1000 :
                # self.send_rpm(self.trottle_in['data'])
                self.logger(self.send_rpm(self.trottle_in['data'])) 
                

            else: 
                print('Time out from trottle send, There is no update from last '+ str(now) + ' seconds')
                self.logger(self.send_rpm(-1)) 


            
    def send_rpm(self,rpm_value):
        
            try : 
                if rpm_value == -1:
                    trotle_msg=[0X0,0X0,0X0,0X0,0X0,0X0,0X0,0X0]
                    trotle_can_msg = can.Message(arbitration_id=self.trottle_id,
                                                data=trotle_msg,
                                                extended_id=False)
                    self.can_connection.bus.send(trotle_can_msg)
                    self.trottle_log['status'] = True
                    self.trottle_log['msg'] = "sent manual mode msg to motor"
                    self.trottle_log['time'] = time()
                
                else:
                    hex_value=hex(rpm_value)[2:]
                    d1,d2=self.hex_to_sub_hex(hex_value)
                    
                    direction=0X1 # for forward , direction=0X2 for reverse
                    msg=[d2,d1,direction,0X0,0X0,0X0,0X0,0X0]
                    can_msg = can.Message(arbitration_id=self.trottle_id,
                                        data=msg,
                                    extended_id=False)
                    self.can_connection.bus.send(can_msg)
                    self.trottle_log['status'] = True
                    self.trottle_log['msg'] = 'rpm of '+str(rpm_value) +' sent to motor'
                    self.trottle_log['time'] = time()

            
                return self.trottle_log
            except Exception as e:
                self.trottle_log['status'] = False
                self.trottle_log['msg'] = 'Failed '+'Error - '+str(e)
                return self.trottle_log
 

    def hex_to_sub_hex(self,hex_value):
        if len(hex_value)==4:
            d1=hex_value[:2]
            d2=hex_value[2:]
        elif len(hex_value)==3:
            d1='0'+hex_value[0]
            d2=hex_value[1:]
        elif len(hex_value)==2:
            d1='00'
            d2=hex_value
        elif len(hex_value)==1:
            d1='00'
            d2='0'+hex_value
        else:
            d1='00'
            d2='00'
        d1 = int(d1, 16)
        d2 = int(d2, 16)
        return d1,d2
    
    def logger(self,logs):
        if logs['status'] != True:
            rospy.logwarn(logs['msg'])
        else:
            rospy.loginfo(logs['msg'])
        

    def sanity_check(self):
        '''
        Inputs  : None
        fun     : It check all the nessesary conditions, if every thing is okay then it would replace dictionary with True in it else False along with what caused to get failed 
        Output  : An usal dic , {status: Bool,
                                 msg    : messege
                                 time   : time at this fuction had excecuted},  if status True then we can do other operaions else we can not.
           
        '''
        # if self.v_info.MotorRPM > 30000:
        #     self.sanity_check_result['status'] = False
        #     self.sanity_check_result['msg'] = 'RPM is high '
        #     self.sanity_check_result['time'] = time() 
        # if self.v_info.mtr_temp > 60 :
       
       
       
        #     self.sanity_check_result['status'] = False
        #     self.sanity_check_result['msg'] = ' motor temparatur is hight '
        #     self.sanity_check_result['time'] = time() 
        pass
            


    def on_start(self):
        print("on start")
        val = None

        if self.v_info.MotorRPM['data'] != None and self.v_info.MotorRPM['data'] == 0: 
            val = True
        else: 
            val = False
        print(val)
        return val

            


    def trottle_send(self,value):
        '''
        Input   : Value to be sent to motor controller or DTU, it can be RPM or voltage, depnding on which mode we are in
        fun     : 
        '''
        self.value = value
        if self.mode == self.modes_list[0]:
            if self.value > 0 and self.value < 3000 :
                self.trottle_in['data'] =  self.value
                self.trottle_in['time'] = time()







if __name__=='__main__':
    rospy.init_node("pilot")
    
    m=Motor_control(can_channel='can0',mode='RPM', direction='F')
    ret_val=m()

    if ret_val['status'] == False:
        print(ret_val['msg'])
        exit()
    else:
        print(ret_val['msg'])
    while True:
         m.trottle_send(1000)
         sleep(0.2)


