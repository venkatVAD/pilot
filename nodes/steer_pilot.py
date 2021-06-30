try: 
    import rospy
    from threading import Thread
    from vehicle_info import Motor,Battery,Steering,Can_connection
    from numpy import interp
    from time import time
    import can
except Exception as e:
    print(e)
    exit()

m=Motor('vcan0')
b=Battery('vcan0')
s=Steering('vcan0')

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

        self.steering_min = rospy.get_param('vehicle_info/steering/Angle/steering_min','steering_min')
        self.steering_max = rospy.get_param('vehicle_info/steering/Angle/steering_max','steering_max')
        self.steering_centre = rospy.get_param('vehicle_info/steering/Angle/steering_centre','steering_centre')




class Steering_control(Parameters, Motor, Steering, Battery):
    def __init__(self, can_channel='vcan0', mode = 'INCREMENTAL' ):
        '''
            Input :
                can_channel = vcan0 or can0
            mode :
                INCREMENTAL - Increment angle by 10 degree
                ANGLE - send demand angle 
                MOTORDUTY - send motor duty
                SPEED - control speed of steering (cmd_vel/angular.z/
                                                    rad/sec)
        '''
        # super.__init__(self)
        # Parameters.__init__(self)
        self.can_channel = can_channel
        self.mode = mode

        self.sanity_check_result = {'status': None, 'msg': None}
        self.steering_in = {'msg': None, 'time' : None }
        self.failed_res = {'status' : None, 'msg' : None, 'time' : None}
        self.steering_log = {'status' : None, 'msg' : None, 'time' : None}
        self.modes_list = ['INCREMENTAL','ANGLE', 'MOTORDUTY', 'SPEED']

        self.diff = 0

        self.steering_id = 0x298



    def __call__(self):
        if 'can' not in self.can_channel:
            self.failed_res['status'] = False
            self.failed_res['msg'] = 'Input proper can. Either vcan0 or can0'
            self.failed_res['time'] = time()
            return self.failed_res
        mode_check = False
        for m in self.modes_list:
            if m == self.mode:
                mode_check = True
        if not mode_check:
            self.failed_res['status'] = False
            self.failed_res['msg'] = 'Input Proper Mode : INCREMENTAL ,ANGLE, MOTORDUTY, SPEED'        
            self.failed_res['time'] = time()
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

        Thread(target=self.sanity_check).start()
        rospy.timer(rospy.Duration(0.2),self.timer)
        self.failed_res['status'] = True
        self.failed_res['msg'] = "Threading started"
        self.failed_res['time'] = time()
        return self.failed_res

    def timer(self,event):
        if self.sanity_check_result['status'] == False:
            print(self.sanity_check_result['msg'])
            self.logger(self.send_angle(0))
            return 0
        else:
            now = time() - self.steering_in['time']
            now = now * 9600000/10
            
            if now < 1.0:
                self.send_angle(self.steering_in['msg'])
                print(str(self.steering_in['msg']+"sent to the steering"))
            else:
                print("Time out for steering. Its not an error, but a warning to check that the msgs are not been receiving since " +str(now)+'seconds')
                self.logger(self.send_angle(0))

    
    def steering_send(self, value):
        '''
            Input - value respective to its mode
        '''
        self.value = value
        if self.mode == self.modes_list[0]:
            self.diff = self.sternAngle - self.value
            while (abs(self.diff > 10)):
                self.diff = self.sternAngle - self.value
                if self.diff >= 10:
                    self.steering_in['msg'] = self.sternAngle + 10
                    self.steering_in['time'] = time()
                    if self.steering_in['msg'] > self.steering_max:
                        self.steering_in['msg'] = self.steering_max
                elif self.diff <= 10:
                    self.steering_in['msg'] = self.sternAngle - 10
                    self.steering_in['time'] = time()
                    if self.steering_in['msg'] < self.steering_min:
                        self.steering_in['msg'] = self.steering_min
        

        elif self.mode == self.modes_list[1]:
            if self.steering_min < self.value < self.steering_max:
                self.steering_in['msg'] = self.value
                self.steering_in['time'] = time()
            elif self.value > self.steering_max:
                self.steering_in['msg'] = self.value = self.steering_max
                self.steering_in['time'] = time()
            elif self.value < self.steering_min:
                self.steering_in['msg'] = self.value = self.steering_min
                self.steering_in['time'] = time()

        elif self.mode == self.modes_list[2]:
            if self.value >= 0:
                self.value = hex(self.value)
                if self.value >= 0x64:
                    self.value = 0x64
                self.steering_in['msg'] = self.value
                self.steering_in['time'] = time()
            #2's compliment
            elif self.value <= 0:
                self.value = hex(self.value & (2**8-1))
                if self.value <= 0x9C:
                    self.value = 0x9C
                self.steering_in['msg'] = self.value
                self.steering_in['time'] = time()
            '''

        elif self.mode == self.modes_list[2]:
            self.diff = self.sternAngle - self.value
            while (abs(self.diff) >= 10):     #if
                self.diff = self.sternAngle - self.value
                if self.diff >= 0:
                    self.diff = hex(self.diff)
                if self.diff >= 0x64:
                    self.diff = 0x64
                self.steering_in['msg'] = self.diff
                self.steering_in['time'] = time()
            #2's compliment
                if self.diff <= 0:
                    self.diff = hex(self.diff & (2**8-1))
                if self.diff <= 0x9C:
                    self.diff = 0x9C
                self.steering_in['msg'] = self.value
                self.steering_in['time'] = time()

            '''
        

        elif self.mode == self.modes_list[3]:
            self.value = int(interp(self.value),[-5,5],[-100,100])
            self.value = hex(self.value)
            if self.value >= 0x32:
                self.value = 0x32
            elif self.value <= 0x9C:
                self.value = 0x9C
            self.steering_in['msg'] = self.value
            self.steering_in['time'] = time()

    def send_angle(self, steer_value):
        try:
            if steer_value == 0:
                steer_msg=[0X0,0X0,0X0,0X0,0X0,0X0,0X0,0X0]
                steer_can_msg = can.Message(arbitration_id=self.steering_id,
                                data=steer_msg,
                                extended_id=False)
                self.can_connection.bus.send(steer_can_msg)
                self.steering_log['status'] = True
                self.steering_log['msg'] = 'sending zeros to the steering'
                self.steering_log['time'] = time()


            elif steer_value != 0 and (self.mode == self.modes_list[0] or self.modes_list[1]):
                steer_value = int(hex(int(steer_value))[2:],16)
                steer_msg=[0X02,steer_value,0X0,0X0,0X0,0X0,0X0,0X0]
                steer_can_msg = can.Message(arbitration_id=self.steering_id,
                                data=steer_msg,
                                extended_id=False)
                self.can_connection.bus.send(steer_can_msg)
                self.steering_log['status'] = True
                self.steering_log['msg'] = 'Angle of' +str(steer_value) + 'sent to steering'
                self.steering_log['time'] = time()
            

            elif steer_value != 0 and self.mode == self.modes_list[2]:
                motor_duty = steer_value
                steer_msg=[0X01,0x00,motor_duty,0X0,0X0,0X0,0X0,0X0]
                steer_can_msg = can.Message(arbitration_id=self.steering_id,
                                data=steer_msg,
                                extended_id=False)
                self.can_connection.bus.send(steer_can_msg)
                self.steering_log['status'] = True
                self.steering_log['msg'] = 'Motor Duty' + str(motor_duty) + 'sent to steering for' + str(time())+'seconds'
                self.steering_log['time'] = time()
            return self.steering_log

        except Exception as e:
            self.steering_log['status'] = False
            self.steering_log['msg'] = 'Failed' + 'Error -' + str(e) 
            self.steering_log['time'] = time()
            return self.steering_log
        

    def logger(self,logs):
        if logs['status'] != True:
            rospy.logwarn(logs['msg'])
        else:
            rospy.info(logs['msg'])

    def sanity_check(self):

        if self.sternAngle != None:
            pass


if __name__ == '__main__':
    s = Steering_control(can_channel='vcan0', mode='INCREMENTAL')
    ret_vel = s()
    if ret_vel['status'] == False:
        print(ret_vel['msg'])
    else:
        s.steering_send(50)


