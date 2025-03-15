from pytrinamic.connections import ConnectionManager
from pytrinamic.modules import TMCM1260
from serial import Serial
from time import sleep
from time import time

start_deviation_error=0

class Stepper:
    def __init__(self,interface,module_id,max_acc,max_velocity,V1):
        global start_deviation_error
        if interface=="can":
            communication="socketcan_tmcl"
        elif interface=="usb":
            communication="usb_tmcl"
        else:
            raise ValueError(f"Cannot use {interface} comunication, possible values are 'can' or 'usb'")

        self.interface=ConnectionManager(f"--interface {communication} --port can1").connect()
        self.module_id=module_id
        self.module = TMCM1260(self.interface,module_id=module_id)
        self.motor=self.module.motors[0]

        print(self.motor.drive_settings)
        self.motor.set_axis_parameter(self.motor.AP.MaxVelocity , max_velocity)
        self.motor.set_axis_parameter(self.motor.AP.MaxAcceleration , max_acc)
        self.motor.set_axis_parameter(self.motor.AP.V1,V1)
        #self.motor.set_axis_parameter(self.motor.AP.MaxDeceleration, MaxDeceleration)
        #self.motor.set_axis_parameter(self.motor.AP.D1, D1)
        self.motor.set_axis_parameter(self.motor.AP.StartVelocity, 1_000)
        self.motor.set_axis_parameter(self.motor.AP.StopVelocity, 1_000)
        self.motor.set_axis_parameter(self.motor.AP.RampWaitTime, 0)
        self.motor.set_axis_parameter(self.motor.AP.MaxCurrent,200)
        self.motor.set_axis_parameter(self.motor.AP.StandbyCurrent,100)
        self.motor.set_axis_parameter(self.motor.AP.SG2Threshold ,11)
        self.motor.set_axis_parameter(self.motor.AP.SG2FilterEnable,0)
        self.motor.set_axis_parameter(self.motor.AP.SmartEnergyStallVelocity,0)
        self.motor.set_axis_parameter(self.motor.AP.SmartEnergyHysteresis,15)
        self.motor.set_axis_parameter(self.motor.AP.SmartEnergyHysteresisStart,0)
        self.motor.set_axis_parameter(self.motor.AP.SECUS,1)
        self.motor.set_axis_parameter(self.motor.AP.SmartEnergyThresholdSpeed, 7_999_774)
        self.motor.drive_settings.boost_current = 0
        self.motor.drive_settings.microstep_resolution = self.motor.ENUM.MicrostepResolution256Microsteps
        #get the initial error between encoder and actual position
        self.positions = self.motor.get_actual_position()
        print(self.positions)
        self.motor.set_axis_parameter(self.motor.AP.EncoderPosition, 0)
        start_deviation_error= self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)-self.motor.get_axis_parameter(self.motor.AP.ActualPosition)
        print(self.motor.drive_settings)
        
    def computeErr(self):
        global start_deviation_error
        t = 200
        deviation_error = self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)-self.motor.get_axis_parameter(self.motor.AP.ActualPosition)-start_deviation_error
        
        #deviation_error = self.pos_nowEnc-self.pos_nowAP- self.prev_error
        if abs(deviation_error) > t:
            return True, deviation_error
        else:
            return False, 0

    def brake(self):
        start_time=time()
        runtime=0
        min_pos_rel = -10
        print("Braking")
        self.start_pos = self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
        self.end_pos = self.start_pos-70000
        self.motor.move_to(self.end_pos)
        print(self.start_pos)
        sleep(2)
        self.s_time = time()
        self.min_pos_rel=-10
        self.pos_nowAP = self.motor.get_axis_parameter(self.motor.AP.ActualPosition)
        self.pos_nowEnc = self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
        self.devErr = 0


        while runtime < 10:
            print('move_relative')
            self.pos_nowAP = self.motor.get_axis_parameter(self.motor.AP.ActualPosition)
            self.pos_nowEnc = self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
            print(f"The position for the parameter 1: {self.pos_nowAP} \nAnd for parameter 209: {self.pos_nowEnc}")
            self.check, self.devErr = self.computeErr()
            if self.check: #checking to see if it slipped, if it did, we should go back to the end_pos so then we move min_pos_rel to guarantee we are braking
                self.motor.move_by(self.devErr)
                print('WARNING!')
                sleep(2)
            self.motor.move_by(min_pos_rel)
            sleep(0.5) # Maybe taking it out could be useful to make the ticks even faster
            runtime = time() - start_time

        print('Going back to almost stretching pos')
        self.pos_nowAP = self.motor.get_axis_parameter(self.motor.AP.ActualPosition)
        self.pos_nowEnc = self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
        x = self.pos_nowAP - self.pos_nowEnc
        while self.pos_nowEnc < -100:
            self.motor.move_to(x)
            self.pos_nowEnc = self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
        self.motor.move_to(x)
        sleep(0.5)

    def move_stepper(self,step):
        desired_position = step+self.positions
        print(f"turn: {desired_position}")
        self.motor.set_axis_parameter(self.motor.AP.SmartEnergyStallVelocity, 0)
        print("setted parameters")
        self.motor.move_to(desired_position)

    def disconnect_motor(self):
        self.interface.close()
