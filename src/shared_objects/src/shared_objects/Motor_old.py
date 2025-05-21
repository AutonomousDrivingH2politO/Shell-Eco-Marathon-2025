#!/usr/bin/env python3
from pytrinamic.connections import ConnectionManager
from pytrinamic.modules import TMCM1260
from time import sleep, time

start_deviation_error = 0

def unwrap_position_signed(current_position, previous_position, max_position=2**32):
    """
    Funzione di unwrapping per ottenere una posizione "continua" evitando il roll-over.
    """
    if current_position >= max_position // 2:
        current_position_signed = current_position - max_position
    else:
        current_position_signed = current_position

    if previous_position >= max_position // 2:
        previous_position_signed = previous_position - max_position
    else:
        previous_position_signed = previous_position

    diff = current_position_signed - previous_position_signed
    if diff > max_position // 2:
        return previous_position_signed - (max_position - diff)
    elif diff < -max_position // 2:
        return previous_position_signed + (max_position + diff)
    else:
        return current_position_signed

class Stepper:
    def __init__(self, interface, data_rate, module_id, max_velocity, max_acc, MaxDeceleration, V1, A1, D1):
        global start_deviation_error
        if interface == "can":
            communication = "socketcan_tmcl"
        elif interface == "usb":
            communication = "usb_tmcl"
        else:
            raise ValueError(f"Cannot use {interface} communication, possible values are 'can' or 'usb'")

        # Inizializzazione dell'interfaccia del motore:
        self.interface = ConnectionManager(f"--interface {communication} --port can1 --data-rate {data_rate}").connect()
        self.module_id = module_id
        self.data_rate = data_rate
        self.module = TMCM1260(self.interface, module_id=module_id)
        self.motor = self.module.motors[0]

        # Impostazione dei parametri del motore:
        self.motor.set_axis_parameter(self.motor.AP.MaxVelocity, max_velocity)
        self.motor.set_axis_parameter(self.motor.AP.MaxAcceleration, max_acc)
        self.motor.set_axis_parameter(self.motor.AP.MaxDeceleration, MaxDeceleration)
        self.motor.set_axis_parameter(self.motor.AP.V1, V1)
        self.motor.set_axis_parameter(self.motor.AP.A1, A1)
        self.motor.set_axis_parameter(self.motor.AP.D1, D1)
        self.motor.set_axis_parameter(self.motor.AP.StartVelocity, 1_000)
        self.motor.set_axis_parameter(self.motor.AP.StopVelocity, 1_000)
        self.motor.set_axis_parameter(self.motor.AP.RampWaitTime, 0)
        self.motor.set_axis_parameter(self.motor.AP.MaxCurrent, 200)
        self.motor.set_axis_parameter(self.motor.AP.StandbyCurrent, 100)
        self.motor.set_axis_parameter(self.motor.AP.SG2Threshold, 11)
        self.motor.set_axis_parameter(self.motor.AP.SG2FilterEnable, 0)
        self.motor.set_axis_parameter(self.motor.AP.SmartEnergyStallVelocity, 0)
        self.motor.set_axis_parameter(self.motor.AP.SmartEnergyHysteresis, 15)
        self.motor.set_axis_parameter(self.motor.AP.SmartEnergyHysteresisStart, 0)
        self.motor.set_axis_parameter(self.motor.AP.SECUS, 1)
        self.motor.set_axis_parameter(self.motor.AP.SmartEnergyThresholdSpeed, 7_999_774)
        self.motor.drive_settings.boost_current = 0
        self.motor.drive_settings.microstep_resolution = self.motor.ENUM.MicrostepResolution256Microsteps

        # Impostazione della posizione iniziale: viene azzerata l'encoder a ogni istanziazione
        self.virtual_position = self.motor.get_actual_position()
        self.motor.set_axis_parameter(self.motor.AP.EncoderPosition, 0)
        self.effective_position = self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
        start_deviation_error = self.effective_position - self.virtual_position
        print(f"{self.virtual_position}")
        # Inizializza le variabili per l'unwrapping
        self.last_encoder_position = self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
        self.unwrapped_position = 0

        print(self.motor.drive_settings)

    def update_unwrapped_position(self):
        """
        Aggiorna e restituisce la posizione "unwrapped" basata sulla lettura corrente dell'encoder.
        """
        current_encoder = self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
        self.unwrapped_position = unwrap_position_signed(current_encoder, self.last_encoder_position)
        self.last_encoder_position = current_encoder
        return self.unwrapped_position

    def computeErr(self):
        global start_deviation_error
        t = 200
        deviation_error = self.effective_position - self.virtual_position - start_deviation_error

        if abs(deviation_error) > t:
            return True, deviation_error
        else:
            return False, 0

    def brake(self):
        start_time = time()
        runtime = 0
        min_pos_rel = -10
        print("Braking")
        self.start_pos = self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
        self.end_pos = self.start_pos - 70000
        self.motor.move_to(self.end_pos)
        print(self.start_pos)
        sleep(2)
        self.s_time = time()
        self.min_pos_rel = -10
        self.pos_nowAP = self.motor.get_axis_parameter(self.motor.AP.ActualPosition)
        self.pos_nowEnc = self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
        self.devErr = 0

        while runtime < 10:
            print('move_relative')
            self.pos_nowAP = self.motor.get_axis_parameter(self.motor.AP.ActualPosition)
            self.pos_nowEnc = self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
            print(f"The position for the parameter 1: {self.pos_nowAP} \nAnd for parameter 209: {self.pos_nowEnc}")
            self.check, self.devErr = self.computeErr()
            if self.check:  # se c'è uno slip, correggi la posizione
                self.motor.move_by(self.devErr)
                print('WARNING!')
                sleep(2)
            self.motor.move_by(min_pos_rel)
            sleep(0.5)
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

    def move_stepper(self, step):
        """
        Esegue il comando di spostamento sullo stepper.
        """
        print(f"Moving stepper to command: {step}")
        #self.motor.set_axis_parameter(self.motor.AP.SmartEnergyStallVelocity, 0)
        self.motor.move_to(step)

    def disconnect_motor(self):
        self.motor.stop()
        self.interface.close()