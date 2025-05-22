#!/usr/bin/env python3
from time import time
from pytrinamic.connections import ConnectionManager
from pytrinamic.modules import TMCM1260

# offset iniziale tra posizione virtuale ed effettiva (encoder)
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
    def __init__(self, interface, data_rate, module_id, max_velocity, max_acc, max_deceleration, v1, a1, d1):
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
        self.motor.set_axis_parameter(self.motor.AP.MaxDeceleration, max_deceleration)
        self.motor.set_axis_parameter(self.motor.AP.V1, v1)
        self.motor.set_axis_parameter(self.motor.AP.A1, a1)
        self.motor.set_axis_parameter(self.motor.AP.D1, d1)
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
        self.virtual_position = 0                    # posizione virtuale: quella "comandata"
        self.effective_position = 0                  # posizione reale letta da encoder
        start_deviation_error = 0                    # offset iniziale

        # Inizializza le variabili per l'unwrapping
        self.last_encoder_position = 0
        self.unwrapped_position = 0

        print(f"[Stepper] Initialized on port can1 id={module_id}")

    def update_effective_position(self):
        """
        Aggiorna il valore di posizione effettiva leggendo l'encoder.
        """
        self.effective_position = self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)

    def update_unwrapped_position(self):
        """
        Aggiorna e restituisce la posizione "unwrapped" basata sulla lettura corrente dell'encoder.
        """
        current_encoder = self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
        self.unwrapped_position = unwrap_position_signed(current_encoder, self.last_encoder_position)
        self.last_encoder_position = current_encoder
        return self.unwrapped_position

    def compute_slip_error(self, threshold=200):
        """
        Calcola l'errore tra la posizione virtuale (comandata) e quella effettiva (encoder).
        Ritorna True e valore di correzione se supera la soglia.
        """
        self.update_effective_position()
        deviation_error = self.effective_position - self.virtual_position
        if abs(deviation_error) > threshold:
            return True, deviation_error
        return False, 0

    def move_to(self, target):
        """
        Comando di movimento assoluto. Aggiorna la posizione virtuale.
        """
        self.motor.move_to(target)
        self.virtual_position = target

    def move_by(self, delta):
        """
        Comando di movimento relativo. Aggiorna la posizione virtuale.
        """
        self.motor.move_by(delta)
        self.virtual_position += delta

    def brake(self):
        """
        Sequenza di frenata progressiva senza sleep:
        1. Ritrae il motore di 70.000 tick
        2. Avanza lentamente correggendo eventuale slittamento
        3. Allinea AP con encoder
        """
        start_time = time()
        runtime = 0
        min_pos_rel = -10
        print("[Stepper] Braking...")

        # Step 1: ritorno rapido
        self.start_pos = self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
        self.end_pos = self.start_pos - 70000
        self.move_to(self.end_pos)
        print(f"[Stepper] Initial encoder position: {self.start_pos}")

        while not self.motor.get_position_reached():
            pass  # attesa attiva

        # Step 2: avanzamento incrementale con correzione slip
        self.devErr = 0
        loop_start = time()

        while runtime < 10:
            pos_ap = self.motor.get_axis_parameter(self.motor.AP.ActualPosition)
            pos_enc = self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
            print(f"[Stepper] Actual: {pos_ap} | Encoder: {pos_enc}")

            slipped, self.devErr = self.compute_slip_error()
            if slipped:
                print(f"[WARN] slip detected → correcting by {self.devErr} ticks")
                self.move_by(self.devErr)
                while not self.motor.get_position_reached():
                    pass

            self.move_by(min_pos_rel)
            while not self.motor.get_position_reached():
                pass

            runtime = time() - loop_start

        # Step 3: ritorno alla posizione bilanciata
        print("[Stepper] Returning to aligned position")
        pos_ap = self.motor.get_axis_parameter(self.motor.AP.ActualPosition)
        pos_enc = self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
        diff = pos_ap - pos_enc

        if abs(diff) > 0:
            self.move_to(pos_ap)
            while not self.motor.get_position_reached():
                pass

        print("[Stepper] Brake sequence completed")

    def disconnect_motor(self):
        """
        Ferma il motore e chiude l'interfaccia.
        """
        self.motor.stop()
        self.interface.close()
        print("[Stepper] Disconnected")
