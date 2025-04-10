#!/usr/bin/env python3
from pytrinamic.connections import ConnectionManager
from pytrinamic.modules import TMCM1260
import time

# Apri la connessione senza "with"
connection_manager = ConnectionManager("--interface socketcan_tmcl --port can1")
connection = connection_manager.connect()

try:
    # Crea il modulo e imposta il motore
    module = TMCM1260(connection, module_id=2)
    motor = module.motors[0]
    motor.set_axis_parameter(motor.AP.MaxVelocity, 2000)
    motor.set_axis_parameter(motor.AP.MaxAcceleration, 1000)
    
    # Avvia rotazione del motore
    motor.rotate(5000)
    print("Il motore sta ruotando")
    time.sleep(5)
    
    # Ferma il motore
    motor.stop()
    print("Il motore si è fermato")

finally:
    # Chiudi la connessione
    connection.close()

