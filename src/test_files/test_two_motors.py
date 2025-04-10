import pytrinamic
from pytrinamic.connections import ConnectionManager
from pytrinamic.modules import TMCM1260
import time

modules={            #Receive ID CAN (da controllare su TMCL_IDE)
    "steering":3,
    "brake":1
}

interface_br=ConnectionManager("--interface socketcan_tmcl --port can0 --data-rate 250000").connect() #data-rate = bitrate se vuoi controllare a un bitrate differente ricorda di cambiare anche nel IDE di pytrinamic
interface_st=ConnectionManager("--interface socketcan_tmcl --port can1 --data-rate 1000000").connect()


steering=TMCM1260(interface_st,module_id=modules["steering"])
brake=TMCM1260(interface_br,module_id=modules["brake"])
while True:
    steering.rotate(0,20000)
    time.sleep(1)
    steering.stop(0)
    time.sleep(1)
    brake.rotate(0,20000)
    time.sleep(1)
    brake.stop(0)
    time.sleep(1)
