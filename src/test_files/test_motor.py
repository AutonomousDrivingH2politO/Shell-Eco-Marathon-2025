import pytrinamic
from pytrinamic.connections import ConnectionManager
from pytrinamic.modules import TMCM1260
import time

modules={            #Receive ID CAN (da controllare su TMCL_IDE)
    "steering":3,
    "brake":1
}

#Creation of CAN interface (uncomment the interface you need)
interface_br=ConnectionManager("--interface socketcan_tmcl --port can1 --data-rate 1000000").connect() #data-rate = bitrate se vuoi controllare a un bitrate differente ricorda di cambiare anche nel IDE di pytrinamic
#interface_st=ConnectionManager("--interface socketcan_tmcl --port can1 --data-rate 1000000").connect()

#Creation of the instance motor (uncomment the instance you need)

#Steering:
#steering=TMCM1260(interface_st,module_id=modules["steering"])
#steering.set_axis_parameter(steering.AP.MaxVelocity, 80000)
#steering.set_axis_parameter(steering.AP.MaxAcceleration, 50000)
#steering.set_axis_parameter(steering.AP.MaxDeceleration, 50000)
#steering.set_axis_parameter(steering.AP.V1, 100000)
#steering.set_axis_parameter(steering.AP.A1, 50000)
#steering.set_axis_parameter(steering.AP.D1, 50000)
#steering.set_axis_parameter(steering.AP.StartVelocity, 1_000)
#steering.set_axis_parameter(steering.AP.StopVelocity, 1_000)
#steering.set_axis_parameter(steering.AP.RampWaitTime, 0)
#steering.set_axis_parameter(steering.AP.MaxCurrent, 200)
#steering.set_axis_parameter(steering.AP.StandbyCurrent, 100)
#steering.set_axis_parameter(steering.AP.SG2Threshold, 11)
#steering.set_axis_parameter(steering.AP.SG2FilterEnable, 0)
#steering.set_axis_parameter(steering.AP.SmartEnergyStallVelocity, 0)
#steering.set_axis_parameter(steering.AP.SmartEnergyHysteresis, 15)
#steering.set_axis_parameter(steering.AP.SmartEnergyHysteresisStart, 0)
#steering.set_axis_parameter(steering.AP.SECUS, 1)
#steering.set_axis_parameter(steering.AP.SmartEnergyThresholdSpeed, 7_999_774)


#Brake:
brake=TMCM1260(interface_br,module_id=modules["brake"])
brake.set_axis_parameter(brake.AP.MaxVelocity, 50000)
brake.set_axis_parameter(brake.AP.MaxAcceleration, 140000)
#brake.set_axis_parameter(brake.AP.MaxDeceleration, max_deceleration)
brake.set_axis_parameter(brake.AP.V1, 0)
#brake.set_axis_parameter(brake.AP.A1, a1)
#brake.set_axis_parameter(brake.AP.D1, d1)
brake.set_axis_parameter(brake.AP.StartVelocity, 1_000)
brake.set_axis_parameter(brake.AP.StopVelocity, 1_000)
brake.set_axis_parameter(brake.AP.RampWaitTime, 0)
brake.set_axis_parameter(brake.AP.MaxCurrent, 200)
brake.set_axis_parameter(brake.AP.StandbyCurrent, 100)
brake.set_axis_parameter(brake.AP.SG2Threshold, 11)
brake.set_axis_parameter(brake.AP.SG2FilterEnable, 0)
brake.set_axis_parameter(brake.AP.SmartEnergyStallVelocity, 0)
brake.set_axis_parameter(brake.AP.SmartEnergyHysteresis, 15)
brake.set_axis_parameter(brake.AP.SmartEnergyHysteresisStart, 0)
brake.set_axis_parameter(brake.AP.SECUS, 1)
brake.set_axis_parameter(brake.AP.SmartEnergyThresholdSpeed, 7_999_774)



#Test for moving steering actuator (uncomment to test)
#while True:
    #steering.rotate(0,100000)
    #time.sleep(3)
    #steering.stop(0)
    #time.sleep(3)
    
#Test for moving brake actuator (uncomment to test)
#while True:
    #brake.rotate(0,20000)
    #time.sleep(1)
    #brake.stop(0)
    #time.sleep(1)




