import can
import time

can_ids = {
    'speed': 0x602,
    'emergency': 0x11,
    'rpm': 0x600,
    'throttle': 0x640,
    'enable': 0x21,  #Correct id is 0x21
}
can_filter = [{"can_id": 0x602, "can_mask": 0x602, "extended": False}]
bus = can.interface.Bus(channel='can0', bustype='socketcan')
print(bus.state)
import subprocess

import subprocess
"""
def close_can(interface="can1"):
 
    Gracefully closes the specified CAN interface, if it exists.

    :param interface: The name of the CAN interface to close (default: "can1").
    
    try:
        # Check if the interface exists
        result = subprocess.run(["ip", "link", "show", interface], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if result.returncode != 0:
            print(f"Error: The interface '{interface}' does not exist.")
            return
        
        # Bring down the CAN interface
        print(f"Closing CAN interface: {interface}...")
        subprocess.run(["sudo", "ip", "link", "set", interface, "down"], check=True)
        print(f"CAN interface {interface} has been closed properly.")
    except subprocess.CalledProcessError as e:
        print(f"Error while closing CAN interface {interface}: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")
"""
def send_can(id:int, data:int):  # TODO change the type and how to send
    try:
        data_array = list(data.to_bytes(2, byteorder="big"))
        for _ in range(8 - len(data_array)):
            data_array.append(0)
        data_array = bytearray(data_array)
        msg = can.Message(arbitration_id=id, data=data_array, is_extended_id=False)
        bus.send(msg)
        print(f"Messaggio inviato con ID: {hex(id)}, Dati: {data_array}")
    except can.CanError as e:
        print(f"Errore nell'invio del messaggio CAN: {e}")
    except Exception as e:
        print(f"Errore sconosciuto: {e}")
        
        
# Test the message sending
send_can(can_ids['enable'], 65535)
#time.sleep(5)
#send_can(can_ids['enable'], 0)

#time.sleep(5)
#close_can("can0")
if bus.recv == True:
    print(bus.recv)
print("Messaggio 'enable' inviato.")


