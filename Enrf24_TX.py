import sys
import time
import PyEnrf24

def dump_radio_status(status):
    sys.stdout.write("Enrf24 radio transceiver status: ")
    if  status == PyEnrf24.ENRF24_STATE_NOTPRESENT:
      print("NO TRANSCEIVER PRESENT");
      
    elif status == PyEnrf24.ENRF24_STATE_DEEPSLEEP:
      print("DEEP SLEEP <1uA power consumption")
      
    elif status == PyEnrf24.ENRF24_STATE_IDLE:
      print("IDLE module powered up w/ oscillators running")
      
    elif status == PyEnrf24.ENRF24_STATE_PTX:
      print("Actively Transmitting")
      
    elif status == PyEnrf24.ENRF24_STATE_PRX:
      print("Receive Mode")
    
    else:
      print("UNKNOWN STATUS CODE")

#                               CE          CSN       IRQ
radio = PyEnrf24.Enrf24(0, 0, "GPIO3_19", "P9_19", "GPIO1_17")
txaddr = [0xDE, 0xAD, 0xBE, 0xEF, 0x01]

str_on = "ON"
str_off = "OFF"


radio.begin()
dump_radio_status(radio.radioState())

radio.setTXaddress(txaddr)

while(1):
  sys.stdout.write("Sending packet: ")
  print(str_on)
  radio.write(str_on)
  radio.flush()  # Force transmit (don't wait for any more data)
  dump_radio_status(radio.radioState())  # Should report IDLE
  time.sleep(1)

  sys.stdout.write("Sending packet: ")
  print(str_off)
  radio.write(str_off)
  radio.flush()
  dump_radio_status(radio.radioState())
  time.sleep(1)

