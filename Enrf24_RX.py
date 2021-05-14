import sys
import PyEnrf24

radio = PyEnrf24.Enrf24(0, 0, "GPIO3_19", "P9_19", "GPIO1_17")

rxaddr = [0xDE, 0xAD, 0xBE, 0xEF, 0x01]

radio.begin()
radio.setRXaddress(rxaddr)
radio.enableRX()

radio.printRadioState()
print("")

#radio.printDetails()


while (1):
    while (not radio.available(True)):
        pass
    
    if (radio.read()):
        sys.stdout.write("Received packet: ")
        print(radio.getMessage())
