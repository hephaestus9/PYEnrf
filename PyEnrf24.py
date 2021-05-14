#* nRF24L01+ I/O for Energia
#*
#* Copyright (c) 2013 Eric Brundick <spirilis [at] linux dot com>
#*  Permission is hereby granted, free of charge, to any person 
#*  obtaining a copy of this software and associated documentation 
#*  files (the "Software"), to deal in the Software without 
#*  restriction, including without limitation the rights to use, copy, 
#*  modify, merge, publish, distribute, sublicense, and/or sell copies 
#*  of the Software, and to permit persons to whom the Software is 
#*  furnished to do so, subject to the following conditions:
#*
#*  The above copyright notice and this permission notice shall be 
#*  included in all copies or substantial portions of the Software.
#*
#*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
#*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
#*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
#*  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
#*  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
#*  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
#*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
#*  DEALINGS IN THE SOFTWARE.

#   Edited: J.Brian 3-24-16

import time
import sys

try:
    from Adafruit_BBIO.SPI import SPI
    import Adafruit_BBIO.GPIO as GPIO
except:
    print("Adafruit_BBIO is required for this library.")
    print("https://learn.adafruit.com/setting-up-io-python-library-on-beaglebone-black/overview")


from nRF24L01 import *

class Enrf24():
    __bus = None
    __device = None
    
    __rf_status = 0
    __rf_addr_width = 5
    __lastirq = None
    __readpending = 0
    __lastTXfailed = False
    __txbuf_len = 0
    __txbuf = []

    # Internal IRQ handling 
    __ENRF24_IRQ_TX           = 0x20
    __ENRF24_IRQ_RX           = 0x40
    __ENRF24_IRQ_TXFAILED     = 0x10
    __ENRF24_IRQ_MASK         = 0x70
    
    __ENRF24_CFGMASK_IRQ      = 0

    def __init__(self, bus, device, cePin, csnPin, irqPin):
        self.__bus = bus
        self.__device = device
        self.cePin = cePin
        self.csnPin = csnPin
        self.irqPin = irqPin
        self.spi = SPI(self.__bus, self.__device)
        self.spi.msh = 10000
        self.spi.bpw = 8  # bits/word
        self.spi.threewire = False
        self.spi.lsbfirst = False
        self.spi.mode = 0
        self.spi.cshigh = False  
        self.spi.open(0,0)
        self.last_payload = ""
        
        

    def begin(self, datarate=1000000, channel=0):   # Specify bitrate & channel
        GPIO.setup(self.cePin, GPIO.OUT)
        GPIO.output(self.cePin, GPIO.LOW)
        GPIO.setup(self.csnPin, GPIO.OUT)
        GPIO.output(self.csnPin, GPIO.HIGH)
        GPIO.setup(self.irqPin, GPIO.IN)            # No pullups; the transceiver provides this!
        self.spi.writebytes([0x00])                 # Strawman transfer, fixes USCI issue on G2553
        #self.spi.writebytes([0xCF, 0x00])

        # Is the transceiver present/alive?
        if (not self.__isAlive()):
            return False                                # Nothing more to do here...

        # Wait 100ms for module to initialize
        time.sleep(0.1)

        # Init certain registers
        self.__writeReg(RF24_CONFIG, 0x00)                # Deep power-down, everything disabled
        self.__writeReg(RF24_EN_AA, 0x03)
        self.__writeReg(RF24_EN_RXADDR, 0x03)
        self.__writeReg(RF24_RF_SETUP, 0x00)
        self.__writeReg(RF24_STATUS, self.__ENRF24_IRQ_MASK)     # Clear all IRQs
        self.__writeReg(RF24_DYNPD, 0x03)
        self.__writeReg(RF24_FEATURE, RF24_EN_DPL)        # Dynamic payloads enabled by default

        # Set all parameters
        if (channel > 125):
            channel = 125
        
        self.deepsleep()
        self.__issueCmd(RF24_FLUSH_TX)
        self.__issueCmd(RF24_FLUSH_RX)
        self.__readpending = 0
        self.__irq_clear(self.__ENRF24_IRQ_MASK)
        self.setChannel(channel)
        self.setSpeed(datarate)
        self.setTXpower()
        self.setAutoAckParams()
        self.setAddressLength(self.__rf_addr_width)
        self.setCRC(True)                           # Default = CRC on, 8-bit
        return True
    
    def end(self):                                  # Shut it off, clear the library's state
        self.__txbuf_len = 0;
        self.__rf_status = 0
        self.__rf_addr_width = 5

        if (not self.__isAlive()):
            return
        
        self.deepsleep()
        self.__issueCmd(RF24_FLUSH_TX)
        self.__issueCmd(RF24_FLUSH_RX)
        self.readpending = 0
        self.__irq_clear(self.__ENRF24_IRQ_MASK)
        GPIO.output(self.cePin, GPIO.LOW)
        GPIO.output(self.csnPin, GPIO.HIGH)
    
    # I/O
    def available(self, checkIrq=False):            # Check if incoming data is ready to be read
        #print(checkIrq and GPIO.input(self.irqPin) and self.__readpending == 0)
        if (checkIrq and GPIO.input(self.irqPin) and self.__readpending == 0):
            return False
        self.__maintenanceHook()
        if ( (not self.__readReg(RF24_FIFO_STATUS)) & RF24_RX_EMPTY):
            return True
        
        if (self.__readpending):
            return True
        
        return False
    
    def read(self, maxlen=32):               # Read contents of RX buffer up to
        buf = None
        plwidth = 0
        res = ""
         
        self.__maintenanceHook()
        self.__readpending = 0
        if ((self.__readReg(RF24_FIFO_STATUS) & RF24_RX_EMPTY) or maxlen < 1):
            return 0
 
        plwidth = self.__readCmdPayload(RF24_R_RX_PL_WID, plwidth, 1, 1)[0]
        buf = self.__readCmdPayload(RF24_R_RX_PAYLOAD, buf, plwidth, maxlen)
        if (self.__irq_derivereason() and self.__ENRF24_IRQ_RX):
            self.__irq_clear(self.__ENRF24_IRQ_RX)

        for i in buf:
            res += chr(i)
        
        self.last_payload = res
        return res                                              # 'maxlen' bytes, return final length.
                                                                # 'inbuf' should be maxlen+1 since a
                                                                # null '\0' is tacked onto the end.
    
    def getMessage(self):
        return self.last_payload
                                                                                                         
    def write(self, data):                          
        if (self.__txbuf_len == 32):                                    # If we're trying to stuff an already-full buffer...
            self.flush()                                                # Blocking OTA TX
        
        txbuf = []
        data = list(data)
        for i in data:
            txbuf.append(ord(i))
        self.__txbuf = txbuf
        self.__txbuf_len = len(txbuf)

        return 1
    
    def flush(self):                                # Force transmission of TX ring buffer contents
        reg = None
        addrbuf = []
        enaa = False
        origrx = False

        if (self.__txbuf_len == 0):
            return                                  # Zero-length buffer?  Nothing to send!

        reg = self.__readReg(RF24_FIFO_STATUS)
        if (reg & BITS["BIT5"]):                    # RF24_TX_FULL #define is BIT0, which is not the correct bit for FIFO_STATUS.
            # Seen this before with a user whose CE pin was messed up.
            self.__issueCmd(RF24_FLUSH_TX)
            self.__txbuf_len = 0
            return  # Should never happen, but nonetheless a precaution to take.

        self.__maintenanceHook()

        if (reg & RF24_TX_REUSE):
            # If somehow TX_REUSE is enabled, we need to flush the TX queue before loading our new payload.
            self.__issueCmd(RF24_FLUSH_TX)

        if (self.__readReg(RF24_EN_AA) & 0x01 and (self.__readReg(RF24_RF_SETUP) & 0x28) != 0x20):
            # AutoACK enabled, must write TX addr to RX pipe#0
            # Note that 250Kbps doesn't support auto-ack, so we check RF24_RF_SETUP to verify that.    
            enaa = True
            self.__readTXaddr(addrbuf)
            self.__writeRXaddrP0(addrbuf)

        reg = self.__readReg(RF24_CONFIG);
        if ( not (reg & RF24_PWR_UP)):
            #digitalWrite(_cePin, HIGH);  // Workaround for SI24R1 knockoff chips
            self.__writeReg(RF24_CONFIG, self.__ENRF24_CFGMASK_IRQ | self.__ENRF24_CFGMASK_CRC(reg) | RF24_PWR_UP)
            time.sleep(.05)  # 5ms delay required for nRF24 oscillator start-up
            #digitalWrite(_cePin, LOW);

        if (reg & RF24_PRIM_RX):
            origrx = True
            GPIO.output(self.cePin, GPIO.LOW)
            self.__writeReg(RF24_CONFIG, self.__ENRF24_CFGMASK_IRQ | self.__ENRF24_CFGMASK_CRC(reg) | RF24_PWR_UP)

        self.__txbuf = self.__issueCmdPayload(RF24_W_TX_PAYLOAD, self.__txbuf)
        GPIO.output(self.cePin, GPIO.HIGH)
        time.sleep(0.1)
        GPIO.output(self.cePin, GPIO.LOW)

        self.__txbuf_len = 0                                              # Reset TX ring buffer

        while (GPIO.input(self.irqPin)):                         # Wait until IRQ fires
            pass

        # IRQ fired
        self.__maintenanceHook()                                  # Handle/clear IRQ

        # Purge Pipe#0 address (set to module's power-up default)
        if (enaa):
            addrbuf = [0xE7, 0xE7, 0xE7, 0xE7, 0xE7]
            self.__writeRXaddrP0(addrbuf)

        # If we were in RX mode before writing, return back to RX mode.
        if (origrx):
            self.enableRX()

    def purge(self):                                # Ignore TX ring buffer contents, return ring pointer to 0.
        self.__txbuf_len = 0

    # Power-state related stuff-
    def radioState(self):                           # Evaluate current state of the transceiver (see ENRF24_STATE_* defines)
        if not self.__isAlive():
            return ENRF24_STATE_NOTPRESENT
  
        counter = 15
        reg = self.__readReg(RF24_CONFIG)
        if reg == 0:
            while reg == 0 and counter < 15:
                reg = self.__readReg(RF24_CONFIG)
                counter += 1

        if ( not (reg & RF24_PWR_UP)):
            return ENRF24_STATE_DEEPSLEEP

        # At this point it's either Standby-I, II or PRX.
        if (reg & RF24_PRIM_RX):
            if (GPIO.input(self.cePin)):
                return ENRF24_STATE_PRX
            # PRIM_RX=1 but CE=0 is a form of idle state.
            return ENRF24_STATE_IDLE

        # Check if TX queue is empty, if so it's idle, if not it's PTX.
        if (self.__readReg(RF24_FIFO_STATUS) & RF24_TX_EMPTY):
            return ENRF24_STATE_IDLE
        return ENRF24_STATE_PTX

    def printRadioState(self):
        status = self.radioState()
        sys.stdout.write("Enrf24 radio transceiver status: ")
        if  status == ENRF24_STATE_NOTPRESENT:
          print("NO TRANSCEIVER PRESENT");
      
        elif status == ENRF24_STATE_DEEPSLEEP:
          print("DEEP SLEEP <1uA power consumption")
      
        elif status == ENRF24_STATE_IDLE:
          print("IDLE module powered up w/ oscillators running")
      
        elif status == ENRF24_STATE_PTX:
          print("Actively Transmitting")
      
        elif status == ENRF24_STATE_PRX:
          print("Receive Mode")
    
        else:
          print("UNKNOWN STATUS CODE")
    
    def printStatus(self):
        status = self.__readReg(RF24_STATUS)
        data_ready = str(hex(status & 0x40))
        data_sent = str(hex(status & 0x20))
        max_tx_retries = str(hex(status & 0x10))
        
        if status & 0x0E == 0x0E:
            rx_pipe_no = "RX FIFO Empty"
        elif status & 0x02 == 0x02:
            rx_pipe_no = 1
        elif status & 0x04 == 0x04:
            rx_pipe_no = 2
        elif status & 0x06 == 0x06:
            rx_pipe_no = 3
        elif status & 0x08 == 0x08:
            rx_pipe_no = 4
        elif status & 0x0A == 0x0A:
            rx_pipe_no = 5
        elif ~status & 0x0E:
            rx_pipe_no = 0
        else:
            rx_pipe_no = "Error"
        
        rx_pipe_no = str(rx_pipe_no)
        tx_fifo_full = str(status & 0x01)

        status = str(hex(status))

        sys.stdout.write("STATUS=")
        sys.stdout.write(status)
        sys.stdout.write("\tRX_DR=")
        sys.stdout.write(data_ready)
        sys.stdout.write(" TX_DS=")
        sys.stdout.write(data_sent)
        sys.stdout.write(" MAX_RT=")
        sys.stdout.write(max_tx_retries)
        sys.stdout.write(" RX_P_NO=")
        sys.stdout.write(rx_pipe_no)
        sys.stdout.write(" TX_FULL=")
        print(tx_fifo_full)
        print("")

    def printDetails(self):
        self.printStatus()
        
        buf = []
        sys.stdout.write("RX_ADDR_P0=")
        buf = self.__readRegMultiLSB(RF24_RX_ADDR_P0, buf, self.__rf_addr_width)
        for i in buf:
            sys.stdout.write(hex(i) + " ")
        print("")
        sys.stdout.write("RX_ADDR_P1=")
        buf = self.__readRegMultiLSB(RF24_RX_ADDR_P1, buf, self.__rf_addr_width)
        for i in buf:
            sys.stdout.write(hex(i) + " ")
        print("")
        sys.stdout.write("RX_ADDR_P2=")
        buf = self.__readRegMultiLSB(RF24_RX_ADDR_P2, buf, self.__rf_addr_width)
        for i in buf:
            sys.stdout.write(hex(i) + " ")
        print("")
        sys.stdout.write("RX_ADDR_P3=")
        buf = self.__readRegMultiLSB(RF24_RX_ADDR_P3, buf, self.__rf_addr_width)
        for i in buf:
            sys.stdout.write(hex(i) + " ")
        print("")
        sys.stdout.write("RX_ADDR_P4=")
        buf = self.__readRegMultiLSB(RF24_RX_ADDR_P4, buf, self.__rf_addr_width)
        for i in buf:
            sys.stdout.write(hex(i) + " ")
        print("")
        sys.stdout.write("RX_ADDR_P5=")
        buf = self.__readRegMultiLSB(RF24_RX_ADDR_P5, buf, self.__rf_addr_width)
        for i in buf:
            sys.stdout.write(hex(i) + " ")
        print("")
        print("")

        sys.stdout.write("TX_ADDR=")
        buf = self.__readRegMultiLSB(RF24_TX_ADDR, buf, self.__rf_addr_width)
        for i in buf:
            sys.stdout.write(hex(i) + " ")
        print("")
        print("")

        sys.stdout.write("RX_PW_P0=")
        print(hex(self.__readReg(RF24_RX_PW_P0)))
       
        sys.stdout.write("RX_PW_P1=")
        print(hex(self.__readReg(RF24_RX_PW_P1)))
        
        sys.stdout.write("RX_PW_P2=")
        print(hex(self.__readReg(RF24_RX_PW_P2)))
        
        sys.stdout.write("RX_PW_P3=")
        print(hex(self.__readReg(RF24_RX_PW_P3)))
        
        sys.stdout.write("RX_PW_P4=")
        print(hex(self.__readReg(RF24_RX_PW_P4)))
        
        sys.stdout.write("RX_PW_P5=")
        print(hex(self.__readReg(RF24_RX_PW_P5)))
        print("")

        sys.stdout.write("EN_AA=")
        print(bin(self.__readReg(RF24_EN_AA)))
        
        sys.stdout.write("EN_RXADDR=")
        print(bin(self.__readReg(RF24_EN_RXADDR)))
        
        sys.stdout.write("RF_CH=")
        print(hex(self.__readReg(RF24_RF_CH)))
        
        sys.stdout.write("RF_SETUP=")
        print(bin(self.__readReg(RF24_RF_SETUP)))
        
        sys.stdout.write("CONFIG=")
        print(bin(self.__readReg(RF24_CONFIG)))
        
        sys.stdout.write("DYNPD=")
        print(bin(self.__readReg(RF24_DYNPD)))
        
        sys.stdout.write("FEATURE=")
        print(bin(self.__readReg(RF24_FEATURE)))
        print("")

        sys.stdout.write("Data Rate=")
        print(self.getSpeed())
        
        sys.stdout.write("CRC Length=")
        print(self.getCRC())

        sys.stdout.write("PA Power=")
        print(self.getTXpower())


    def deepsleep(self):                            # Enter POWERDOWN mode, ~0.9uA power consumption
        reg = self.__readReg(RF24_CONFIG)
        if (reg & (RF24_PWR_UP | RF24_PRIM_RX)):
            self.__writeReg(RF24_CONFIG, self.__ENRF24_CFGMASK_IRQ | self.__ENRF24_CFGMASK_CRC(reg))

        GPIO.output(self.cePin, GPIO.LOW)
    
    def enableRX(self):                             # Enter PRX mode (~14mA)
        reg = self.__readReg(RF24_CONFIG)
        self.__writeReg(RF24_CONFIG, self.__ENRF24_CFGMASK_IRQ | self.__ENRF24_CFGMASK_CRC(reg) | RF24_PWR_UP | RF24_PRIM_RX)
        self.__writeReg(RF24_RX_PW_P0, 0x20)
        self.__writeReg(RF24_RX_PW_P1, 0x20)
        GPIO.output(self.cePin, GPIO.HIGH)

        if ( not (reg & RF24_PWR_UP)):       # Powering up from deep-sleep requires 5ms oscillator start delay
            time.sleep(0.05)
        
    def disableRX(self):                            # Disable PRX mode (PRIM_RX bit in CONFIG register)
                                                    # Note this won't necessarily push the transceiver into deep sleep, but rather
                                                    # an idle standby mode where its internal oscillators are ready & running but
                                                    # the RF transceiver PLL is disabled.  ~26uA power consumption.
        GPIO.output(self.cePin, GPIO.LOW)

        reg = self.__readReg(RF24_CONFIG)
        if (reg & RF24_PWR_UP):              # Keep us in standby-I if we're coming from RX mode, otherwise stay
                                                    # in deep-sleep if we call this while already in PWR_UP=0 mode.
                
            self.__writeReg(RF24_CONFIG, self.__ENRF24_CFGMASK_IRQ | self.__ENRF24_CFGMASK_CRC(reg) | RF24_PWR_UP)
        else:
            self.__writeReg(RF24_CONFIG, self.__ENRF24_CFGMASK_IRQ | self.__ENRF24_CFGMASK_CRC(reg))
                        

    # Custom tweaks to RF parameters, packet parameters
    def autoAck(self, onoff=True):                  # Enable/disable auto-acknowledgements (enabled by default)
        reg = self.__readReg(RF24_EN_AA)
        if (onoff):
            if ( not (reg & 0x01) or not (reg & 0x02)):
                self.__writeReg(RF24_EN_AA, 0x03)
           
        else:
            if (reg & 0x03):
                self.__writeReg(RF24_EN_AA, 0x00)
    
    def setChannel(self, channel):
        if (channel > 125):
            channel = 125
        self.__writeReg(RF24_RF_CH, channel)

    def setTXpower(self, dBm=0):                    # Only a few values supported by this (0, -6, -12, -18 dBm)
        reg = self.__readReg(RF24_RF_SETUP) & 0xF8  # preserve RF speed settings
        pwr = 0x06
        if (dBm >= 7):
            pwr = 0x07
        if (dBm < 0):
            pwr = 0x04
        if (dBm < -6):
            pwr = 0x02
        if (dBm < -12):
            pwr = 0x00
        self.__writeReg(RF24_RF_SETUP, reg | pwr)
    
    def setSpeed(self, rfspeed):                    # Set 250000, 1000000, 2000000 speeds.
        reg = self.__readReg(RF24_RF_SETUP) & 0xD7  # preserve RF power settings
        spd = 0x01
        if (rfspeed < 2000000):
            spd = 0x00
        if (rfspeed < 1000000):
            spd = 0x04
        self.__writeReg(RF24_RF_SETUP, reg | (spd << 3))
    
    def setCRC(self, onoff, crc16bit=False):                # Enable/disable CRC usage inside nRF24's
                                                            # hardware packet engine, specify 8 or
                                                            # 16-bit CRC.
        crcbits = 0

        reg = self.__readReg(RF24_CONFIG) & 0xF3            # preserve IRQ mask, PWR_UP/PRIM_RX settings
        if (onoff):
            crcbits |= RF24_EN_CRC
        if (crc16bit):
            crcbits |= RF24_CRCO
        self.__writeReg(RF24_CONFIG, (reg | crcbits))
                                                         
    # Set AutoACK retry count, timeout params (0-15, 250-4000 respectively)
    def setAutoAckParams(self, autoretry_count=15, autoretry_timeout=2000):
        setup_retr = 0

        setup_retr = autoretry_count & 0x0F
        autoretry_timeout -= 250
        setup_retr |= ((autoretry_timeout / 250) & 0x0F) << 4
        self.__writeReg(RF24_SETUP_RETR, setup_retr)
    
    # Protocol addressing -- receive, transmit addresses
    def setAddressLength(self, len):                # Valid parameters = 3, 4 or 5.  Defaults to 5.
        if (len < 3):
            len = 3
        if (len > 5):
            len = 5

        self.__writeReg(RF24_SETUP_AW, len-2)
        self.__rf_addr_width = len
    
    def setRXaddress(self, rxaddr):                 # 3-5 byte RX address loaded into pipe#1
        self.__writeRegMultiLSB(RF24_RX_ADDR_P1, rxaddr)
    
    def setTXaddress(self, txaddr):                 # 3-5 byte TX address loaded into TXaddr register
        self.__writeRegMultiLSB(RF24_TX_ADDR, txaddr)

    # Miscellaneous feature
    def rfSignalDetected(self):                     # Read RPD register to determine if transceiver has presently detected an RF signal
                                                    # of -64dBm or greater.  Only works in PRX (enableRX()) mode.
        rpd = self.__readReg(RF24_RPD)
        return rpd
                                
    # Query current parameters
    def getChannel(self):
       return self.__readReg(RF24_RF_CH)
    
    
    def getSpeed(self):
        reg = self.__readReg(RF24_RF_SETUP) & 0x28

        if (reg == 0x00):
                return 1000000
        elif (reg == 0x08):
                return 2000000
        elif (reg == 0x20):
                return 250000
        else:
            return 0
    
    def getTXpower(self):
        reg = self.__readReg(RF24_RF_SETUP) & 0x07

        if (reg & 0x01):
            return 7                                    # SI24R1-only +7dBm mode
        elif (reg == 0x02):
                return -12
        elif (reg == 0x04):
                return -6
        elif (reg == 0x06):
                return 0
        else:
            return -18
    
    def getAddressLength(self):
       return self.__rf_addr_width   
    
    def getRXaddress(self):
        buf = []
        buf = self.__readRegMultiLSB(RF24_RX_ADDR_P1, buf, self.__rf_addr_width)
        return buf
    
    def getTXaddress(self):
        buf = []
        buf = self.__readRegMultiLSB(RF24_TX_ADDR, buf, rf_addr_width)
        return buf
    
    def getAutoAck(self):
        reg = self.__readReg(RF24_EN_AA)

        if (reg):
            return True
        else:
            return False
    
    def getCRC(self):
        reg = self.__readReg(RF24_CONFIG) & 0x0C

        if (reg == 0x08):
            return 8
        elif (reg == 0x0C):
            return 16
        else:
            return 0

    def __readReg(self, addr):
        GPIO.output(self.csnPin, GPIO.LOW)
        result = self.spi.xfer2([(RF24_R_REGISTER | addr), RF24_NOP])
        self.__rf_status = result[0]
        GPIO.output(self.csnPin, GPIO.HIGH)
        return result[1]

    def __readRegMultiLSB(self, addr, buf, length):
        txbuf = [(RF24_R_REGISTER | addr)]
        for i in range(length):
            txbuf.append(RF24_NOP)
        GPIO.output(self.csnPin, GPIO.LOW)
        buf = self.spi.xfer2(txbuf)
        self.__rf_status = buf[0]
        status = []
        for i in range(1, len(buf) + 1):
            status.append(buf[-i])
        status.pop()
        GPIO.output(self.csnPin, GPIO.HIGH)
        return status

    def __writeReg(self, addr, val):
        GPIO.output(self.csnPin, GPIO.LOW)
        res = self.spi.xfer2([(RF24_W_REGISTER | addr), val])
        GPIO.output(self.csnPin, GPIO.HIGH)

    def __writeRegMultiLSB(self, addr, buf):
        txbuf = [(RF24_W_REGISTER | addr)]
        for i in range(1, len(buf) + 1):
            txbuf.append(buf[-i])
        GPIO.output(self.csnPin, GPIO.LOW);
        status = self.spi.xfer2(txbuf)
        GPIO.output(self.csnPin, GPIO.HIGH)

    def __issueCmd(self, cmd):
        GPIO.output(self.csnPin, GPIO.LOW)
        self.spi.writebytes([cmd])
        GPIO.output(self.csnPin, GPIO.HIGH)

    def __readCmdPayload(self, cmd, buf, length, maxlen):
        GPIO.output(self.csnPin, GPIO.LOW);
        messg = []
        txbuf = [cmd]
        for i in range(maxlen):
            txbuf.append(RF24_NOP)
        buf = self.spi.xfer2(txbuf)                  # Beyond maxlen bytes, just discard the remaining data.
        self.__rf_status = buf[0]
        for i in range(1, length + 1):
            messg.append(buf[i])
        GPIO.output(self.csnPin, GPIO.HIGH)
        return messg

    def __issueCmdPayload(self, cmd, buf):
        payload = []
        payload.append(cmd)
        for i in buf:
            payload.append(i)
        GPIO.output(self.csnPin, GPIO.LOW)
        res = self.spi.xfer2(payload)
        GPIO.output(self.csnPin, GPIO.HIGH)

    def __irq_getreason(self):
        self.__lastirq = self.__readReg(RF24_STATUS) & self.__ENRF24_IRQ_MASK

    def __irq_derivereason(self):                               # Get IRQ status from rf_status w/o querying module over SPI.
        self.__lastirq = self.__rf_status & self.__ENRF24_IRQ_MASK
        
    def __irq_clear(self, irq):
        self.__writeReg(RF24_STATUS, (irq & self.__ENRF24_IRQ_MASK))

    def __isAlive(self):
        self.spi.writebytes([0x00])
        aw = self.__readReg(RF24_SETUP_AW)
        return ((aw & 0xFC) == 0x00 and (aw & 0x03) != 0x00)

    def __readTXaddr(self, buf):
        self.__readRegMultiLSB(RF24_TX_ADDR, buf, self.__rf_addr_width)

    def __writeRXaddrP0(self, buf):
        self.__writeRegMultiLSB(RF24_RX_ADDR_P0, buf);

    def __maintenanceHook(self):                                # Handles IRQs and purges RX queue when erroneous contents exist.
        i = 0

        self.__irq_getreason()

        if (self.__lastirq & self.__ENRF24_IRQ_TXFAILED):
            self.__lastTXfailed = True
            self.__issueCmd(RF24_FLUSH_TX)
            self.__irq_clear(self.__ENRF24_IRQ_TXFAILED)

        if (self.__lastirq & self.__ENRF24_IRQ_TX):
            self.__lastTXfailed = False
            self.__irq_clear(self.__ENRF24_IRQ_TX)

        if (self.__lastirq & self.__ENRF24_IRQ_RX):
            if ( (not self.__readReg(RF24_FIFO_STATUS)) & RF24_RX_FULL) :                   # Don't feel it's necessary
                                                                                            # to be notified of new
                                                                                            # incoming packets if the RX
                                                                                            # queue is full.
                self.__irq_clear(self.__ENRF24_IRQ_RX)

         # Check if RX payload is 0-byte or >32byte (erroneous conditions)
         # Also check if data was received on pipe#0, which we are ignoring.
         # The reason for this is pipe#0 is needed for receiving AutoACK acknowledgements,
         # its address gets reset to the module's default and we do not care about data
         # coming in to that address...

            i = self.__readCmdPayload(RF24_R_RX_PL_WID, i, 1, 1)[0]
            
            if (i == 0 or i > 32 or ((self.__rf_status & 0x0E) >> 1) == 0):
                                                                                            # Zero-width RX payload is an error that happens a lot
                                                                                            # with non-AutoAck, and must be cleared with FLUSH_RX.
                                                                                            # Erroneous >32byte packets are a similar phenomenon.
                self.__issueCmd(RF24_FLUSH_RX)
                self.__irq_clear(self.__ENRF24_IRQ_RX)
                self.__readpending = 0
            else:
                self.__readpending = 1
      
            # Actual scavenging of RX queues is performed by user-directed use of read().

    def __ENRF24_CFGMASK_CRC(self, a):
        return (a & (RF24_EN_CRC | RF24_CRCO))
