#* nRF24L01.h
#* Register definitions for manipulating the Nordic Semiconductor
#* nRF24L01+ RF transceiver chipsets.
#*
#
#    Copyright (c) 2007 Stefan Engelke <mbox@stefanengelke.de>
#    Some parts copyright (c) 2012 Eric Brundick <spirilis [at] linux dot com>
#
#    Permission is hereby granted, free of charge, to any person 
#    obtaining a copy of this software and associated documentation 
#    files (the "Software"), to deal in the Software without 
#    restriction, including without limitation the rights to use, copy, 
#    modify, merge, publish, distribute, sublicense, and/or sell copies 
#    of the Software, and to permit persons to whom the Software is 
#    furnished to do so, subject to the following conditions:
#
#    The above copyright notice and this permission notice shall be 
#    included in all copies or substantial portions of the Software.
#
#    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
#    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
#    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
#    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
#    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
#    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
#    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
#    DEALINGS IN THE SOFTWARE.
#
#    Edited: J.Brian 3-24-16
#*


# Register Map 
RF24_CONFIG      = 0x00
RF24_EN_AA       = 0x01
RF24_EN_RXADDR   = 0x02
RF24_SETUP_AW    = 0x03
RF24_SETUP_RETR  = 0x04
RF24_RF_CH       = 0x05
RF24_RF_SETUP    = 0x06
RF24_STATUS      = 0x07
RF24_OBSERVE_TX  = 0x08
RF24_CD          = 0x09
RF24_RPD         = 0x09
RF24_RX_ADDR_P0  = 0x0A
RF24_RX_ADDR_P1  = 0x0B
RF24_RX_ADDR_P2  = 0x0C
RF24_RX_ADDR_P3  = 0x0D
RF24_RX_ADDR_P4  = 0x0E
RF24_RX_ADDR_P5  = 0x0F
RF24_TX_ADDR     = 0x10
RF24_RX_PW_P0    = 0x11
RF24_RX_PW_P1    = 0x12
RF24_RX_PW_P2    = 0x13
RF24_RX_PW_P3    = 0x14
RF24_RX_PW_P4    = 0x15
RF24_RX_PW_P5    = 0x16
RF24_FIFO_STATUS = 0x17
RF24_DYNPD       = 0x1C
RF24_FEATURE     = 0x1D

# Register Bits
BITS = {"BIT0": 0x0001, "BIT1": 0x0002, "BIT2": 0x0004, "BIT3": 0x0008,
        "BIT4": 0x0010, "BIT5": 0x0020, "BIT6": 0x0040, "BIT7": 0x0080,
        "BIT8": 0x0100, "BIT9": 0x0200, "BITA": 0x0400, "BITB": 0x0800,
        "BITC": 0x1000, "BITD": 0x2000, "BITE": 0x4000, "BITF": 0x8000}

RF24_MASK_RX_DR  = BITS["BIT6"]
RF24_MASK_TX_DS  = BITS["BIT5"]
RF24_MASK_MAX_RT = BITS["BIT4"]
RF24_EN_CRC      = BITS["BIT3"]
RF24_CRCO        = BITS["BIT2"]
RF24_PWR_UP      = BITS["BIT1"]
RF24_PRIM_RX     = BITS["BIT0"]
RF24_ENAA_P5     = BITS["BIT5"]
RF24_ENAA_P4     = BITS["BIT4"]
RF24_ENAA_P3     = BITS["BIT3"]
RF24_ENAA_P2     = BITS["BIT2"]
RF24_ENAA_P1     = BITS["BIT1"]
RF24_ENAA_P0     = BITS["BIT0"]
RF24_ERX_P5      = BITS["BIT5"]
RF24_ERX_P4      = BITS["BIT4"]
RF24_ERX_P3      = BITS["BIT3"]
RF24_ERX_P2      = BITS["BIT2"]
RF24_ERX_P1      = BITS["BIT1"]
RF24_ERX_P0      = BITS["BIT0"]
RF24_AW          = BITS["BIT0"]
RF24_ARD         = BITS["BIT4"]
RF24_ARC         = BITS["BIT0"]
RF24_PLL_LOCK    = BITS["BIT4"]
RF24_CONT_WAVE   = BITS["BIT7"]
RF24_RF_DR       = BITS["BIT3"]
RF24_RF_DR_LOW   = BITS["BIT5"]
RF24_RF_DR_HIGH  = BITS["BIT3"]
RF24_RF_PWR      = BITS["BIT1"]
RF24_LNA_HCURR   = BITS["BIT0"]
RF24_RX_DR       = BITS["BIT6"]
RF24_TX_DS       = BITS["BIT5"]
RF24_MAX_RT      = BITS["BIT4"]
RF24_RX_P_NO     = BITS["BIT1"]
RF24_TX_FULL     = BITS["BIT0"]
RF24_PLOS_CNT    = BITS["BIT4"]
RF24_ARC_CNT     = BITS["BIT0"]
RF24_TX_REUSE    = BITS["BIT6"]
RF24_FIFO_FULL   = BITS["BIT5"]
RF24_TX_EMPTY    = BITS["BIT4"]
RF24_RX_FULL     = BITS["BIT1"]
RF24_RX_EMPTY    = BITS["BIT0"]
RF24_EN_DPL      = BITS["BIT2"]
RF24_EN_ACK_PAY  = BITS["BIT1"]
RF24_EN_DYN_ACK  = BITS["BIT0"]

# Instructions 
RF24_R_REGISTER    = 0x00
RF24_W_REGISTER    = 0x20
RF24_REGISTER_MASK = 0x1F
RF24_R_RX_PAYLOAD  = 0x61
RF24_W_TX_PAYLOAD  = 0xA0
RF24_FLUSH_TX      = 0xE1
RF24_FLUSH_RX      = 0xE2
RF24_REUSE_TX_PL   = 0xE3
RF24_R_RX_PL_WID   = 0x60
RF24_W_ACK_PAYLOAD = 0xA8
RF24_W_TX_PAYLOAD_NOACK = 0xB0
RF24_NOP           = 0xFF

# Constants for speed, radio state 
ENRF24_STATE_NOTPRESENT = 0
ENRF24_STATE_DEEPSLEEP  = 1
ENRF24_STATE_IDLE       = 2
ENRF24_STATE_PTX        = 3
ENRF24_STATE_PRX        = 4
