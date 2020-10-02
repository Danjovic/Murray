#ifndef __MurrayPinout_h__
#define __MurrayPinout_h__
/*
  Murray - SNES controller adapter for Atari 5200
  Danjovic 2020 - danjovic@hotmail.com - https://hackaday.io/danjovic

  01 october 2020

  Pinout:

  FUNCTION  ARDUINO AVR                       AVR  ARDUINO  FUNCTION
                            +---\__/---+
                    RST  [ 1]          [28]   PC5    A5/D19   HALF-X1
   POT-Y2     D0    PD0  [ 2]          [27]   PC4    A4/D18   POT-X1
   HALF-Y2    D1    PD1  [ 3]          [26]   PC3    A3/D17   DETECT PORT2
   VAC        D2    PD2  [ 4]INT0      [25]   PC2    A2/D16   MUX-ENABLE
   POT-X2     D3    PD3  [ 5]          [24]   PC1    A1/D15   O-MUX-A
   HALF-X2    D4    PD4  [ 6]          [23]   PC0    A0/D14   O-MUX-B
                    VCC  [ 7]          [22]   GND
                    GND  [ 8]          [21]   AREF
                    XT1  [ 9]          [20]   AVCC
                    XT2  [10]          [19]   PB5    D13      BOTTOM
  SNES CLOCK  D5    PD5  [11]          [18]   PB4    D13      TOP
  SNES LATCH  D6    PD6  [12]          [17]   PB3    D11      I-MUX-B
  SNES DATA   D7    PD7  [12]          [16]   PB2    D10      I-MUX-A
   POT-Y1     D8    PB0  [14]          [15]   PB1    D9       HALF-Y1
                            +----------+
*/


////////////////////////////////////////////////////////////////////////////////
//
//  SNES Controller Interface
//
#define DDRclockSignal  DDRD
#define DDRlatchSignal  DDRD
#define DDRdataSignal   DDRD

#define PORTclockSignal PORTD
#define PORTlatchSignal PORTD
#define PORTdataSignal  PORTD

#define PINdataSignal   PIND

#define BITclockSignal 5
#define BITlatchSignal 6
#define BITdataSignal  7

#define latchHigh()  PORTlatchSignal|= (1<<BITlatchSignal)
#define latchLow()   PORTlatchSignal&=~(1<<BITlatchSignal)

#define clockHigh()  PORTclockSignal|= (1<<BITclockSignal)
#define clockLow()   PORTclockSignal&=~(1<<BITclockSignal)

#define dataIn()     ((PINdataSignal & (1<<BITdataSignal))==0)


////////////////////////////////////////////////////////////////////////////////
//
//  ATARI 5200 Interface
//

//
// Buttons
#define DDRtopButton     DDRB
#define DDRbottomButton  DDRB

#define PORTtopButton    PORTB
#define PORTbottomButton PORTB

#define BITtopButton     4
#define BITbottomButton  5

#define TOPbuttonPress()    do { DDRtopButton|= (1<<BITtopButton); PORTtopButton&=~(1<<BITtopButton); } while (0)
#define TOPbuttonRelease()  do { DDRtopButton&=~(1<<BITtopButton); PORTtopButton|= (1<<BITtopButton); } while (0)

#define BOTTOMbuttonPress()    do { DDRbottomButton|= (1<<BITbottomButton); PORTbottomButton&=~(1<<BITbottomButton); } while (0)
#define BOTTOMbuttonRelease()  do { DDRbottomButton&=~(1<<BITbottomButton); PORTbottomButton|= (1<<BITbottomButton); } while (0)

//
// Axes
#define DDRpotX1 DDRC
#define DDRpotY1 DDRB
#define DDRpotX2 DDRD
#define DDRpotY2 DDRD

#define DDRhalfX1 DDRC
#define DDRhalfY1 DDRB
#define DDRhalfX2 DDRD
#define DDRhalfY2 DDRD

#define PORTpotX1 PORTC
#define PORTpotY1 PORTB
#define PORTpotX2 PORTD
#define PORTpotY2 PORTC

#define PORThalfX1 PORTC
#define PORThalfY1 PORTB
#define PORThalfX2 PORTD
#define PORThalfY2 PORTC

#define BITpotX1 4
#define BIThalfX1 5
#define BITpotY1 0
#define BIThalfY1 1
#define BITpotX2 3
#define BIThalfX2 4
#define BITpotY2 0
#define BIThalfY2 1

// Port 1 Macros
#define potX1high()  do { DDRpotX1|= (1<<BITpotX1); PORTpotX1|= (1<<BITpotX1); } while (0)
#define potX1open()  do { DDRpotX1&=~(1<<BITpotX1); PORTpotX1&=~(1<<BITpotX1); } while (0)
#define potX1low()   do { DDRpotX1|= (1<<BITpotX1); PORTpotX1&=~(1<<BITpotX1); } while (0)

#define halfX1high()  do { DDRhalfX1|= (1<<BIThalfY1); PORThalfX1|= (1<<BIThalfY1); } while (0)
#define halfX1open()  do { DDRhalfX1&=~(1<<BIThalfY1); PORThalfX1&=~(1<<BIThalfY1); } while (0)

#define potY1high()  do { DDRpotY1|= (1<<BITpotY1); PORTpotY1|= (1<<BITpotY1); } while (0)
#define potY1open()  do { DDRpotY1&=~(1<<BITpotY1); PORTpotY1&=~(1<<BITpotY1); } while (0)
#define potY1low()   do { DDRpotY1|= (1<<BITpotY1); PORTpotY1&=~(1<<BITpotY1); } while (0)

#define halfY1high()  do { DDRhalfY1|= (1<<BIThalfY1); PORThalfY1|= (1<<BIThalfY1); } while (0)
#define halfY1open()  do { DDRhalfY1&=~(1<<BIThalfY1); PORThalfY1&=~(1<<BIThalfY1); } while (0)

// Port 2 Macros
#define potX2high()  do { DDRpotX2|= (1<<BITpotX2); PORTpotX2|= (1<<BITpotX2); } while (0)
#define potX2open()  do { DDRpotX2&=~(1<<BITpotX2); PORTpotX2&=~(1<<BITpotX2); } while (0)
#define potX2low()   do { DDRpotX2|= (1<<BITpotX2); PORTpotX2&=~(1<<BITpotX2); } while (0)

#define halfX2high()  do { DDRhalfX2|= (1<<BIThalfX2); PORThalfX2|= (1<<BIThalfX2); } while (0)
#define halfX2open()  do { DDRhalfX2&=~(1<<BIThalfX2); PORThalfX2&=~(1<<BIThalfX2); } while (0)

#define potY2high()  do { DDRpotY2|= (1<<BITpotY2); PORTpotY2|= (1<<BITpotY2); } while (0)
#define potY2open()  do { DDRpotY2&=~(1<<BITpotY2); PORTpotY2&=~(1<<BITpotY2); } while (0)
#define potY2low()   do { DDRpotY2|= (1<<BITpotY2); PORTpotY2&=~(1<<BITpotY2); } while (0)

#define halfY2high()  do { DDRhalfY2|= (1<<BIThalfY2); PORThalfY2|= (1<<BIThalfY2); } while (0)
#define halfY2open()  do { DDRhalfY2&=~(1<<BIThalfY2); PORThalfY2&=~(1<<BIThalfY2); } while (0)


// Potentiometer position Macros
#define potX1left()    do { potX1high(); halfX1open(); } while (0)
#define potX1center()  do { potX1open(); halfX1high(); } while (0)
#define potX1right()   do { potX1open(); halfX1open(); } while (0)

#define potY1up()      do { potY1high(); halfY1open(); } while (0)
#define potY1middle()  do { potY1open(); halfY1high(); } while (0)
#define potY1down()    do { potY1open(); halfY1open(); } while (0)

#define potX2left()    do { potX2high(); halfX2open(); } while (0)
#define potX2center()  do { potX2open(); halfX2high(); } while (0)
#define potX2right()   do { potX2open(); halfX2open(); } while (0)

#define potY2up()      do { potY2high(); halfY2open(); } while (0)
#define potY2middle()  do { potY2open(); halfY2high(); } while (0)
#define potY2down()    do { potY2open(); halfY2open(); } while (0)

////////////////////////////////////////////////////////////////////////////////
//
//  MUX Controller Interface
//
#define DDRinputMuxA  DDRB
#define DDRinputMuxB  DDRB
#define DDRoutputMuxA DDRC
#define DDRoutputMuxB DDRC
#define DDRenableMux  DDRC

#define PORTinputMuxA  PORTB
#define PORTinputMuxB  PORTB
#define PORToutputMuxA PORTC
#define PORToutputMuxB PORTC
#define PORToutputMux  PORTC

#define BITinputMuxA  3
#define BITinputMuxB  2
#define BIToutputMuxA 1
#define BIToutputMuxB 0
#define BITenableMux  2


#define setInputMuxAlow()   PORTinputMuxA &= ~(1<<BITinputMuxA)
#define setInputMuxAHigh()  PORTinputMuxA |=  (1<<BITinputMuxA)

#define setInputMuxBlow()   PORTinputMuxB &= ~(1<<BITinputMuxB)
#define setInputMuxBHigh()  PORTinputMuxB |=  (1<<BITinputMuxB)

#define setOutputMuxAlow()   PORToutputMuxA &= ~(1<<BIToutputMuxA)
#define setOutputMuxAHigh()  PORToutputMuxA |=  (1<<BIToutputMuxA)

#define setOutputMuxBlow()   PORToutputMuxB &= ~(1<<BIToutputMuxB)
#define setOutputMuxBHigh()  PORToutputMuxB |=  (1<<BIToutputMuxB)

#define enableMuxOutput() PORToutputMux &= ~(1<<BITenableMux)
#define disableMuxOutput() PORToutputMux |= (1<<BITenableMux)

#define _key1        ((0<<3)|(0<<2)|(0<<1)|(0<<0)) // 0
#define _key2        ((0<<3)|(0<<2)|(0<<1)|(1<<0)) // 1
#define _key3        ((0<<3)|(0<<2)|(1<<1)|(0<<0)) // 2
#define _keySTART    ((0<<3)|(0<<2)|(1<<1)|(1<<0)) // 3
#define _key4        ((0<<3)|(1<<2)|(0<<1)|(0<<0)) // 4
#define _key5        ((0<<3)|(1<<2)|(0<<1)|(1<<0)) // 5
#define _key6        ((0<<3)|(1<<2)|(1<<1)|(0<<0)) // 6
#define _keyPAUSE    ((0<<3)|(1<<2)|(1<<1)|(1<<0)) // 7
#define _key7        ((1<<3)|(0<<2)|(0<<1)|(0<<0)) // 8
#define _key8        ((1<<3)|(0<<2)|(0<<1)|(1<<0)) // 9
#define _key9        ((1<<3)|(0<<2)|(1<<1)|(0<<0)) // 10
#define _keyRESET    ((1<<3)|(0<<2)|(1<<1)|(1<<0)) // 11
#define _keyASTERISK ((1<<3)|(1<<2)|(0<<1)|(0<<0)) // 12
#define _key0        ((1<<3)|(1<<2)|(0<<1)|(1<<0)) // 13
#define _keyHASH     ((1<<3)|(1<<2)|(1<<1)|(0<<0)) // 14
#define _keyNONE     ((1<<3)|(1<<2)|(1<<1)|(1<<0)) // 15

////////////////////////////////////////////////////////////////////////////////
//
// Voltage Controller Interface (Cav)
//
#define DDRcavSignal  DDRD
#define PORTcavSignal PORTD
#define PINcavSignal  PIND
#define BITcavSignal 2

#define CAVinput()     (PINcavSignal & (1<<BITcavSignal))

////////////////////////////////////////////////////////////////////////////////
//
// Second port connection detection
//

#define DDRdetect2ndPort  DDRC
#define PORTdetect2ndPort  PORTC
#define PINdetect2ndPort   PINC
#define BITdetect2ndPort  3

#define secondPortConnected()  ( (PINdetect2ndPort & (1<<BITdetect2ndPort)) == 0)

#endif // __MurrayPinout_h__
