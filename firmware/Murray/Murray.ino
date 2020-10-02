/*******************************************************************************
           __  ___
          /  |/  /_  ________________ ___  __
         / /|_/ / / / / ___/ ___/ __ `/ / / /
        / /  / / /_/ / /  / /  / /_/ / /_/ /
       /_/  /_/\__,_/_/  /_/   \__,_/\__, /
                                    /____/

  SNES controller adapter for Atari 5200
  Danjovic 2020 - danjovic@hotmail.com - https://hackaday.io/danjovic

  01 october 2020

  This adapter lets you play Robotron 2084 using an SNES gamepad. It also provides full keyboard control.
  The adapter is named after The 8 Bit Guy that came with the SNES layout for a game that he is developing for Commodore PET.
  The directional cross controls the movement while the X Y A B buttons control the shooting direction.

  Main features are:

    # Digital Controller on Port 1 using the D-PAD just like a Masterplay adapter;
    # Digital Controller on Port 2 using the buttons X-Y-A-B as a directional control
    # Full keypad emulation;
    # Top/Bottom mapped on Shoulder L-R buttons.
    # Auto detected second controller adapter for games like Robotron;
    # Support for NES controllers with Auto detection (keypad functionality limited, though).

  The circuit is built around an ATMega xx8 and two analog multiplexers.

  Buttons are mapped as follows:

    +----------+------SELECT-------+SEL+START+
    | Button   | Released| Pressed | Pressed |
    +----------+---------+---------+---------+
    | D-PAD UP | Port1 UP|    2    |   none  |
    | D-PAD DW | Port1 DW|    7    |   none  |
    | D-PAD LF | Port1 LF|    1    |   none  |
    | D-PAD RG | Port1 RG|    3    |   none  |
    |    Y     | Port2 UP|    5    |   none  |
    |    B     | Port2 DW|    8    |  PAUSE  |
    |    Y     | Port2 LF|    4    |  START  |
    |    A     | Port2 RG|    6    |  RESET  |
    | Top LEFT |  TOP    |    9    |    *    |
    | Top RIGHT| BOTTOM  |    0    |    #    |
    |  START   |  PAUSE  |   ---   |   ---   |
    |  SELECT  |   ---   |   ---   |   ---   |
    +----------+---------+---------+---------+

  Keypresses are emulated by activation of the following combination of signals.
  The 'none' key state is issued by rising the INHIBT signal to disabe the output MUX.

        +------- Output Mux -MSbits-----+
  Input |   7       6       5       8
   Mux  |  0 0  |  0 1  |  1 0  |  1 1  |
   B A  +-------+-------+-------+-------+
   0 0  |   1   |   4   |   7   |   *   | 3
   0 1  |   2   |   5   |   8   |   0   | 2
   1 0  |   3   |   6   |   9   |   #   | 1
   1 1  | start | pause | reset |  none | 4
        +-------+-------+-------+-------+

  CAV Voltage on PIN 9 is connected to INT0 IRQ pin and is used to promptly turn off the timing pins whenever CAV drops to zero.
  A resistor is added to protect the AVR input, as the CAV voltage can reach up to 6.4Volts.

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


 *******************************************************************************
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

*/

/*******************************************************************************
           _      __ _      _ _   _
        __| |___ / _(_)_ _ (_) |_(_)___ _ _  ___
       / _` / -_)  _| | ' \| |  _| / _ \ ' \(_-<
       \__,_\___|_| |_|_||_|_|\__|_\___/_||_/__/

*/

#include "MurrayPinout.h"

enum stickOrientation {
  _CENTER = 0,
  _NORTH,
  _NORTHEAST,
  _EAST,
  _SOUTHEAST,
  _SOUTH,
  _SOUTHWEST,
  _WEST,
  _NORTHWEST
};


enum controllerTypes {
  _SNES,
  _NTT_DATA,
  _NES,
  _UNKNOWN
};

/*******************************************************************************
                     _      _    _
       __ ____ _ _ _(_)__ _| |__| |___ ___
       \ V / _` | '_| / _` | '_ \ / -_|_-<
        \_/\__,_|_| |_\__,_|_.__/_\___/__/

*/
static volatile uint8_t CAVoff;
uint16_t combinedButtons = 0;
uint16_t nttKeys = 0;

uint8_t keyActive;         // current key active on keyboard.
uint8_t port1orientation;  // 0 center, 1 north, 2 northeast, 3 east, etc...
uint8_t port2orientation;  //
uint8_t topFire;           // state of fire buttons.
uint8_t bottomFire;           // state of fire buttons.




/*******************************************************************************
        _     _                         _
       (_)_ _| |_ ___ _ _ _ _ _  _ _ __| |_ ___
       | | ' \  _/ -_) '_| '_| || | '_ \  _(_-<
       |_|_||_\__\___|_| |_|  \_,_| .__/\__/__/
                                  |_|
*/
// Pin change interrupt, trigger by Cav line change state.
ISR (PCINT0_vect) {
  if ( CAVinput() ) {  // rising edge ?

    CAVoff = 0;    // flag that CAV voltage is present

  } else { // Falling edge

    potX1low(); // force all outputs to zero
    potY1low();
    potX2low();
    potY2low();

    CAVoff = 1; // flag that CAV voltage has dropped
  }
}


/*******************************************************************************
        ___      _
       / __| ___| |_ _  _ _ __
       \__ \/ -_)  _| || | '_ \
       |___/\___|\__|\_,_| .__/
                         |_|
*/
void setup() {
  // initialize Atari 5200 interface
  setIdleState();

  DDRcavSignal &= ~(1 << BITcavSignal); // CAV detection pin as input
  PORTcavSignal &= ~(1 << BITcavSignal); // no pullup

  DDRdetect2ndPort &= ~(1 << BITdetect2ndPort); // detection pin as input
  PORTdetect2ndPort |= (1 << BITdetect2ndPort); // activate pullup


  // initialize SNES interface
  DDRclockSignal |= (1 << BITclockSignal); // outputs
  DDRlatchSignal |= (1 << BITlatchSignal); //
  DDRdataSignal  &= ~(1 << BITdataSignal); // input

  PORTclockSignal |= (1 << BITclockSignal); // initial state High
  PORTlatchSignal &= ~(1 << BITlatchSignal); // initial state LOW
  PORTdataSignal  |= (1 << BITdataSignal);  // Pullup enabled

  // initialize interrupts
  // Setup Pin Change Interrupt for Cav
  PCICR =   (0 << PCIE0)    // Pin change interrupt enable for  PCINT0..PCINT7   at pins PB0..PB7
            | (0 << PCIE1)    // Pin change interrupt enable for  PCINT8..PCINT14  at pins PC0..PC6
            | (1 << PCIE2);   // Pin change interrupt enable for PCINT16..PCINT23  at pins PD0..PD7

  PCMSK2 =   (0 << PCINT16)  // Pin change enable for PD0 pin
             | (0 << PCINT17)  // Pin change enable for PD1 pin
             | (1 << PCINT18)  // Pin change enable for PD2 pin - Vac voltage is connected here
             | (0 << PCINT19)  // Pin change enable for PD3 pin
             | (0 << PCINT20)  // Pin change enable for PD4 pin
             | (0 << PCINT21)  // Pin change enable for PD5 pin
             | (0 << PCINT22)  // Pin change enable for PD6 pin
             | (0 << PCINT23); // Pin change enable for PD7 pin


  PCIFR =   (0 << PCIF0) // Clear any pending interrupt flag
            | (0 << PCIF1)
            | (1 << PCIF2);

  // enable interrupts
  sei();

}


/*******************************************************************************
 *      __  __      _        _                  
 *     |  \/  |__ _(_)_ _   | |   ___  ___ _ __ 
 *     | |\/| / _` | | ' \  | |__/ _ \/ _ \ '_ \
 *     |_|  |_\__,_|_|_||_| |____\___/\___/ .__/
 *                                        |_|   
 */
void loop() {

  // Scan controller and
  switch ( ScanSnesController() ) {
    case _SNES:
      mapSNESbuttons();
      break;

    case _NTT_DATA:
      mapNTTbuttons();
      break;

    case _NES:
      mapNESbuttons();

    default:
      setIdleState();
  }
  activateOutputs();
  delay(20);
}



/*******************************************************************************
 *       __              _   _             
 *      / _|_  _ _ _  __| |_(_)___ _ _  ___
 *     |  _| || | ' \/ _|  _| / _ \ ' \(_-<
 *     |_|  \_,_|_||_\__|\__|_\___/_||_/__/
 *                                         
 */
void activateOutputs() {

  /*
    uint8_t keyActive;         // current key active on keyboard.
    uint8_t port1orientation;  // 0 center, 1 north, 2 northeast, 3 east, etc...
    uint8_t port2orientation;  //
    uint8_t fireButtons;       // state of Top Bottom fire buttons.
  */
  // Activate fire buttons
  if ( topFire     )    TOPbuttonPress();     else  TOPbuttonRelease();
  if (  bottomFire ) BOTTOMbuttonPress();  else  BOTTOMbuttonRelease();

  // Activate keyboard
  if (keyActive & (1 << 3) ) setOutputMuxBHigh(); else  setOutputMuxBlow();
  if (keyActive & (1 << 2) ) setOutputMuxAHigh(); else  setOutputMuxAlow();

  if (keyActive & (1 << 1) ) setInputMuxBHigh(); else  setInputMuxBlow();
  if (keyActive & (1 << 0) ) setInputMuxAHigh(); else setInputMuxAlow();

  if ( keyActive == _keyNONE ) {
    disableMuxOutput();
  } else {
    enableMuxOutput();
  }


  // Activate controllers
  if (!CAVoff) {
    // Activate Controller 1
    switch (port1orientation) {
      case  _CENTER    :  potX1center();  potY1middle();  break;
      case  _NORTH     :  potX1center();  potY1up();      break;
      case  _NORTHEAST :  potX1right();   potY1up();      break;
      case  _EAST      :  potX1right();   potY1middle();  break;
      case  _SOUTHEAST :  potX1right();   potY1down();    break;
      case  _SOUTH     :  potX1center();  potY1down();    break;
      case  _SOUTHWEST :  potX1left();    potY1down();    break;
      case  _WEST      :  potX1left();    potY1middle();  break;
      case  _NORTHWEST :  potX1left();    potY1up();      break;
    } // switch


    // Activate Controller 2
    switch (port1orientation) {
      case  _CENTER    :  potX2center();  potY2middle();  break;
      case  _NORTH     :  potX2center();  potY2up();      break;
      case  _NORTHEAST :  potX2right();   potY2up();      break;
      case  _EAST      :  potX2right();   potY2middle();  break;
      case  _SOUTHEAST :  potX2right();   potY2down();    break;
      case  _SOUTH     :  potX2center();  potY2down();    break;
      case  _SOUTHWEST :  potX2left();    potY2down();    break;
      case  _WEST      :  potX2left();    potY2middle();  break;
      case  _NORTHWEST :  potX2left();    potY2up();      break;
    } // switch


  } // if (!CAVoff)
}


//
// Set Idle State
void setIdleState() {
  potX1center();
  potY1middle();
  potX2center();
  potY2middle();
  TOPbuttonRelease();
  BOTTOMbuttonRelease();
  disableMuxOutput();
}


//
// Map SNES controller buttons
void mapSNESbuttons() {
  // bit    15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
  // SNES   B  Y  SL ST UP DW LF RG A  X  L  R  .  .  .  .

#define SnesB         (1<<15)
#define SnesY         (1<<14)
#define SnesSELECT    (1<<13)
#define SnesSTART     (1<<12)
#define SnesUP        (1<<11)
#define SnesDOWN      (1<<10)
#define SnesLEFT      (1<< 9)
#define SnesRIGHT     (1<< 8)
#define SnesA         (1<< 7)
#define SnesX         (1<< 6)
#define SnesL         (1<< 5)
#define SnesR         (1<< 4)

  // initialize mapped variables
  keyActive        = _keyNONE;
  port1orientation = _CENTER;
  port2orientation = _CENTER;
  topFire          = 0;
  bottomFire       = 0;

  // clear unused bits
  combinedButtons &= 0xfff0;

  // Check for modifier key
  if ( combinedButtons & SnesSELECT ) {  // 1st Alternate key
    if ( combinedButtons & SnesSTART ) {  // 2nd Alternate key
      // Process second alternate functions

      // clear both Alternate bits to not jeopardize the processing
      combinedButtons &= ~( SnesSELECT | SnesSTART );

      switch (combinedButtons) {
        case SnesL:    keyActive = _keyASTERISK; break;
        case SnesR:    keyActive = _keyHASH;     break;
        case SnesY:    keyActive = _keySTART;    break;
        case SnesA:    keyActive = _keyRESET;    break;
        case SnesB:    keyActive = _keyPAUSE;    break;
      } // switch

      //  (end of processing second alternate functions)
    } else {
      // process first alternate functions

      // clear the Alternate bit to not jeopardize the processing
      combinedButtons &= ~SnesSELECT;

      switch (combinedButtons) {
        case SnesUP:    keyActive = _key2;        break;
        case SnesDOWN:  keyActive = _key7;        break;
        case SnesLEFT:  keyActive = _key1;        break;
        case SnesRIGHT: keyActive = _key3;        break;
        case SnesL:     keyActive = _key9;        break;
        case SnesR:     keyActive = _key0;        break;
        case SnesX:     keyActive = _key5;        break;
        case SnesY:     keyActive = _key4;        break;
        case SnesA:     keyActive = _key6;        break;
        case SnesB:     keyActive = _key8;        break;

      } // switch
    } // else (end of processing first alternate functions)

  } else {
    // process normal function (no alternate key)

    // process keys
    if ( combinedButtons & SnesSTART) keyActive = _keyPAUSE;

    // process directionals for the first port
    switch (combinedButtons & (SnesUP | SnesDOWN | SnesLEFT | SnesRIGHT) ) {
      case (SnesUP              ):  port1orientation = _NORTH;      break;
      case (SnesUP   | SnesRIGHT):  port1orientation = _NORTHEAST;  break;
      case (           SnesRIGHT):  port1orientation = _EAST;       break;
      case (SnesDOWN | SnesRIGHT):  port1orientation = _SOUTHEAST;  break;
      case (SnesDOWN            ):  port1orientation = _SOUTH;      break;
      case (SnesDOWN | SnesLEFT ):  port1orientation = _SOUTHWEST;  break;
      case (           SnesLEFT ):  port1orientation = _WEST;       break;
      case (SnesUP   | SnesLEFT ):  port1orientation = _NORTHWEST;  break;
    } // switch

    // check if the adapter is connected to the second port
    if (secondPortConnected() ) {

      // preocess directionals for the second port
      switch (combinedButtons & (SnesX  | SnesY   | SnesA   | SnesB    ) ) {
        case (SnesX               ):  port2orientation = _NORTH;      break;
        case (SnesX    | SnesA    ):  port2orientation = _NORTHEAST;  break;
        case (           SnesA    ):  port2orientation = _EAST;       break;
        case (SnesB    | SnesA    ):  port2orientation = _SOUTHEAST;  break;
        case (SnesB               ):  port2orientation = _SOUTH;      break;
        case (SnesB    | SnesY    ):  port2orientation = _SOUTHWEST;  break;
        case (           SnesY    ):  port2orientation = _WEST;       break;
        case (SnesX    | SnesY    ):  port2orientation = _NORTHWEST;  break;
      } // switch


    } else { // Second port not connected,

      // process buttons X Y A B as fire buttons
      if ( combinedButtons & SnesX) topFire |= 1 ;
      if ( combinedButtons & SnesA) topFire |= 1 ;
      if ( combinedButtons & SnesY) bottomFire |=1;
      if ( combinedButtons & SnesB) bottomFire |=1;
    }

    // process fire buttons
    if ( combinedButtons & SnesL) topFire |= 1 ;
    if ( combinedButtons & SnesR) bottomFire |=1;


  } // else  (end or processing normal functions)

}


//
// Map NTT controller buttons
void mapNTTbuttons() {
  // bit    15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
  // BTNS   B  Y  SL ST UP DW LF RG A  X  L  R  .  .  .  .
  // KEYS   0  1  2  3  4  5  6  7  8  9  *  #  .  C  `  Eot
  #define nttB         (1<<15)
  #define nttY         (1<<14)
  #define nttPREVIOUS  (1<<13)
  #define nttNEXT      (1<<12)
  #define nttUP        (1<<11)
  #define nttDOWN      (1<<10)
  #define nttLEFT      (1<< 9)
  #define nttRIGHT     (1<< 8)
  #define nttA         (1<< 7)
  #define nttX         (1<< 6)
  #define nttL         (1<< 5)
  #define nttR         (1<< 4)
  
  #define ntt0         (1<<15)
  #define ntt1         (1<<14)
  #define ntt2         (1<<13)
  #define ntt3         (1<<12)
  #define ntt4         (1<<11)
  #define ntt5         (1<<10)
  #define ntt6         (1<< 9)
  #define ntt7         (1<< 8)
  #define ntt8         (1<< 7)
  #define ntt9         (1<< 6)
  #define nttASTERISK  (1<< 5)
  #define nttHASH      (1<< 4)
  #define nttDOT       (1<< 3)
  #define nttC         (1<< 2)
  #define nttDUMMY     (1<< 1)
  #define nttEOT       (1<< 0)

  // initialize mapped variables
  keyActive        = _keyNONE;
  port1orientation = _CENTER;
  port2orientation = _CENTER;
  topFire          = 0;
  bottomFire       = 0;

  // clear unused bits
  combinedButtons &= 0xfff0;
  nttKeys         &= ~nttDUMMY;

  // substitute dummy key - if both PREVIOUS and NEXT are pressed set bit to flat a RESET key
  if ( (combinedButtons & (nttPREVIOUS | nttNEXT)) == (nttPREVIOUS | nttNEXT) ) nttKeys |= nttDUMMY;

  // Process Keyboard
  switch (nttKeys) {
    case ntt0:          keyActive = _key0;        break;
    case ntt1:          keyActive = _key1;        break;
    case ntt2:          keyActive = _key2;        break;
    case ntt3:          keyActive = _key3;        break;
    case ntt4:          keyActive = _key4;        break;
    case ntt5:          keyActive = _key5;        break;
    case ntt6:          keyActive = _key6;        break;
    case ntt7:          keyActive = _key7;        break;
    case ntt8:          keyActive = _key8;        break;
    case ntt9:          keyActive = _key9;        break;
    case nttASTERISK:   keyActive = _keyASTERISK; break;
    case nttHASH:       keyActive = _keyHASH;     break;
    //  case nttDOT:        keyActive = _keyNONE;     break;
    case nttC:          keyActive = _keyPAUSE;    break;
    case nttDUMMY:      keyActive = _keyRESET;    break;
    case nttEOT:        keyActive = _keySTART;    break;
  } // switch


  // process directionals for the first port
  switch (combinedButtons & (nttUP | nttDOWN | nttLEFT | nttRIGHT) ) {
    case (nttUP             ):  port1orientation = _NORTH;      break;
    case (nttUP   | nttRIGHT):  port1orientation = _NORTHEAST;  break;
    case (          nttRIGHT):  port1orientation = _EAST;       break;
    case (nttDOWN | nttRIGHT):  port1orientation = _SOUTHEAST;  break;
    case (nttDOWN           ):  port1orientation = _SOUTH;      break;
    case (nttDOWN | nttLEFT ):  port1orientation = _SOUTHWEST;  break;
    case (          nttLEFT ):  port1orientation = _WEST;       break;
    case (nttUP   | nttLEFT ):  port1orientation = _NORTHWEST;  break;
  } // switch

  // check if the adapter is connected to the second port
  if (secondPortConnected() ) {

    // preocess directionals for the second port
    switch (combinedButtons & (nttX  | nttY   | nttA   | nttB    ) ) {
      case (nttX              ):  port2orientation = _NORTH;      break;
      case (nttX    | nttA    ):  port2orientation = _NORTHEAST;  break;
      case (          nttA    ):  port2orientation = _EAST;       break;
      case (nttB    | nttA    ):  port2orientation = _SOUTHEAST;  break;
      case (nttB              ):  port2orientation = _SOUTH;      break;
      case (nttB    | nttY    ):  port2orientation = _SOUTHWEST;  break;
      case (          nttY    ):  port2orientation = _WEST;       break;
      case (nttX    | nttY    ):  port2orientation = _NORTHWEST;  break;
    } // switch


  } else { // Second port not connected,

    // process buttons X Y A B as fire buttons
    if ( combinedButtons & nttX) topFire |= 1 ;
    if ( combinedButtons & nttA) topFire |= 1 ;
    if ( combinedButtons & nttY) bottomFire |=1;
    if ( combinedButtons & nttB) bottomFire |=1;
  }

  // process fire buttons
  if ( combinedButtons & nttL) topFire |= 1 ;
  if ( combinedButtons & nttR) bottomFire |=1;
 
}


//
// Map NES buttons
void mapNESbuttons() {
  // bit    15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
  // NES    A  B  SL ST UP DW LF RG .  .  .  .  .  .  .  .

#define nesA         (1<<15)
#define nesB         (1<<14)
#define nesSELECT    (1<<13)
#define nesSTART     (1<<12)
#define nesUP        (1<<11)
#define nesDOWN      (1<<10)
#define nesLEFT      (1<< 9)
#define nesRIGHT     (1<< 8)

  // initialize mapped variables
  keyActive        = _keyNONE;
  port1orientation = _CENTER;
  port2orientation = _CENTER;
  topFire         = 0;
  bottomFire      = 0;


  // clear unused bits
  combinedButtons &= 0xff00;

  // Check for modifier key
  if ( combinedButtons & nesSELECT ) {  // Alternate functions
    // clear the Alternate bit to not jeopardize the processing
    combinedButtons &= ~nesSELECT;

    //process key
    switch (combinedButtons) {
      case nesUP:    keyActive = _key2;        break;
      case nesDOWN:  keyActive = _keyRESET;    break;
      case nesLEFT:  keyActive = _key1;        break;
      case nesRIGHT: keyActive = _key3;        break;
      case nesSTART: keyActive = _keySTART;    break;
      case nesA:     keyActive = _keyHASH;     break;
      case nesB:     keyActive = _keyASTERISK; break;
    } // switch
  } else { // Normal function
    // process keys
    if ( combinedButtons & nesSTART) keyActive = _keyPAUSE;

    // process directionals
    switch (combinedButtons & (nesUP | nesDOWN | nesLEFT | nesRIGHT) ) {
      case (nesUP             ):  port1orientation = _NORTH;      break;
      case (nesUP   | nesRIGHT):  port1orientation = _NORTHEAST;  break;
      case (          nesRIGHT):  port1orientation = _EAST;       break;
      case (nesDOWN | nesRIGHT):  port1orientation = _SOUTHEAST;  break;
      case (nesDOWN           ):  port1orientation = _SOUTH;      break;
      case (nesDOWN | nesLEFT ):  port1orientation = _SOUTHWEST;  break;
      case (          nesLEFT ):  port1orientation = _WEST;       break;
      case (nesUP   | nesLEFT ):  port1orientation = _NORTHWEST;  break;
    } // switch

    // process buttons
    if ( combinedButtons & nesA) topFire |= 1 ;
    if ( combinedButtons & nesB) bottomFire |=1;

  }
}


//
// Pulse clock line.
inline void pulseClock() {
  delayMicroseconds(6);
  clockHigh();
  delayMicroseconds(6);
  clockLow();
}

//
// Scan SnesController
controllerTypes ScanSnesController() {

  uint8_t i, controllerType;
  uint16_t dataReceived;

  //latch buttons state. After latching, first button is ready at data output
  clockHigh();
  latchHigh();
  delayMicroseconds(12);
  clockLow();
  latchLow();

  // read 16 bits from the controller
  dataReceived = 0;
  for (i = 0; i < 16; i++) {
    dataReceived <<= 1;
    if ( dataIn() ) dataReceived |= (1 << 0) ; // shift in one bit
    pulseClock();
  }

  // bits inverted at reception
  // bit    15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00  17th
  // SNES   B  Y  SL ST UP DW LF RG A  X  L  R  0  0  0  0    1
  // NTT    B  Y  SL ST UP DW LF RG A  X  L  R  0  1  0  0   "0"
  // NES o  A  B  SL ST UP DW LF RG 1  1  1  1  1  1  1  1    1   o = Original)
  // NES k  A  B  SL ST UP DW LF RG 0  0  0  0  0  0  0  0    0   k = knockoff/clone



  // Test for NTT data keypad
  if ( (dataReceived  & 0x000f) == 0x0004 ) {
    combinedButtons = dataReceived;
    // read further 16 bits from the controller
    dataReceived = 0;
    for (i = 0; i < 16; i++) {
      dataReceived <<= 1;
      if ( dataIn() ) dataReceived |= (1 << 0) ; // shift in one bit
      pulseClock();
    } // for
    nttKeys = dataReceived;

    return _NTT_DATA;

  } else if ( dataIn() ) { // 17th bit received should be active on original controllers
    if ( (dataReceived & 0x000f) == 0x0000)
      return _SNES;
    else  if ( (dataReceived & 0x00ff) == 0x00ff)
      return _NES;
  } //


  return _UNKNOWN;  // anything else return unknown
}
