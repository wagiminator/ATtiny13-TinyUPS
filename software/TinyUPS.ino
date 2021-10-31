// ===================================================================================
// Project:   tinyUPS - Simple Uninterruptible Power Supply based on ATtiny13A
// Version:   v1.2
// Year:      2020
// Author:    Stefan Wagner
// Github:    https://github.com/wagiminator
// EasyEDA:   https://easyeda.com/wagiminator
// License:   http://creativecommons.org/licenses/by-sa/3.0/
// ===================================================================================
//
// Description:
// ------------
// If external power is connected to the tinyUPS the input voltage or vcc of the
// ATtiny13 is delivered by this source, otherwise by the battery. The ATtiny13
// monitors the input voltage and tells the connected device to shutdown by pulling
// the SHUTDOWN-line low when the input voltage falls below a certain threshold
// (SHUTDOWNLEVEL). This happens when the external power source is diconnected or
// disbled and the battery level falls below this threshold.
// After waiting a certain time (SHUTDOWNTIMER) to allow the connected device to
// safely shut down, the ATtiny13 deactivates the boost converter and turns off the
// power to the connected device.
// If the input voltage rises again above a certain threshold (POWERONLEVEL) it
// activates the boost converter and turns on the power to the connected device. This
// happens when the external power source is available again.
// When power is turned on a BOOTUPTIMER starts to count. If a shutdown is initiated
// before the boot up is completed, the left-over time is added to the SHUTDOWNTIMER
// in order to allow the connected device to completely boot up and shut down.
// A shutdown can also be initiated by pressing and holding the button or by setting
// the REQUEST-line to high (>0.7V) for 2 seconds. After such shutdowns the power will
// not be turned on again automatically. The power to the connected device can be
// turned on manually by pressing the button or setting the REQUEST-line to high if
// the battery level is above a certain threshold (USERPOWERLEVEL) or the external
// power source is connected.
//
// The ATtiny13 spends most of the time in power-down sleep mode to save energy. The
// watch dog timer wakes it up every 8 seconds. It will also wake up if the button was
// pressed or the REQUEST-line was changed (pin change interrupt). After doing its
// stuff the ATtiny13 sleeps again.
//
// Status LED:
// - steady on:       normal power-on operation
// - blinking:        shutdown sequence
// - short flashes:   in power-off mode (short flash occurs every 8 seconds)
//
// Note:  Logic levels on SHUTDOWN and REQUEST pins are inverted to the actual lines
//        which are connected to the device. Connected device must have pullup
//        resistor on SHUTDOWN line! This is neccessary because of the different
//        voltage levels.
//
// Wiring:
// -------
//                           +-\/-+
//        --- RST ADC0 PB5  1|°   |8  Vcc
//    LED ------- ADC3 PB3  2|    |7  PB2 ADC1 -------- VOLTAGE SENSE
// ENABLE ------- ADC2 PB4  3|    |6  PB1 AIN1 OC0B --- REQUEST/BUTTON
//                     GND  4|    |5  PB0 AIN0 OC0A --- SHUTDOWN
//                           +----+
//
// Compilation Settings:
// ---------------------
// Controller:  ATtiny13A
// Core:        MicroCore (https://github.com/MCUdude/MicroCore)
// Clockspeed:  1.2 MHz internal
// BOD:         BOD disabled
// Timing:      Micros disabled
//
// Leave the rest on default settings. Don't forget to "Burn bootloader"!
// No Arduino core functions or libraries are used. Use the makefile if 
// you want to compile without Arduino IDE.
//
// Fuse settings: -U lfuse:w:0x2a:m -U hfuse:w:0xff:m


// ===================================================================================
// Libraries and Definitions
// ===================================================================================

// Libraries
#include <avr/io.h>             // for gpio
#include <avr/sleep.h>          // for the sleep modes
#include <avr/interrupt.h>      // for the interrupts
#include <avr/wdt.h>            // for the watch dog timer
#include <util/delay.h>         // for delays

// Pin definitions
#define SHUTDOWN        PB0     // when set high the SHUTDOWN line is pulled low
#define REQUEST         PB1     // gets low on button pressed or REQ-line high
#define SENSE           PB2     // voltage divider for measuring vcc
#define LED             PB3     // status LED: on when high
#define ENABLE          PB4     // enable pin of boost converter: on when high

// Control parameters
#define SHUTDOWNLEVEL   3000    // supply voltage threshold in mV for auto shutdown
#define USERPOWERLEVEL  3500    // supply voltage when user is allowed to power on  
#define POWERONLEVEL    4300    // supply voltage threshold in mV for auto power on
#define BOOTUPTIMER     60      // time in seconds the connected device needs to boot up
#define SHUTDOWNTIMER   30      // time in seconds the connected device needs to shut down
#define REQUESTTIMER    20      // duration in 100ms request/button has to be low to shut down

// Watchdog timer intervals
#define WDT1S   (1<<WDTIE)|(1<<WDP2)|(1<<WDP1)  // WDTCR value for 1 second
#define WDT8S   (1<<WDTIE)|(1<<WDP3)|(1<<WDP0)  // WDTCR value for 8 seconds

// Pin manipulation macros
#define pinOutput(x)  DDRB  |=  (1<<(x))                  // set pin to OUTPUT
#define pinLow(x)     PORTB &= ~(1<<(x))                  // set pin to LOW
#define pinHigh(x)    PORTB |=  (1<<(x))                  // set pin to HIGH
#define pinToggle(x)  PINB  |=  (1<<(x))                  // TOGGLE pin
#define pinDisable(x) DIDR0 |=  (1<<(x))                  // disable digital input buffer
#define pinIntEn(x)   PCMSK |=  (1<<(x))                  // enable pin change interrupt
#define pinRead(x)    (PINB &   (1<<(x)))                 // READ pin
#define pinADC(x)     ((x)==2?1:((x)==3?3:((x)==4?2:0)))  // convert pin to ADC port

// ===================================================================================
// ADC Implementation
// ===================================================================================

// Init ADC
void ADC_init(void) {
  ADCSRA  = (1<<ADPS0)|(1<<ADPS1);      // set ADC clock prescaler to 8
  ADCSRA |= (1<<ADIE);                  // ADC interrupts enable
}

// Get supply voltage in mV by reading it via voltage divider against 1.1V reference
uint16_t getVcc(void) {
  uint16_t result = 0;                  // result
  ADCSRA |= (1<<ADEN)|(1<<ADIF);        // enable ADC, turn off any pending interrupt
  ADMUX   = (1<<REFS0)|(pinADC(SENSE)); // set SENSE against 1.1V reference
  _delay_ms(2);                         // wait for Vref to settle
  set_sleep_mode (SLEEP_MODE_ADC);      // sleep during sample for noise reduction
  for(uint8_t i=21; i; i--) {           // get 21 samples
    sleep_mode();                       // go to sleep while taking ADC sample
    while(ADCSRA & (1<<ADSC));          // make sure sampling is completed
    result += ADC;                      // add them up
  }
  ADCSRA &= ~(1<<ADEN);                 // disable ADC to save energy
  result >>= 2;                         // divide by 4 (21/4 ~ 1100mV*(R17+R18)/R17/1023)
  return result;                        // return supply voltage in mV
}

// ADC interrupt service routine
EMPTY_INTERRUPT(ADC_vect);              // nothing to be done here

// ===================================================================================
// Watchdog Timer and Sleep Implementation
// ===================================================================================

// Reset watchdog timer
void resetWatchdog(uint8_t WDTtime) {
  cli();                                // timed sequence coming up
  wdt_reset();                          // reset watchdog
  MCUSR = 0;                            // clear various "reset" flags
  WDTCR = (1<<WDCE)|(1<<WDE)|(1<<WDTIF);// allow changes, clear interrupt
  WDTCR = WDTtime;                      // set WDT interval
  sei();                                // interrupts are required now
}

// Go to sleep in order to save energy, wake up by watchdog timer or pin change interrupt
void sleep(uint8_t WDTtime) {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // set sleep mode to power down
  GIFR |= (1<<PCIF);                    // clear any outstanding interrupts
  resetWatchdog(WDTtime);               // get watchdog ready
  sleep_mode();                         // sleep
}

// Watchdog interrupt service routine
ISR(WDT_vect) {
  wdt_disable();                        // disable watchdog
}

// Pin change interrupt service routine
EMPTY_INTERRUPT(PCINT0_vect);           // nothing to be done here

// ===================================================================================
// Main Function
// ===================================================================================

int main(void) {
  // Reset watchdog timer
  resetWatchdog(WDT8S);                 // do this first in case WDT fires

  // Local variables
  uint16_t  vcc;                        // stores the actual supply voltage mV
  uint8_t   gtimer;                     // global timer variable
  uint8_t   autopoweron = 1;            // auto power-on flag
 
  // Setup pins
  pinOutput(LED);                       // set SHUTDOWN pin as OUTPUT
  pinOutput(ENABLE);                    // set SHUTDOWN pin as OUTPUT
  pinOutput(SHUTDOWN);                  // set SHUTDOWN pin as OUTPUT
  pinHigh(REQUEST);                     // set pull-up on REQUEST pin
  
  // Setup ADC
  ADC_init();                           // init ADC

  // Disable unused peripherals to save power
  ACSR = (1<<ACD);                      // disable analog comperator
  PRR  = (1<<PRTIM0);                   // shut down timer0
  pinDisable(SENSE);                    // disable digital intput buffer on SENSE pin

  // Setup pin change interrupt
  GIMSK = (1<<PCIE);                    // pin change interrupts enable
  pinIntEn(REQUEST);                    // pin change interrupt on REQUEST pin
  sei();                                // enable global interrupts

  // Loop
  while(1) {
    // Check if power has to be turned on
    while(1) {
      pinHigh(LED);                     // heartbeat on status LED
      vcc = getVcc();                   // get supply voltage
      if(autopoweron && (vcc >= POWERONLEVEL)) break;
      if((!pinRead(REQUEST)) && (vcc >= USERPOWERLEVEL)) break;
      pinLow(LED);                      // turn off status LED again
      sleep(WDT8S);                     // sleep for a while...
    }

    // Turn on power
    pinHigh(ENABLE);                    // set enable pin of boost converter
    autopoweron = 1;                    // assume auto power on by now
    gtimer = (BOOTUPTIMER >> 3) + 1;    // set timer for bootup

    // Check if power has to be turned off
    while(1) {
      vcc = getVcc();                   // get battery voltage
      if(vcc < SHUTDOWNLEVEL) break;    // shutdown when battery low
      uint8_t counter = REQUESTTIMER;   // timer for manual shutdown
      while((!pinRead(REQUEST)) && (--counter)) _delay_ms(100);
      if(!counter) {autopoweron = 0; break;}
      if(gtimer) gtimer--;              // decrease boottimer
      sleep(WDT8S);                     // sleep for a while...
    }

    // Shut down sequence
    GIMSK = 0;                          // disable pin change interrupt
    pinHigh(SHUTDOWN);                  // tell the device to shutdown now
    gtimer = (gtimer << 3) + SHUTDOWNTIMER; // set timer for shutdown
    do {                                // start timed sequence
      pinToggle(LED);                   // toggle LED
      sleep(WDT1S);                     // sleep one second
    } while(--gtimer);
    pinLow(SHUTDOWN);                   // release SHUTDOWN pin
    GIMSK = (1<<PCIE);                  // pin change interrupts enable

    // Turn off power
    pinLow(ENABLE);                     // disable boost converter
  }
}
