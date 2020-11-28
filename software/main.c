// tinyUPS - Simple Uninterruptible Power Supply
//
// If external power is connected to the tinyUPS the input voltage or vcc of the ATtiny13 is
// delivered by this source, otherwise by the battery. The ATtiny13 monitors the input voltage
// and tells the connected device to shutdown by pulling the SHUTDOWN-line low when the input
// voltage falls below a certain threshold (SHUTDOWNLEVEL). This happens when the external
// power source is diconnected or disbled and the battery level falls below this threshold.
// After waiting a certain time (SHUTDOWNTIMER) to allow the connected device to safely shut
// down, the ATtiny13 deactivates the boost converter and turns off the power to the connected
// device.
// If the input voltage rises again above a certain threshold (POWERONLEVEL) it activates the
// boost converter and turns on the power to the connected device. This happens when the
// external power source is available again.
// When power is turned on a BOOTUPTIMER starts to count. If a shutdown is initiated before
// the boot up is completed, the left-over time is added to the SHUTDOWNTIMER in order to
// allow the connected device to completely boot up and shut down.
// A shutdown can also be initiated by pressing and holding the button or by setting the
// REQUEST-line to high (>0.7V) for 2 seconds. After such shutdowns the power will not be
// turned on again automatically. The power to the connected device can be turned on manually
// by pressing the button or setting the REQUEST-line to high if the battery level is above
// a certain threshold (USERPOWERLEVEL) or the external power source is connected.
//
// The ATtiny13 spends most of the time in power-down sleep mode to save energy. The watch dog
// timer wakes it up every 8 seconds. It will also wake up if the button was pressed or the
// REQUEST-line was changed (pin change interrupt). After doing its stuff the ATtiny13
// sleeps again.
//
// Status LED:
// - steady on:       normal power-on operation
// - blinking:        shutdown sequence
// - short flashes:   in power-off mode (short flash occurs every 8 seconds)
//
// Note:  Logic levels on SHUTDOWN and REQUEST pins are inverted to the actual lines which
//        are connected to the device. Connected device must have pullup resistor on SHUTDOWN
//        line! This is neccessary because of the different voltage levels.
//
//                        +-\/-+
// -------- A0 (D5) PB5  1|    |8  Vcc
// LED ---- A3 (D3) PB3  2|    |7  PB2 (D2) A1 --- VOLTAGE SENSE
// ENABLE - A2 (D4) PB4  3|    |6  PB1 (D1) ------ REQUEST/BUTTON
//                  GND  4|    |5  PB0 (D0) ------ SHUTDOWN
//                        +----+    
//
// Controller:  ATtiny13A
// Clockspeed:  1.2 MHz internal
//
// 2020 by Stefan Wagner 
// Project Files (EasyEDA): https://easyeda.com/wagiminator
// Project Files (Github):  https://github.com/wagiminator
// License: http://creativecommons.org/licenses/by-sa/3.0/


// libraries
#include <avr/io.h>             // for gpio
#include <avr/sleep.h>          // for the sleep modes
#include <avr/interrupt.h>      // for the interrupts
#include <avr/power.h>          // for the power control
#include <avr/wdt.h>            // for the watch dog timer
#include <util/delay.h>         // for delays

// pin definitions
#define SHUTDOWN  0             // when set high the SHUTDOWN line is pulled low
#define REQUEST   1             // gets low on button pressed or REQ-line high
#define SENSE     2             // voltage divider for measuring vcc
#define LED       3             // status LED: on when high
#define ENABLE    4             // enable pin of boost converter: on when high

// control parameters
#define SHUTDOWNLEVEL   3000    // supply voltage threshold in mV for auto shutdown
#define USERPOWERLEVEL  3500    // supply voltage when user is allowed to power on  
#define POWERONLEVEL    4300    // supply voltage threshold in mV for auto power on
#define BOOTUPTIMER     60      // time in seconds the connected device needs to boot up
#define SHUTDOWNTIMER   30      // time in seconds the connected device needs to shut down
#define REQUESTTIMER    20      // duration in 100ms request/button has to be low to shut down

// watch dog timer intervals
#define WDT1S   (1<<WDTIE)|(1<<WDP2)|(1<<WDP1)  // WDTCR value for 1 second
#define WDT8S   (1<<WDTIE)|(1<<WDP3)|(1<<WDP0)  // WDTCR value for 8 seconds


// get supply voltage in mV by reading it via voltage divider against 1.1V reference
uint16_t getVcc() {
  uint16_t result = 0;                  // result
  ADCSRA |= (1<<ADEN)|(1<<ADIF);        // enable ADC, turn off any pending interrupt
  ADMUX   = (1<<REFS0)|(1<<MUX0);       // set ADC1 against 1.1V reference
  _delay_ms(2);                         // wait for Vref to settle
  set_sleep_mode (SLEEP_MODE_ADC);      // sleep during sample for noise reduction
  for (uint8_t i=21; i; i--) {          // get 21 samples
    sleep_mode();                       // go to sleep while taking ADC sample
    while (ADCSRA & (1<<ADSC));         // make sure sampling is completed
    result += ADC;                      // add them up
  }
  ADCSRA &= ~(1<<ADEN);                 // disable ADC to save energy
  result >>= 2;                         // divide by 4 (21/4 ~ 1100mV*(R17+R18)/R17/1023)
  return result;                        // return supply voltage in mV
}

// reset watchdog timer
void resetWatchdog (uint8_t WDTtime) {
  cli();                                // timed sequence coming up
  MCUSR = 0;                            // clear various "reset" flags
  WDTCR = (1<<WDCE)|(1<<WDE)|(1<<WDTIF);// allow changes, disable reset, clear existing interrupt
  WDTCR = WDTtime;                      // set interval
  wdt_reset();                          // pat the dog
  sei();                                // interrupts are required now
}

// go to sleep in order to save energy, wake up again by watchdog timer or pin change interrupt
void sleep(uint8_t WDTtime) {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // set sleep mode to power down
  GIFR |= (1<<PCIF);                    // clear any outstanding interrupts
  resetWatchdog(WDTtime);               // get watchdog ready
  sleep_mode();                         // sleep
}

// main function
int main(void) {
  // local variables
  uint16_t  vcc;                              // stores the actual supply voltage mV
  uint8_t   gtimer;                           // global timer variable
  uint8_t   autopoweron = 1;                  // auto power-on flag

  // reset watchdog timer
  resetWatchdog (WDT8S);                      // do this first in case WDT fires
  
  // setup pins
  DDRB  = (1<<LED)|(1<<ENABLE)|(1<<SHUTDOWN); // set output pins
  PORTB = (1<<REQUEST);                       // set pull-ups

  // disable unused peripherals to save power
  ACSR   = (1<<ACD);                          // disable analog comperator
  DIDR0 |= (1<<SENSE);                        // disable digital intput buffer on SENSE pin

  // setup pin change interrupt
  GIMSK = (1<<PCIE);                          // pin change interrupts enable
  PCMSK = (1<<REQUEST);                       // set pins for pin change interrupt

  // setup ADC
  ADCSRA  = (1<<ADPS0)|(1<<ADPS1);            // set ADC clock prescaler to 8
  ADCSRA |= (1<<ADIE);                        // ADC interrupts enable
  sei();                                      // enable global interrupts

  // main loop
  while(1) {
    // check if power has to be turned on
    while(1) {
      PORTB |= (1<<LED);                      // heartbeat on status LED
      vcc = getVcc();                         // get supply voltage
      if (autopoweron && (vcc >= POWERONLEVEL)) break;
      if ((~PINB & (1<<REQUEST)) && (vcc >= USERPOWERLEVEL)) break;
      PORTB &= ~(1<<LED);                     // turn off status LED again
      sleep(WDT8S);                           // sleep for a while...
    }

    // turn on power
    PORTB |= (1<<ENABLE);                     // set enable pin of boost converter
    autopoweron = 1;                          // assume auto power on by now
    gtimer = (BOOTUPTIMER >> 3) + 1;          // set timer for bootup

    // check if power has to be turned off
    while(1) {
      vcc = getVcc();                         // get battery voltage
      if (vcc < SHUTDOWNLEVEL) break;         // shutdown when battery low
      uint8_t counter = REQUESTTIMER;         // timer for manual shutdown
      while((~PINB & (1<<REQUEST)) && (--counter)) _delay_ms(100);
      if (!counter) {autopoweron = 0; break;}
      if (gtimer) gtimer--;                   // decrease boottimer
      sleep(WDT8S);                           // sleep for a while...
    }

    // shut down sequence
    GIMSK = 0;                                // disable pin change interrupt
    PORTB |= (1<<SHUTDOWN);                   // tell the device to shutdown now
    gtimer = (gtimer << 3) + SHUTDOWNTIMER;   // set timer for shutdown
    do {                                      // start timed sequence
      PINB = (1<<LED);                        // toggle LED
      sleep(WDT1S);                           // sleep one second
    } while (--gtimer);
    PORTB &= ~(1<<SHUTDOWN);                  // release SHUTDOWN pin
    GIMSK = (1<<PCIE);                        // pin change interrupts enable

    // turn off power
    PORTB &= ~(1<<ENABLE);                    // disable boost converter
  }
}

// watchdog interrupt service routine
ISR (WDT_vect) {
  wdt_disable();                              // disable watchdog
}

// pin change interrupt service routine
EMPTY_INTERRUPT (PCINT0_vect);                // nothing to be done here

// ADC interrupt service routine
EMPTY_INTERRUPT (ADC_vect);                   // nothing to be done here
