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
// Controller:  ATtiny13
// Core:        MicroCore (https://github.com/MCUdude/MicroCore)
// Clockspeed:  1.2 MHz internal
// BOD:         disabled (to save energy)
// Timing:      Micros disabled
//
// 2020 by Stefan Wagner


// Libraries
#include <avr/sleep.h>          // for the sleep modes
#include <avr/interrupt.h>      // for the interrupts
#include <avr/power.h>          // for the power control
#include <avr/wdt.h>            // for the watch dog timer

// Pin definitions
#define SHUTDOWN  0             // when set high the SHUTDOWN line is pulled low
#define REQUEST   1             // gets low on button pressed or REQ-line high
#define SENSE     2             // voltage divider for measuring vcc
#define LED       3             // status LED: on when high
#define ENABLE    4             // enable pin of boost converter: on when high

// Control parameters
#define SHUTDOWNLEVEL   3000    // supply voltage threshold in mV for auto shutdown
#define USERPOWERLEVEL  3500    // supply voltage when user is allowed to power on  
#define POWERONLEVEL    4300    // supply voltage threshold in mV for auto power on
#define BOOTUPTIMER     60      // time in seconds the connected device needs to boot up
#define SHUTDOWNTIMER   30      // time in seconds the connected device needs to shut down
#define REQUESTTIMER    20      // duration in 100ms request/button has to be low to shut down

// Watch dog timer intervals
#define WDT1S   bit(WDTIE)|bit(WDP2)|bit(WDP1)  // WDTCR value for 1 second
#define WDT8S   bit(WDTIE)|bit(WDP3)|bit(WDP0)  // WDTCR value for 8 seconds

// Global variables
uint16_t  vcc;                  // stores the actual supply voltage mV
uint8_t   gtimer;               // global timer variable
bool      autopoweron = true;   // auto power-on flag


void setup() {
  // reset watchdog timer
  resetWatchdog (WDT8S);                      // do this first in case WDT fires
  
  // setup pins
  DDRB  = bit (LED) | bit (ENABLE) | bit (SHUTDOWN);  // set output pins
  PORTB = bit (REQUEST);                              // set pull-ups

  // setup pin change interrupt
  GIMSK = bit (PCIE);                         // pin change interrupts enable
  PCMSK = bit (REQUEST);                      // set pins for pin change interrupt

  // setup ADC
  ADCSRA  = bit (ADPS0) | bit (ADPS1);        // set ADC clock prescaler to 8
  ADCSRA |= bit (ADIE);                       // ADC interrupts enable
  interrupts ();                              // enable global interrupts
}

void loop() {
  // check if power has to be turned on
  while(true) {
    bitSet(PORTB, LED);                       // heartbeat on status LED
    vcc = getVcc();                           // get supply voltage
    if (autopoweron && (vcc >= POWERONLEVEL)) break;
    if ( (!bitRead(PINB, REQUEST)) && (vcc >= USERPOWERLEVEL)) break;
    bitClear(PORTB, LED);                     // turn off status LED again
    sleep(WDT8S);                             // sleep for a while...
  }

  // turn on power
  bitSet(PORTB, ENABLE);                      // set enable pin of boost converter
  autopoweron = true;                         // assume auto power on by now
  gtimer = (BOOTUPTIMER >> 3) + 1;            // set timer for bootup

  // check if power has to be turned off
  while(true) {
    vcc = getVcc();                           // get battery voltage
    if (vcc < SHUTDOWNLEVEL) break;           // shutdown when battery low
    uint8_t counter = REQUESTTIMER;           // timer for manual shutdown
    while( (!bitRead(PINB, REQUEST))  && (--counter)) delay(100);
    if (!counter) {autopoweron = false; break;}
    if (gtimer) gtimer--;                     // decrease boottimer
    sleep(WDT8S);                             // sleep for a while...
  }

  // shut down sequence
  GIMSK = 0;                                  // disable pin change interrupt
  bitSet(PORTB, SHUTDOWN);                    // tell the device to shutdown now
  gtimer = (gtimer << 3) + SHUTDOWNTIMER;     // set timer for shutdown
  while (gtimer--) {                          // start timed sequence
    bitSet(PINB, LED);                        // toggle LED
    sleep(WDT1S);                             // sleep one second
  }
  bitClear(PORTB, SHUTDOWN);                  // release SHUTDOWN pin
  GIMSK = bit (PCIE);                         // pin change interrupts enable

  // turn off power
  bitClear(PORTB, ENABLE);                    // disable boost converter
}

// get supply voltage in mV by reading it via voltage divider against 1.1V reference
uint16_t getVcc() {
  uint16_t result = 0;                  // result
  ADCSRA |= bit (ADEN)  | bit (ADIF);   // enable ADC, turn off any pending interrupt
  ADMUX   = bit (REFS0) | bit (MUX0);   // set ADC1 against 1.1V reference
  delay(2);                             // wait for Vref to settle
  set_sleep_mode (SLEEP_MODE_ADC);      // sleep during sample for noise reduction
  for (uint8_t i=0; i<21; i++) {        // get 21 samples
    sleep_mode();                       // go to sleep while taking ADC sample
    while (bitRead(ADCSRA, ADSC));      // make sure sampling is completed
    result += ADC;                      // add them up
  }
  bitClear(ADCSRA, ADEN);               // disable ADC to save energy
  result >>= 2;                         // divide by 4 (21/4 ~ 1100mV*(R17+R18)/R17/1023)
  return result;                        // return supply voltage in mV
}

// go to sleep in order to save energy, wake up again by watchdog timer or pin change interrupt
void sleep(uint8_t WDTtime) {
  set_sleep_mode (SLEEP_MODE_PWR_DOWN); // set sleep mode to power down
  bitSet (GIFR, PCIF);                  // clear any outstanding interrupts
  power_all_disable ();                 // power off ADC and timer
  noInterrupts ();                      // timed sequence coming up
  resetWatchdog (WDTtime);              // get watchdog ready
  sleep_enable ();                      // ready to sleep
  interrupts ();                        // interrupts are required now
  sleep_cpu ();                         // sleep              
  sleep_disable ();                     // precaution
  power_all_enable ();                  // power everything back on
}

// reset watchdog timer
void resetWatchdog (uint8_t WDTtime) {
  MCUSR = 0;                            // clear various "reset" flags
  WDTCR = bit (WDCE)  | bit (WDE)  | bit (WDTIF); // allow changes, disable reset, clear existing interrupt
  WDTCR = WDTtime;                      // set interval
  wdt_reset();                          // pat the dog
}

// watchdog interrupt service routine
ISR (WDT_vect) {
  wdt_disable();                        // disable watchdog
}

// pin change interrupt service routine
EMPTY_INTERRUPT (PCINT0_vect);          // nothing to be done here

// ADC interrupt service routine
EMPTY_INTERRUPT (ADC_vect);             // nothing to be done here
