#include <Arduino.h>
#include <avr/sleep.h>
//#include <avr/wdt.h>

//#include <SystemStatus.h>
//#include "ATtinySerialOut.hpp"

/*uint8_t width = 128;
  uint8_t height = 32;
  oled.begin(width, height, sizeof(tiny4koled_init_128x32br), tiny4koled_init_128x32br);

*/

const uint32_t deviceNumber = 0x50000000;
const uint32_t valTemp = 0x01000000;
const uint32_t valHum = 0x02000000;
const uint32_t valPres = 0x03000000;
const uint32_t valVolts = 0x04000000;
const uint32_t valDoor = 0x09000000;

//#define TX_PIN PB2  // for Serial.print

// SystemStatus sst;

#define PIN_NO PB3 // fil orange
#define PIN_NC PB2 // fil vert
#define PIN_RF_VCC PB1
#define PIN_RF_DATA PB4
#define PIN_STATE_1 PB1 // vert NO
#define PIN_STATE_2 PB4 // orange NC
#define PIN_STATE_0 PB0 // fil jaune

volatile bool interruptFiredPC = true;
volatile bool interruptFiredWDT = false;
bool doorClosed = false;

void blinkLEDOnce(int nb, int duree)
{
  for (int i = 0; i < nb; i++)
  {
    digitalWrite(PIN_STATE_2, HIGH);
    delay(duree);
    digitalWrite(PIN_STATE_2, LOW);
    delay(duree);
  }
}

void blinkLED(int nb, int duree = 500)
{
  blinkLEDOnce(nb, duree);
  delay(duree);
}

bool getDoorState()
// false = door open
// true = door closed
{
  pinMode(PIN_NO, INPUT_PULLUP); // interrupt pin to gnd to restart tiny
  pinMode(PIN_NC, INPUT_PULLUP); // interrupt pin to gnd to restart tiny

  uint32_t pinClosed = digitalRead(PIN_NC); // door open when LOW
  uint32_t pinOpened = digitalRead(PIN_NO); // door closed when LOW

  if ((pinClosed == LOW) && (pinOpened == HIGH)) // porte ouverte
  {
    pinMode(PIN_NC, OUTPUT);
    digitalWrite(PIN_NC, LOW);
    pinMode(PIN_NO, INPUT_PULLUP); // interrupt pin to gnd to restart tiny
    PCMSK |= _BV(PIN_NO);          // PC NO int to wake up
                                   //    PCMSK |= _BV(PIN_NC);          // PC NO int to wake up
    PCMSK &= ~(_BV(PIN_NC));       // PC NC no considered
    return false;
  }

  if ((pinClosed == HIGH) && (pinOpened == LOW)) // porte fermee
  {
    pinMode(PIN_NO, OUTPUT);
    digitalWrite(PIN_NO, LOW);
    pinMode(PIN_NC, INPUT_PULLUP); // interrupt pin to gnd to restart tiny
    PCMSK |= _BV(PIN_NC);          // PC NC int to wake up
                                   //    PCMSK |= _BV(PIN_NO);          // PC NC int to wake up
    PCMSK &= ~(_BV(PIN_NO));       // PC NO no considered
    return true;
  }
}

void doSleep()
{
  //  GIMSK |= _BV(PCIE); // enable pin  change interrupt
  MCUCR |= _BV(SM1);    // sleep mode power down
  MCUCR &= ~(_BV(SM0)); // sleep mode power down

  ADCSRA &= ~_BV(ADEN); // ADC stop
                        //    PCMSK |= _BV(PIN_NC);  // PC int to wake up
                        //    PCMSK |= _BV(PIN_NO);  // PC int to wake up

  MCUCR |= _BV(SE); // SE: Sleep Enable
  __asm__ __volatile__("sleep"
                       "\n\t" ::);
  MCUCR &= ~(_BV(SE)); // SE: Sleep DISable
  ADCSRA |= _BV(ADEN); // ADC start
}

ISR(PCINT0_vect)
{
  //  cli(); // noeffect as I-bit is set automatically when exiting ISR with RETI

  //  char cSREG = SREG;       // save SREG
  //  GIMSK &= ~(_BV(PCIE));   // stop pin  change interrupt
  PCMSK &= ~(_BV(PIN_NC)); // disable pin change int
  PCMSK &= ~(_BV(PIN_NO)); // disable pin change int
                           //  GIFR |= _BV(PCIF);       // delete current int??
                           //  SREG = cSREG;
  GIFR &= ~(_BV(PCIF));
  interruptFiredPC = true;
}

void setup()
{

  delay(1000);
  doorClosed = getDoorState();
  pinMode(PIN_STATE_0, OUTPUT);
  pinMode(PIN_STATE_1, OUTPUT);
  pinMode(PIN_STATE_2, OUTPUT);
  digitalWrite(PIN_STATE_0, LOW);
  digitalWrite(PIN_STATE_1, LOW);
  digitalWrite(PIN_STATE_2, LOW);
  if (doorClosed == true)
  {
    digitalWrite(PIN_STATE_0, HIGH);
  }
  else
  {
    digitalWrite(PIN_STATE_1, HIGH);
  }

  blinkLED(1, 200);
  GIMSK |= _BV(PCIE); // enable pin  change interrupt
}

void setupWDT()
{
  // setup wdt for 8 secs to proove door state
  byte bb = 8 & 7;
  bb |= (1 << 5); // Set the special 5th bit if necessary
  // This order of commands is important and cannot be combined
  MCUSR &= ~(1 << WDRF);             // Clear the watch dog reset
  WDTCR |= (1 << WDCE) | (1 << WDE); // Set WD_change enable, set WD enable
  WDTCR = bb;                        // Set new watchdog timeout value
  WDTCR |= _BV(WDIE);                // Set the interrupt enable, this will keep unit from resetting after each int

  // 1. In the same operation, write a logic one to WDCE and WDE. A logic one must be written to WDE even though it is set to one before the disable operation starts.
  // 2. Within the next four clock cycles, write a logic 0 to WDE. This disables the Watchdog.
}

void stopWDT()
{
  __asm__ __volatile__("wdr"
                       "\n\t" ::);
//_WDR();
MCUSR = 0x00;
WDTCR |= (1<<WDCE) | (1<<WDE);
WDTCR = 0x00;

//  WDTCR |= (1 << WDCE) | (1 << WDE); // Set WD_change enable, set WD enable
//  WDTCR &= ~(_BV(WDE));
}

void loop()
{

  if (interruptFiredPC == true)
  {
    delay(100);
    interruptFiredPC = false;
    setupWDT();
    doorClosed = getDoorState();
    pinMode(PIN_STATE_0, OUTPUT);
    pinMode(PIN_STATE_1, OUTPUT);
    pinMode(PIN_STATE_2, OUTPUT);
    digitalWrite(PIN_STATE_0, LOW);
    digitalWrite(PIN_STATE_1, LOW);
    digitalWrite(PIN_STATE_2, LOW);
    if (doorClosed == true)
    {
      digitalWrite(PIN_STATE_0, HIGH);
      //      pinMode(PIN_NC, INPUT_PULLUP);
      //      pinMode(PIN_NO, OUTPUT);
      //      digitalWrite(PIN_NO, LOW);
      // do
      // {
      // } while (digitalRead(PIN_NC) == HIGH);
    }
    else
    {
      digitalWrite(PIN_STATE_1, HIGH);
      // pinMode(PIN_NO, INPUT_PULLUP);
      // pinMode(PIN_NC, OUTPUT);
      // digitalWrite(PIN_NC, LOW);
      // do
      // {
      // } while (digitalRead(PIN_NO) == HIGH);
    }
  }

  doSleep();
  GIFR &= ~(_BV(PCIF));
  blinkLED(1, 20);

  if (interruptFiredWDT == true)
  {
    interruptFiredWDT = false;
    if (doorClosed == getDoorState())
    {
      stopWDT();
    }
    else
    {
      interruptFiredPC = true;
    }
  }
}

ISR(WDT_vect)
{
  interruptFiredWDT = true;
  //  interruptFiredPC = true;
}
