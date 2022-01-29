#include <Arduino.h>
#include <avr/sleep.h>
#include <RCSwitch.h>


const uint32_t deviceNumber = 0x50000000;
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
bool checkDoorState = false;
bool doorClosed = false;
const int nbLoops = 800; // nbloops * 9 seconds = wait time before next check. (800 = 2h)
int loopCounter = 0;

RCSwitch mySwitch = RCSwitch();


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


long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1126400L / result; // Calculate Vcc (in mV); 1126400 = 1.1*1024*1000
  return result;
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

void setupWDT()
{
  // setup wdt for 8 secs to proove door state
  byte bb = 9 & 7; // 8=4sec 9=8 sec
  bb |= (1 << 5);  // Set the special 5th bit if necessary
  // This order of commands is important and cannot be combined
  MCUSR &= ~(1 << WDRF);             // Clear the watch dog reset
  WDTCR |= (1 << WDCE) | (1 << WDE); // Set WD_change enable, set WD enable
  WDTCR = bb;                        // Set new watchdog timeout value
  WDTCR |= _BV(WDIE);                // Set the interrupt enable, this will keep unit from resetting after each int

  // 1. In the same operation, write a logic one to WDCE and WDE. A logic one must be written to WDE even though it is set to one before the disable operation starts.
  // 2. Within the next four clock cycles, write a logic 0 to WDE. This disables the Watchdog.
}



void setup()
{

  pinMode(PIN_NO, OUTPUT);       digitalWrite(PIN_NO, LOW);
  pinMode(PIN_NC, OUTPUT);       digitalWrite(PIN_NC, LOW);
  pinMode(PIN_RF_VCC, OUTPUT);   digitalWrite(PIN_RF_VCC, LOW);
  pinMode(PIN_RF_DATA, OUTPUT);  digitalWrite(PIN_RF_DATA, LOW);
  pinMode(PB0, OUTPUT);          digitalWrite(PB0, LOW);

  doorClosed = getDoorState();

/*
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
*/
  mySwitch.enableTransmit(PIN_RF_DATA); //simply sets pin number to output

//  blinkLED(1, 200);
  GIMSK |= _BV(PCIE); // enable pin  change interrupt
  setupWDT();

}


void stopWDT()
{
  __asm__ __volatile__("wdr"
                       "\n\t" ::);
  //_WDR();
  MCUSR = 0x00;
  WDTCR |= (1 << WDCE) | (1 << WDE);
  WDTCR = 0x00;

  //  WDTCR |= (1 << WDCE) | (1 << WDE); // Set WD_change enable, set WD enable
  //  WDTCR &= ~(_BV(WDE));
}


void sendDoorState(bool doorState)
{
      uint32_t door = doorState + deviceNumber + valDoor;
      digitalWrite(PIN_RF_VCC, HIGH);
      mySwitch.send(door, 32);
      digitalWrite(PIN_RF_VCC, LOW);
      digitalWrite(PIN_RF_DATA, LOW);
}

void sendVcc()
{
      uint32_t battery = readVcc();
      battery = battery + deviceNumber + valVolts;
      digitalWrite(PIN_RF_VCC, HIGH);
      mySwitch.send(battery, 32);
      digitalWrite(PIN_RF_VCC, LOW);
      digitalWrite(PIN_RF_DATA, LOW);


}

void loop()
{


  if (interruptFiredPC == true)
  {
    interruptFiredPC = false;
    checkDoorState = true; // recheck doorstate in next loop
//    setupWDT();

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
//  blinkLED(1, 20);

  if (interruptFiredWDT == true)
  {
    interruptFiredWDT = false;
    if (checkDoorState == true)
    { //  we have to check door if state  has changed since previous loop.
      checkDoorState = false;
      if (doorClosed == getDoorState())
      {
        sendDoorState(doorClosed);
      }
      else // check door state again in next loop
      {
        interruptFiredPC = true;
//                loopCounter = 0;
      }
    }
    else
    {
      loopCounter++;
      if (loopCounter > nbLoops)
      {
        loopCounter = 0;
        //did door state change within last period of wdt seconds * nbcount?
        if (doorClosed != getDoorState()) checkDoorState = true; // door state to be checked in next iteration
        sendVcc();//send heartbeat aftr each period
      }
//      else
  //      doSleep();
      // count and if 1 hr delay is reached, recheck door state

    }
  }
}

ISR(WDT_vect)
{
  interruptFiredWDT = true;
  //  interruptFiredPC = true;
}
