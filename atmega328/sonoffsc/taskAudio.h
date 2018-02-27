#ifndef TASKAUDIO_H__
#define TASKAUDIO_H__

#include <algorithm>

#define BUFFERING 2
#define WAVSIZE 64

#define OTHER_BUFFER(current) ((current + 1) & 1)

static const int gDebug_ = 0;

class TaskAudio;

static TaskAudio *gTA = NULL;

volatile int isrdiff;

uint32_t lasttime;

static uint8_t buff[BUFFERING][WAVSIZE];
static uint8_t current;
static uint8_t *cBuff;
static uint8_t *cBuffEnd;

ISR(ADC_vect){
#if 0
  //if((ADMUX & 0x1F) == 0x02)
  {
    TaskAudio::adcInt();
    if(gDebug_) isrdiff--;
  }
#else // cut the crap
  *cBuff++ = ADCH;

  if(cBuff == cBuffEnd) {
    current = !current;
    cBuff = &buff[current][0];
    cBuffEnd = cBuff + WAVSIZE;
  }

#endif
}

class TaskAudio {
public:
  TaskAudio(int pin) : pin_(pin){
    pinMode(pin_, INPUT_PULLUP);
    gTA = this;

    current = 0;
    cBuff = &buff[current][0];
    cBuffEnd = &buff[current][WAVSIZE];
  }

  void initialize(void) {
    static bool isInitialized = false;
    if (!isInitialized) {
      startSampling();
      setSampleRate(16000);
      isInitialized = true;
    }
  }

  int32_t irqstamp;

  static void adcInt() {

    *cBuff++ = ADCH;

    if(cBuff == cBuffEnd) {
      current = !current;
      cBuff = &buff[current][0];
      cBuffEnd = &buff[current][WAVSIZE];
    }
#if 0

    buff[current&1][buffCnt[current&1]++] = ADCH;
    if(buffCnt[current&1] == WAVSIZE) {
      current++;
      buffCnt[current&1] = 0;
      notifications++;
      irqstamp = micros();
    }
#endif
  }

private:
  int pin_;
  int noise_;

  volatile int notifications;

  void startSampling(){
    analogRead(pin_);
    adcSetup();
    tcSetup();
  }

  void setSampleRate(uint32_t sampRate){
    uint16_t tcTicks = 1000;                    /* Stores the current TC0 Ch0 counter value */
    tcTicks = std::max(5UL, frequencyToTimerCount(sampRate));
    ICR1 = tcTicks;
    OCR1A = tcTicks/2;
    OCR1B = tcTicks/2;

    //Based on the frequency chosen, figure out how many bits of precision are being used for the timer PWM
    //Allows to shift 8-bit samples when the timer is running at >=10-bit pwm
    uint16_t ticks = tcTicks;
    uint8_t ctr = 0;

    while(ticks){
      ticks >>= 1;
      ctr++;
    }
  }

  uint32_t frequencyToTimerCount(uint32_t frequency){
    if(frequency < 5) {
      TCCR1B &= ~(_BV(CS11)) | ~(_BV(CS10));
      TCCR1B |= _BV(CS12);
      return F_CPU / 256UL / frequency;
    } else if(frequency < 35) {
      TCCR1B |= _BV(CS11) | _BV(CS10);      //Prescaler F_CPU/64
      return F_CPU / 64UL / frequency;
    } else if(frequency < 250) {
      TCCR1B |= _BV(CS11);                  //Prescaler F_CPU/8
      TCCR1B &= ~(_BV(CS10));
      return F_CPU / 8UL / frequency;
    } else {
      TCCR1B &= ~(_BV(CS11));               //Prescaler F_CPU
      TCCR1B |= _BV(CS10);
      return F_CPU / frequency;
    }
  }

  void adcSetup(void){
    ADCSRA = _BV(ADEN) | _BV(ADATE) | _BV(ADPS2) ; // En ADC, En Auto Trigger, Prescaler 8 (16Mhz/13/8 = ~150Khz)
    ADCSRB = _BV(ADTS2) | _BV(ADTS1);
    ADMUX |= _BV(REFS0) | _BV(ADLAR);   // 5V reference and capacitor |  Left adjust result for 8-bit

    TIMSK1 |= _BV(TOIE1);
    ADCSRA |= _BV(ADIE);
  }

  void tcSetup (){
    TCCR1A = _BV(WGM11) | _BV(COM1A1) | _BV(COM1B0) | _BV(COM1B1); // Set WGM mode, opposite action for output mode
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10); // Set WGM mode & no prescaling

    ICR1 = 1000;    // Default 16Khz with 16Mhz CPU
    OCR1A = 250;    // Default 25%/75% duty cycle
  }

public:
  bool loop (void) {
    static uint8_t oldcurrent;
    static int oldnotifications;
    initialize();

    if(current == oldcurrent)
      return false;
    oldcurrent = current;

    uint32_t us = micros();
    uint32_t delay = us - lasttime;
    lasttime = us;

    notifications--;

    oldnotifications = notifications;

    if( oldnotifications != notifications) {
      Serial.print("Audio: ");
      Serial.print(current);
      Serial.print(" ");
      Serial.print(isrdiff);
      Serial.print(" ");
      Serial.print(notifications);
      Serial.print(" ");
      Serial.print((int32_t)us - irqstamp);
      Serial.print(" ");
      Serial.println(delay);
    }
#if 1
    tlink.link.sendByteStream("AT+WAV",
                              (const char *)buff[OTHER_BUFFER(current)],
                              WAVSIZE,
                              false);
#endif

    if( 0 ) {
      Serial.print("Audio: time ");
      Serial.println(micros() - us);
    }

    return true;
  }
};


// Don't know why this is required
ISR(TIMER1_OVF_vect){
  if(gDebug_) isrdiff++;
}

#endif
