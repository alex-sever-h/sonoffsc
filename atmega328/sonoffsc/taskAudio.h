#ifndef TASKAUDIO_H__
#define TASKAUDIO_H__

#if FREERTOS
#include <Arduino_FreeRTOS.h>
#endif
#include "taskLink.h"

#define BUFFERING 2
#define WAVSIZE 64

#define OTHER_BUFFER(current) ((current + 1) & 1)

extern TaskLink tlink;

class TaskAudio;

TaskAudio *gTA = NULL;

volatile int isrdiff;

uint32_t lasttime;

uint8_t gCurrent;
uint8_t gOldCurrent;

class TaskAudio {
public:
  TaskAudio(int pin) : pin_(pin){
    pinMode(pin_, INPUT_PULLUP);
    gTA = this;
  }

  void initialize(void) {
    static bool isInitialized = false;
    if (!isInitialized) {
      startSampling();
      setSampleRate(16000);
      isInitialized = true;
    }
  }

#if FREERTOS
  TaskHandle_t xTaskToNotify;
  void begin(void) {
    xTaskCreate( taskAUDIO,
                             (const portCHAR *) "TaskAUDIO",
                             192,  // Stack size
                             this,
                             3,  // Priority
                             &xTaskToNotify );
  }
#endif

  uint8_t rot;

  int32_t irqstamp;

  void adcInt() {
    buff[current&1][buffCnt[current&1]++] = ADCH; //rot;

    if(buffCnt[current&1] == WAVSIZE) {
      current ++;
      gCurrent = current;
      rot = (rot+1)&0x3;
      buffCnt[current&1] = 0;
      notifications++;
#if FREERTOS
      vTaskNotifyGiveFromISR( xTaskToNotify, NULL );
#endif
      irqstamp = micros();
    }
  }

private:

  static void reInit() {
    if(gTA) {
      gTA->configureADC();
    }
  }

  int pin_;
  int noise_;

  volatile int notifications;

  uint8_t buff[BUFFERING][WAVSIZE];
  uint8_t buffCnt[BUFFERING];
  uint8_t current;

  void configureADC(void) {
    analogRead(pin_);
    adcSetup();
  }

  void startSampling(){
    configureADC();
    tcSetup();
  }

  void setSampleRate(uint32_t sampRate){
    uint16_t tcTicks = 1000;                    /* Stores the current TC0 Ch0 counter value */
    tcTicks = max(5, frequencyToTimerCount(sampRate));
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
#if 0
    if(ctr >= 10){
      defaultShift = ctr - 10;
    }else{
      defaultShift = 0;
    }
#endif
  }

  uint32_t frequencyToTimerCount(uint32_t frequency){

    if(frequency < 5){
      TCCR1B &= ~(_BV(CS11)) | ~(_BV(CS10));
      TCCR1B |= _BV(CS12);
      return F_CPU / 256UL / frequency;
    }else
      if(frequency < 35){
        TCCR1B |= _BV(CS11) | _BV(CS10);      //Prescaler F_CPU/64
        return F_CPU / 64UL / frequency;
      }else
        if(frequency < 250){
          TCCR1B |= _BV(CS11);                  //Prescaler F_CPU/8
          TCCR1B &= ~(_BV(CS10));
          return F_CPU / 8UL / frequency;
        }else
        {
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
    gOldCurrent = current;
    oldcurrent = current;

#if FREERTOS
    ulTaskNotifyTake( pdTRUE, 100 );
#endif

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

    if(gCurrent != gOldCurrent)
    {
      Serial.println("ACTUALY OKKKKKKKKK");
    }


    return true;
  }

#if FREERTOS
  static void taskAUDIO( void *pvParameters ) {
    TaskAudio *t = (TaskAudio*)pvParameters;
    for(;;) {
      t->loop();

#if 1
      UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
      Serial.print(__func__);
      Serial.print(" Stack remaining: ");
      Serial.println(uxHighWaterMark);
#endif
    }
  }
#endif
};

ISR(ADC_vect){
  if((ADMUX & 0x1F) == 0x02){
    gTA->adcInt();
    isrdiff--;
  }
}

// Don't know why this is required
ISR(TIMER1_OVF_vect){
  isrdiff++;
}

#endif
