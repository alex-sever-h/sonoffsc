#ifndef TASKAUDIO_H__
#define TASKAUDIO_H__

#include <Arduino_FreeRTOS.h>

#define BUFFERING 2
#define WAVSIZE 64

#define OTHER_BUFFER(current) ((current + 1) & 1)

class TaskAudio;

TaskAudio *gTA = NULL;

class TaskAudio {
public:
  TaskAudio(int pin) : pin_(pin){
    pinMode(pin_, INPUT_PULLUP);

    gTA = this;
  }

  void begin(void) {
    xTaskCreate( taskAUDIO,
                             (const portCHAR *) "TaskAUDIO",
                             100,  // Stack size
                             this,
                             3,  // Priority
                             &xTaskToNotify );

    startSampling();
    setSampleRate(16000);
  }

  void adcInt() {
    buff[current&1][buffCnt[current&1]++] = ADCH;

    if(buffCnt[current&1] == WAVSIZE) {
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      current ++;
      buffCnt[current&1] = 0;
      notifications++;
      vTaskNotifyGiveFromISR( xTaskToNotify, &xHigherPriorityTaskWoken );
    }
  }

  static void reInit() {
    if(gTA) {
      gTA->configureADC();
    }
  }

private:
  int pin_;
  int noise_;

  volatile int notifications;

  uint8_t buff[BUFFERING][WAVSIZE];
  uint8_t buffCnt[BUFFERING];
  uint8_t current;

  TaskHandle_t xTaskToNotify;

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

  void loop() {
    ulTaskNotifyTake( pdTRUE, 100 );
    notifications--;
#if 1
    Serial.print("Audio: ");
    Serial.write(buff[OTHER_BUFFER(current)], WAVSIZE);
    //Serial.print(current);
    Serial.print(" ");
    Serial.println(notifications);
#endif
#if 0
    if (aaAudio.getADC(buff, WAVSIZE) )
    {
      //link.sendByteStream("AT+WAV", &buff[0], WAVSIZE, false);
      Serial.write(buff, WAVSIZE);
    }
#endif
  }

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

};

ISR(ADC_vect){
  gTA->adcInt();
}

// Don't know why this is required
ISR(TIMER1_OVF_vect){
}

#endif
