#ifndef TASKDUST_H__
#define TASKDUST_H__

#include <Arduino_FreeRTOS.h>

#define ADC_COUNTS              1024

#define SHARP_SAMPLING_TIME	    280
#define SHARP_DELTA_TIME		40
#define SHARP_SLEEP_TIME		9680

class TaskDust {
public:
TaskDust(int pin, int ledPin) : pin_(pin), ledPin_(ledPin), periodMs_(500) {
    xDelay_ = periodMs_ / portTICK_PERIOD_MS;
    pinMode(pin_, INPUT);
    pinMode(ledPin_, OUTPUT);
  }

  void begin(void) {
    xTaskCreate( taskDUST,
                 (const portCHAR *) "TaskDUST",
                 164,  // Stack size
                 this,
                 1,  // Priority
                 NULL );
  }

  void loop(void) {
    static unsigned long previousMillis = 0;
    unsigned long currentMillis = millis();
    if(currentMillis - previousMillis > periodMs_) {
      previousMillis = currentMillis;
      work();
    }
  }

  void work() {
    dust_ = getDust();
#if 1
    Serial.print("Dust: ");
    Serial.println(dust_);
#endif
    if (1) tlink.link.send_P(at_dust, dust_, false);

    if (1) checkStack();
  }

private:
  TickType_t xDelay_;
  int pin_;
  int ledPin_;
  float dust_;
  int periodMs_;

  void taskLoop() {
    vTaskDelay(xDelay_);
    work();
  }

  uint16_t MYanalogRead(uint8_t pin) {
    uint8_t analog_reference = 0x1;
    uint16_t low, high;
    uint16_t raw;
    uint8_t channel;
    uint8_t oldADCSRA, oldADMUX;

    if(ADCSRA & _BV(ADATE)){      // wait for a conversion to start and end if auto-trigger
      while(!conversionInProgress()){};
      while(conversionInProgress()){};
      // Stop interrupts and trigger
      oldADCSRA = ADCSRA;
      oldADMUX = ADMUX;
      ADCSRA &= ~_BV(ADIE);
      ADCSRA &= ~_BV(ADATE);
    }

    if (pin >= 14)
      channel = pin - 14;

    ADMUX = (analog_reference << 6) | (channel & 0x07);
    cli();
    // start the conversion
    ADCSRA |= _BV(ADSC);
    // ADSC is cleared when the conversion finishes
    while (bit_is_set(ADCSRA, ADSC));
    low  = ADCL;
    high = ADCH;

    raw = (high << 8) | low;

    ADCSRA = oldADCSRA | _BV(ADIF);
    ADMUX = oldADMUX;

    sei();

    return raw;
  }

  // 0.5V ==> 100ug/m3
  float getDust() {

    digitalWrite(ledPin_, LOW);
    delayMicroseconds(SHARP_SAMPLING_TIME);

    float reading = MYanalogRead(pin_);

    delayMicroseconds(SHARP_DELTA_TIME);
    digitalWrite(ledPin_, HIGH);

    // mg/m3
    float dust = 170.0 * reading * (5.0 / 1024.0) - 100.0;
    if (dust < 0) dust = 0;
    return dust;

  }

  bool conversionInProgress() {
    bool convProgress = ADCSRA & (1<<6);
    return convProgress;
  }

  void checkStack() {
      UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
      Serial.print(__func__);
      Serial.print(" Stack remaining: ");
      Serial.println(uxHighWaterMark);
  }

  static void taskDUST( void *pvParameters ) {
    TaskDust *t = (TaskDust*)pvParameters;
    for(;;) {
      t->taskLoop();
    }
  }

};

#endif
