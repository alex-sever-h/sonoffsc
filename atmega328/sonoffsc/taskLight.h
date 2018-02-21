#ifndef TASKLIGHT_H__
#define TASKLIGHT_H__

#if FREERTOS
#include <Arduino_FreeRTOS.h>
#endif

#define ADC_COUNTS              1024

class TaskLight {
public:
TaskLight(int pin) : pin_(pin), periodMs_(100) {
    pinMode(pin_, INPUT);
    analogRead(pin_);
  }

#if FREERTOS
  void begin(void) {
    xTaskCreate( taskLIGHT,
                 (const portCHAR *) "TaskLIGHT",
                 164,  // Stack size
                 this,
                 1,  // Priority
                 NULL );
  }
#endif

  bool loop(void) {
    static unsigned long previousMillis = 0;
    unsigned long currentMillis = millis();
    if(currentMillis - previousMillis > periodMs_) {
      previousMillis = currentMillis;
      work();
      return true;
    }
    return false;
  }

private:
  int pin_;
  int light_;
  unsigned long periodMs_;

  void work() {
    getLight();
#if 0
    Serial.print("Light: ");
    Serial.println(light_);
#endif
    if (1) tlink.link.send_P(at_light, light_, false);

#if FREERTOS
    if (0) checkStack();
#endif
  }

  void taskLoop() {
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
    }
    // Stop interrupts and trigger
    oldADCSRA = ADCSRA;
    oldADMUX = ADMUX;
    ADCSRA &= ~_BV(ADIE);
    ADCSRA &= ~_BV(ADATE);

    if (pin >= 14)
      channel = pin - 14;

    ADMUX = (analog_reference << 6) | (channel & 0x07);
    //cli();
    // start the conversion
    ADCSRA |= _BV(ADSC);
    // ADSC is cleared when the conversion finishes
    while (bit_is_set(ADCSRA, ADSC));
    low  = ADCL;
    high = ADCH;

    raw = (high << 8) | low;

    ADCSRA = oldADCSRA | _BV(ADIF);
    ADMUX = oldADMUX;

    //sei();

    return raw;
  }

  bool conversionInProgress() {
    bool convProgress = ADCSRA & (1<<6);
    return convProgress;
  }

  void getLight() {

    int raw = MYanalogRead(pin_);

    light_ = map(raw, 0, ADC_COUNTS, 100, 0);
  }

#if FREERTOS
  void checkStack() {
      UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
      Serial.print(__func__);
      Serial.print(" Stack remaining: ");
      Serial.println(uxHighWaterMark);
  }

  static void taskLIGHT( void *pvParameters ) {
    TaskLight *t = (TaskLight*)pvParameters;
    for(;;) {
      t->taskLoop();
    }
  }
#endif

};

#endif
