#ifndef TASKANALOG_H__
#define TASKANALOG_H__

class TaskAnalog {
public:
  bool conversionInProgress() {
    bool convProgress = ADCSRA & (1<<6);
    return convProgress;
  }

  uint16_t MYanalogRead(uint8_t pin) {
    uint8_t analog_reference = 0x1;
    uint16_t low, high;
    uint16_t raw;
    uint8_t channel;
    uint8_t oldADCSRA, oldADMUX;

    // wait for a conversion to start and end if auto-trigger
    if(ADCSRA & _BV(ADATE)){
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


};

#endif
