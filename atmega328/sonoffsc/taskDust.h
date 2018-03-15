#ifndef TASKDUST_H__
#define TASKDUST_H__

#include <TaskPeriodic.h>
#include <TaskAnalog.h>

#define ADC_COUNTS              1024

#define SHARP_SAMPLING_TIME  (280 - 10)
#define SHARP_DELTA_TIME     ( 40 - 10)
#define SHARP_SLEEP_TIME     (9680)

// 0.5V ==> 100ug/m3
#define VOLT_DUST_RATIO (0.5 / 100)
#define VOLT_DUST_OFFSET (0.7)

class TaskDust : public TaskPeriodic, public TaskAnalog {
  SerialLink &link_;

  int pin_;
  int ledPin_;
  float dust_ug_m3_;

  void MYdelayMicroseconds(int us) {
    long start = micros();
    while ((long)micros() - start < us);
  }

  float getDust() {

    digitalWrite(ledPin_, LOW);
    MYdelayMicroseconds(SHARP_SAMPLING_TIME);

    float reading = MYanalogRead(pin_);

    MYdelayMicroseconds(SHARP_DELTA_TIME);
    digitalWrite(ledPin_, HIGH);

    float readingVolts = reading * 5.0 / ADC_COUNTS;

    float dust_ug_m3_ = (readingVolts - VOLT_DUST_OFFSET) / VOLT_DUST_RATIO;

    if (dust_ug_m3_ < 0) dust_ug_m3_ = 0;
    return dust_ug_m3_;
  }

  bool work() {
    dust_ug_m3_ = getDust();
#if 0
    Serial.print("Dust: ");
    Serial.println(dust_ug_m3_);
#endif
    if (1) link_.send_P(at_dust, dust_ug_m3_ * 100, false);

    return true;
  }

public:
TaskDust(SerialLink &link, int pin, int ledPin) : TaskPeriodic(100),
                                                  link_(link),
                                                  pin_(pin),
                                                  ledPin_(ledPin) {
    pinMode(pin_, INPUT);
    pinMode(ledPin_, OUTPUT);
    analogRead(pin_);
  }
};

#endif
