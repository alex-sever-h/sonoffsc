#ifndef TASKDUST_H__
#define TASKDUST_H__

#include <TaskPeriodic.h>
#include <TaskAnalog.h>

#define ADC_COUNTS              1024

#define SHARP_SAMPLING_TIME  (280 - 10)
#define SHARP_DELTA_TIME     ( 40 - 10)
#define SHARP_SLEEP_TIME     (9680)

class TaskDust : public TaskPeriodic, public TaskAnalog {
  int pin_;
  int ledPin_;
  float dust_mg_m3_;

  void MYdelayMicroseconds(int us) {
    long start = micros();
    while ((long)micros() - start < us);
  }

  // 0.5V ==> 100ug/m3
  float getDust() {

    digitalWrite(ledPin_, LOW);
    MYdelayMicroseconds(SHARP_SAMPLING_TIME);

    float reading = MYanalogRead(pin_);

    MYdelayMicroseconds(SHARP_DELTA_TIME);
    digitalWrite(ledPin_, HIGH);

    // mg/m3
    float dust = 170.0 * reading * (5.0 / ADC_COUNTS) - 100.0;
    if (dust < 0) dust = 0;
    return dust;
  }

  bool work() {
    dust_mg_m3_ = getDust();
#if 0
    Serial.print("Dust: ");
    Serial.println(dust_mg_m3_);
#endif
    if (1) tlink.link.send_P(at_dust, dust_mg_m3_ * 100, false);

    return true;
  }

public:
TaskDust(int pin, int ledPin) : TaskPeriodic(100), pin_(pin), ledPin_(ledPin) {
    pinMode(pin_, INPUT);
    pinMode(ledPin_, OUTPUT);
    analogRead(pin_);
  }
};

#endif
