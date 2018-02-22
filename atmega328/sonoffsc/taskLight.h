#ifndef TASKLIGHT_H__
#define TASKLIGHT_H__

#include <TaskPeriodic.h>
#include <TaskAnalog.h>

#define ADC_COUNTS              1024

class TaskLight : public TaskPeriodic, public TaskAnalog {
  int pin_;
  int light_;

  bool work() {
    getLight();
#if 0
    Serial.print("Light: ");
    Serial.println(light_);
#endif
    if (1) tlink.link.send_P(at_light, light_, false);

    return true;
  }

  void getLight() {

    int raw = MYanalogRead(pin_);

    light_ = map(raw, 0, ADC_COUNTS, 100, 0);
  }

public:
TaskLight(int pin) : TaskPeriodic(100), pin_(pin) {
    pinMode(pin_, INPUT);
    analogRead(pin_);
  }

};

#endif
