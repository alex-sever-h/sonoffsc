#ifndef TASKLIGHT_H__
#define TASKLIGHT_H__

#include <Arduino.h>
#include <TaskPeriodic.h>
#include <TaskAnalog.h>
#include <math.h>

#define ADC_COUNTS              1024

#define MAX_ADC_READING  1024.0 // should be 1023, but allow for max reading
#define ADC_REF_VOLTAGE     5.0
#define REF_RESISTANCE  10000

#define LUX_CALC_SCALAR 9253401.57419056 // 2974618.09526589 // 5343960.47011858 // 71463100.1987006
#define LUX_CALC_EXPONENT -1.09873701931857 // -0.993045989676258 // -1.10174973605253 // -1.41248172220646


class TaskLight : public TaskPeriodic, public TaskAnalog {
  int pin_;
  float lightLux_;

  bool work() {
    lightLux_ = getLight();

    if (1) tlink.link.send_P(at_light, lightLux_ * 100, false);

    return true;
  }

  float getLight(void) {

    int rawADC = MYanalogRead(pin_);

    if(debug_) {
      Serial.print("Light rawADC value ");
      Serial.println(rawADC);
    }

    float ldrVoltage = ((float)rawADC / MAX_ADC_READING) * ADC_REF_VOLTAGE;
    float resistorVoltage = ADC_REF_VOLTAGE - ldrVoltage;

    if (debug_) {
      Serial.print("Light voltage ");
      Serial.print(ldrVoltage);
      Serial.print(" resistor voltage ");
      Serial.println(resistorVoltage);
    }

    float ldrResistance = ldrVoltage * REF_RESISTANCE / resistorVoltage;

    if (debug_) {
      Serial.print("Light ldrResistance ldrResistance value ");
      Serial.println(ldrResistance);
    }

    float luxLevel = LUX_CALC_SCALAR * pow(ldrResistance, LUX_CALC_EXPONENT);

    if (debug_) {
      Serial.print("Light it is LUX LUX LUX LUX LUX LUX value ");
      Serial.println(luxLevel);
    }

    return luxLevel;
}

public:
TaskLight(int pin) : TaskPeriodic(100), pin_(pin) {
    pinMode(pin_, INPUT);
    analogRead(pin_);
  }

};

#endif
