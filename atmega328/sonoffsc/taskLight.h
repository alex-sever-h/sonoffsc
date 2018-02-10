#ifndef TASKLIGHT_H__
#define TASKLIGHT_H__

#include <Arduino_FreeRTOS.h>

#define ADC_COUNTS              1024

class TaskLight {
public:
  TaskLight(int pin) : pin_(pin){
    xDelay_ = 500 / portTICK_PERIOD_MS;
    pinMode(pin_, INPUT);
  }

  void begin(void) {
    BaseType_t xReturned;
    xReturned = xTaskCreate( taskLIGHT,
                             (const portCHAR *) "TaskLIGHT",
                             96,  // Stack size
                             this,
                             1,  // Priority
                             NULL );
    if (xReturned != pdPASS) {
      Serial.println("task create failed");
    }
  }
private:
  TickType_t xDelay_;
  int pin_;
  int light_;

  bool conversionInProgress() {
    bool convProgress = ADCSRA & (1<<6);
    return convProgress;
  }

  void getLight() {
    // wait for a conversion to start and end !!!
    while(!conversionInProgress()){};
    while(conversionInProgress()){};

    light_ = map(analogRead(pin_), 0, ADC_COUNTS, 100, 0);

    TaskAudio::reInit();
  }

  void loop() {
    getLight();
    Serial.print("Light: ");
    Serial.println(light_);

    //    if (push) link.send_P(at_light, light, false);

    vTaskDelay(xDelay_);
  }

  static void taskLIGHT( void *pvParameters ) {
    TaskLight *t = (TaskLight*)pvParameters;
    for(;;) {
      t->loop();
      UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
      Serial.print(__func__);
      Serial.print(" Stack remaining: ");
      Serial.println(uxHighWaterMark);
    }
  }

};

#endif
