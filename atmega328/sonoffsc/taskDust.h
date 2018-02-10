#ifndef TASKDUST_H__
#define TASKDUST_H__

#include <Arduino_FreeRTOS.h>

#define ADC_COUNTS              1024

#define SHARP_SAMPLING_TIME	    280
#define SHARP_DELTA_TIME		40
#define SHARP_SLEEP_TIME		9680

class TaskDust {
public:
TaskDust(int pin, int ledPin) : pin_(pin), ledPin_(ledPin) {
    xDelay_ = 500 / portTICK_PERIOD_MS;
    pinMode(pin_, INPUT);
    pinMode(ledPin_, OUTPUT);
  }

  void begin(void) {
    BaseType_t xReturned;
    xReturned = xTaskCreate( taskDUST,
                             (const portCHAR *) "TaskDUST",
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
  int ledPin_;
  float dust_;

  // 0.5V ==> 100ug/m3
  float getDust() {

    digitalWrite(ledPin_, LOW);
    delayMicroseconds(SHARP_SAMPLING_TIME);

    float reading = analogRead(pin_);

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

  void loop() {

    // wait for a conversion to start and end !!!
    while(!conversionInProgress()){};
    while(conversionInProgress()){};

    dust_ = getDust();

    TaskAudio::reInit();

    Serial.print("Dust: ");
    Serial.println(dust_);

    //    if (push) link.send_P(at_dust, dust, false);

    vTaskDelay(xDelay_);
  }

  static void taskDUST( void *pvParameters ) {
    TaskDust *t = (TaskDust*)pvParameters;
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
