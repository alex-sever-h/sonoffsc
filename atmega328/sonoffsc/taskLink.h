#ifndef TASKLINK_H__
#define TASKLINK_H__

#if FREERTOS
#include <Arduino_FreeRTOS.h>
#endif
#include <SerialLink.h>

#define SERIAL_BAUDRATE         1000000

class TaskLink {
public:
  TaskLink() : link(Serial){
  }

#if FREERTOS
  void begin(void) {
    xTaskCreate( taskLINK,
                 (const portCHAR *) "TaskLINK",
                 128,  // Stack size
                 this,
                 1,  // Priority
                 NULL );
  }
#endif

  SerialLink link;

private:

  void loop() {
    //link.handle();
    //Serial.println("lloop");
  }

#if FREERTOS
  static void taskLINK( void *pvParameters ) {
    TaskLink *t = (TaskLink*)pvParameters;
    for(;;) {
      t->loop();
      vTaskDelay(100);

      UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
      Serial.print(__func__);
      Serial.print(" Stack remaining: ");
      Serial.println(uxHighWaterMark);
    }
  }
#endif
};

#endif
