#ifndef TASKLINK_H__
#define TASKLINK_H__

#include <Arduino_FreeRTOS.h>
#include <SerialLink.h>

class TaskLink {
public:
TaskLink() : link(Serial){
  }
  void begin(void) {
    BaseType_t xReturned;
    xReturned = xTaskCreate( taskLINK,
                             (const portCHAR *) "TaskLINK",
                             96,  // Stack size
                             this,
                             1,  // Priority
                             NULL );
    if (xReturned != pdPASS) {
      Serial.println("task create failed");
    }
  }
private:

  SerialLink link;

  void loop() {
    //link.handle();
    //Serial.println("lloop");
  }

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

};

#endif
