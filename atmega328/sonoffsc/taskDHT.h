#ifndef TASKDHT_H__
#define TASKDHT_H__

#include <Arduino_FreeRTOS.h>
#include <idDHTLib.h>

//static const int debug_;

class TaskDHT {
public:
TaskDHT(int pin, idDHTLib::DHTType type) : pin_(pin), DHTLib(pin, type) {
      pinMode(pin_, INPUT);
  }

  void begin(void) {
    xTaskCreate( taskDHT,
                             (const portCHAR *) "TaskDHT",
                             192,  // Stack size
                             this,
                             3,  // Priority
                             NULL );
  }

  int pin_;
  TaskHandle_t pxCreatedTask_;

  float temperature_;
  int humidity_;

  unsigned long maxYield;

  idDHTLib DHTLib;

private:
  void loop() {
    DHTLib.acquireFastLoop();

    maxYield = 0;
    while (DHTLib.acquiring()){
      unsigned long before = micros();
      taskYIELD();
      unsigned long diff = micros() - before;
      //Serial.print("Yielded for ");
      if(diff > maxYield)
        maxYield = diff;
    }

    Serial.print("temp max yield: ");
    Serial.println(maxYield);

    int result = DHTLib.getStatus();
    switch (result)
    {
      case IDDHTLIB_OK:
        if(1) {
          Serial.print("Humidity (%): ");
          Serial.println(DHTLib.getHumidity(), 2);

          Serial.print("Temperature (oC): ");
          Serial.println(DHTLib.getCelsius(), 2);
        }

        temperature_ = DHTLib.getCelsius();
        humidity_ = DHTLib.getHumidity();

        break;
      default:
        if (1) {
          Serial.print("Error: ");
          Serial.println(result);
        }
        break;
    }

    if(result == IDDHTLIB_OK) {
      tlink.link.send_P(at_temp, temperature_ * 10, false);
      tlink.link.send_P(at_hum, humidity_, false);
    }
  }


  static void taskDHT( void *pvParameters ) {
    TaskDHT *t = (TaskDHT*)pvParameters;
    while(1) {
      const TickType_t xDelay = 2000 / portTICK_PERIOD_MS;

      t->loop();
#if 1
      UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
      Serial.print(__func__);
      Serial.print(" Stack remaining: ");
      Serial.println(uxHighWaterMark);
#endif
      vTaskDelay(xDelay);
    }
  }

};


#endif
