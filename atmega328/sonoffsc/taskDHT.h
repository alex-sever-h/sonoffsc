#ifndef TASKDHT_H__
#define TASKDHT_H__

#if FREERTOS
#include <Arduino_FreeRTOS.h>
#endif
#include <dht.h>

static const int debug_ = 0;

class TaskDHT {
public:
TaskDHT(int pin, int type) : pin_(pin), periodMs_(2000) {
    //pinMode(pin_, INPUT);
  }

#if FREERTOS
  void begin(void) {
    xTaskCreate( taskDHT,
                             (const portCHAR *) "TaskDHT",
                             192,  // Stack size
                             this,
                             1,  // Priority
                             NULL );
  }
  TaskHandle_t pxCreatedTask_;
#endif

private:
  int pin_;

  unsigned long periodMs_;

  float temperature_;
  int humidity_;

  dht DHT;

  struct
  {
    uint32_t total;
    uint32_t ok;
    uint32_t crc_error;
    uint32_t time_out;
    uint32_t connect;
    uint32_t ack_l;
    uint32_t ack_h;
    uint32_t unknown;
    uint32_t maxReadTime;
  } stat = { 0 };

public:
  bool loop(void) {
    static unsigned long previousMillis = 0;
    unsigned long currentMillis = millis();
    if(currentMillis - previousMillis > periodMs_) {
      previousMillis = currentMillis;
      work();
      return true;
    }
    return false;
  }

private:
  bool work() {

    if (0){
      Serial.print("Audio: will be blocked from: ");
      Serial.println(micros() - lasttime);
    }

    uint32_t start = micros();
    int chk = DHT.read22(pin_);
    uint32_t stop = micros();
    uint32_t diff = stop-start;

    //    if(diff > stat.maxReadTime)
      stat.maxReadTime = diff;
    if (0) {
      Serial.print("Audio: delay will be  ");
      Serial.println(stat.maxReadTime);
    }
    //return true;

    stat.total++;
    switch (chk)
    {
    case DHTLIB_OK:
        stat.ok++;
        if (debug_) Serial.print("OK,\n");
        break;
    case DHTLIB_ERROR_CHECKSUM:
        stat.crc_error++;
        if (debug_) Serial.print("Checksum error,\n");
        break;
    case DHTLIB_ERROR_TIMEOUT:
        stat.time_out++;
        if (debug_) Serial.print("Time out error,\n");
        break;
    case DHTLIB_ERROR_CONNECT:
        stat.connect++;
        if (debug_) Serial.print("Connect error,\n");
        break;
    case DHTLIB_ERROR_ACK_L:
        stat.ack_l++;
        if (debug_) Serial.print("Ack Low error,\n");
        break;
    case DHTLIB_ERROR_ACK_H:
        stat.ack_h++;
        if (debug_) Serial.print("Ack High error,\n");
        break;
    default:
        stat.unknown++;
        if (debug_) Serial.print("Unknown error,\n");
        break;
    }

    if (debug_) {
      if (stat.total % 20 == 0) {
        Serial.println("\nTOT\tOK\tCRC\tTO\tCON\tACK_L\tACK_H\tUNK\tREAD TIME");
        Serial.print(stat.total);
        Serial.print("\t");
        Serial.print(stat.ok);
        Serial.print("\t");
        Serial.print(stat.crc_error);
        Serial.print("\t");
        Serial.print(stat.time_out);
        Serial.print("\t");
        Serial.print(stat.connect);
        Serial.print("\t");
        Serial.print(stat.ack_l);
        Serial.print("\t");
        Serial.print(stat.ack_h);
        Serial.print("\t");
        Serial.print(stat.unknown);
        Serial.print("\t");
        Serial.print(stat.maxReadTime);
        Serial.println("\n");
    }
    }

    temperature_ = DHT.temperature;
    humidity_ = DHT.humidity;

    if(chk == DHTLIB_OK) {
      tlink.link.send_P(at_temp, temperature_ * 10, false);
      tlink.link.send_P(at_hum, humidity_, false);
    }

    return true;
  }

#if FREERTOS
  static void taskDHT( void *pvParameters ) {
    TaskDHT *t = (TaskDHT*)pvParameters;
    while(1) {
      const TickType_t xDelay = 2000 / portTICK_PERIOD_MS;

      t->loop();
#if 0
      UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
      Serial.print(__func__);
      Serial.print(" Stack remaining: ");
      Serial.println(uxHighWaterMark);
#endif
      vTaskDelay(xDelay);
    }
  }
#endif
};


#endif
