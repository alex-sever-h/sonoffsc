#ifndef TASKDHT_H__
#define TASKDHT_H__

#include <TaskPeriodic.h>
#include <dht.h>

static const int debug_ = 0;

class TaskDHT : public TaskPeriodic {
private:
  int pin_;
  dht DHT;

  float temperature_;
  float humidity_;

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

  void updateStats(int chk, uint32_t diff) {
    if(diff > stat.maxReadTime)
      stat.maxReadTime = diff;

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
  }

  void printStats() {
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
  }

  bool work(void) {
    uint32_t start = micros();
    int chk = DHT.read22(pin_);
    uint32_t stop = micros();
    uint32_t diff = stop-start;

    updateStats(chk, diff);
    printStats();

    if(chk == DHTLIB_OK) {
      temperature_ = DHT.temperature;
      humidity_ = DHT.humidity;

      tlink.link.send_P(at_temp, temperature_ * 10, false);
      tlink.link.send_P(at_hum, humidity_ * 10, false);
    }

    return true;
  }

public:
TaskDHT(int pin) : TaskPeriodic(1000), pin_(pin) {
    pinMode(pin_, INPUT);
  }

};


#endif
