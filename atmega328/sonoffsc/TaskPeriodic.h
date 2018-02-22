#ifndef TASKPERIODIC_H__
#define TASKPERIODIC_H__

class TaskPeriodic {
public:
    TaskPeriodic(int periodMs) : periodMs_(periodMs) {}

    unsigned long periodMs_;
    unsigned long previousMillis = 0;

    virtual bool work(void);

    bool loop(void) {
      unsigned long currentMillis = millis();
      unsigned long passedMilis = currentMillis - previousMillis;
      if(passedMilis > periodMs_) {
        previousMillis = currentMillis;
        bool wr = work();
        return wr;
      }

      return false;
    }

};

#endif
