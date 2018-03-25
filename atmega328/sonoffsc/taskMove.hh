#ifndef TASKMOVE_H__
#define TASKMOVE_H__

#include <TaskPeriodic.h>

class TaskMove : public TaskPeriodic {
  SerialLink &link_;

  int pin_;
  bool move_;

  float getMove() {
    bool move_ = digitalRead(pin_) == HIGH;
    return move_;
  }

  bool work() {
    move_ = getMove();

    if (1) link_.send_P(at_move, move_, false);

    return true;
  }

public:
TaskMove(SerialLink &link, int pin) : TaskPeriodic(100),
                                      link_(link),
                                      pin_(pin) {
    pinMode(pin_, INPUT);
  }
};

#endif
