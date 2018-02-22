#ifndef TASKLINK_H__
#define TASKLINK_H__

#include <SerialLink.h>

#define SERIAL_BAUDRATE         1000000

class TaskLink {
public:
  TaskLink() : link(Serial){
  }

  SerialLink link;

private:

  void loop() {
    //link.handle();
    //Serial.println("lloop");
  }
};

extern TaskLink tlink; // TODO: remove

#endif
