#ifndef TASKLINK_H__
#define TASKLINK_H__

#include <SerialLink.h>
#include <map>

#define SERIAL_BAUDRATE         1000000

typedef bool (*onSetHandler)(long value);
typedef bool (*onHandler)(long value);


bool effectSetHandler(long value) {
  Serial.println("cool my effect handler was called");
  return true;
}
class TaskLink;

TaskLink *gTL;

class TaskLink {
 public:
  TaskLink() : link(Serial){
    gTL = this;
    link.onGet(linkGet);
    link.onSet(linkSet);
    registerSetHandler("AT+EFFECT", effectSetHandler);
  }

  static bool linkSet(char * key, long value) {
    Serial.print("Got key ");
    Serial.print(key);
    Serial.print(" value ");
    Serial.print(value);
    Serial.print(" handler ");
    Serial.println((uintptr_t)gTL->onSetHandlers[key]);

    if(gTL->onSetHandlers[key] != nullptr) {
      return gTL->onSetHandlers[key](value);
    }

    return false;
  }

  static bool linkGet(char * key) {
    return false;
  }

  SerialLink link;

  std::map<std::string, onSetHandler> onSetHandlers;
  std::map<std::string, onHandler> onGetHandlers;
  std::map<std::string, onHandler> onSetByteStreamHandlers;

 public:
  void loop() {
    link.handle();
    //Serial.println("lloop");
  }

  bool registerSetHandler(std::string at, onSetHandler hndl) {
    onSetHandlers[at] = hndl;
    return true;
  }
};

extern TaskLink tlink; // TODO: remove

#endif
