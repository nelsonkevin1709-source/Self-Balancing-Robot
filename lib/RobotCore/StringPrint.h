#pragma once
#include <Arduino.h>

// A Print target that collects text into a String instead of sending it
// anywhere. Lets us build one status message, then send it to both the
// USB serial monitor and the phone.
class StringPrint : public Print {
 public:
  using Print::write;
  size_t write(uint8_t c) override {
    s += (char)c;
    return 1;
  }
  size_t write(const uint8_t* buffer, size_t size) override {
    s.concat((const char*)buffer, (unsigned int)size);
    return size;
  }
  String s;
};
