#ifndef WatchDog_h
#define WatchDog_h
class WatchDog {
public:
  WatchDog() { clear(); }
  void clear() {  delay = 0;  armed = false; }
  void set(unsigned long d) { 
    if (!armed)  delay = millis() + d; 
    armed = true; 
  }
  void reset(unsigned long d) { 
    armed = true; delay = millis() + d; 
  }
  bool isFree() { return (delay == 0); }
  bool isArmed() { return armed; }
  bool isFinished() {
    if (!armed) return false; // not armed
    if (millis() > delay) { clear(); return true; }
    return false;
  }
private:
  unsigned long delay;
  bool armed = false;
};
#endif