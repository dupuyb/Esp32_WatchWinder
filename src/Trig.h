#ifndef Trig_h
#define Trig_h

class Trig {
public:
  Trig(bool b) { 
    vtb = b; Q = false; 
  }
  void set(bool b) {
    if (b != vtb)  Q = true;
    vtb = b;
  }
  bool get() { 
    return vtb; 
  }
  bool isQ() {
    bool ret = Q;
    Q = false;
    return ret;
  } // state has triggered
private:
  bool vtb; // State
  bool Q;   // State was changed
};
#endif