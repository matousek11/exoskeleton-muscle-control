#ifndef DEBUGGER_H_
#define DEBUGGER_H_

/**
 * @brief Houses tools used for debugging
 */
class Debugger {
 public:
  /**
   * @brief Goes through I2C addresses and prints those on which are devices.
   */
  static void scanI2C();
};

#endif