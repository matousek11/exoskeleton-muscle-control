#ifndef IACTUATOR_H_
#define IACTUATOR_H_

#include "IValve.h"

class IActuator {
  private:
    /**
     * Valve used for inflow of pressure.
     */
    IValve* inputValve;

    /**
     * Valve used for outflow of pressure.
     */
    IValve* outputValve;
  public:
};

#endif