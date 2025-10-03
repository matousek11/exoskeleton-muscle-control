#ifndef ARDUINOMONITORSERVICE_H
#define ARDUINOMONITORSERVICE_H

#include "Arduino.h"
#include "./../models/Muscle.h"

class ArduinoMonitorService {
    private:
        void clearSerialMonitor();

    public:
        void controlThroughMonitor(Muscle* muscle);
        void printPossibleCommands(String* inputCommand = nullptr, bool unknownCommand = false);
};

#endif