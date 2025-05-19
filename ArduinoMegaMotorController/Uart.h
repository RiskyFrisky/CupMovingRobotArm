#ifndef UART_H
#define UART_H

#include <Arduino.h>

extern volatile unsigned int jointAngles[5];
extern volatile unsigned int gripperState;

void initUART();
void checkUART();
void parseUARTMessage(const char* msg);

#endif // UART_H
