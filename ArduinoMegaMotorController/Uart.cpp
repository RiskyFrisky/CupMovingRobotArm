#include "Uart.h"

// UART receive buffer and state
static char uartBuffer[64];
static uint8_t uartIdx = 0;

// Angles and gripper state (global for access in main code)
volatile unsigned int jointAngles[5] = {0};
volatile unsigned int gripperState = 0;

void initUART() {
  Serial1.begin(115200);
}

void parseUARTMessage(const char* msg) {
  unsigned int angles[5];
  unsigned int gripper;
  int parsed = sscanf(msg, "%u,%u,%u,%u,%u,%u",
                      &angles[0], &angles[1], &angles[2], &angles[3], &angles[4], &gripper);
  if (parsed == 6) {
    for (int i = 0; i < 5; ++i) jointAngles[i] = angles[i];
    gripperState = gripper;
    // Optionally: call a function to update motors here
    // updateMotors(jointAngles, gripperState);
  }
}

void checkUART() {
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n' || c == '\r') {
      uartBuffer[uartIdx] = '\0';
      parseUARTMessage(uartBuffer);
      uartIdx = 0;
    } else if (uartIdx < sizeof(uartBuffer) - 1) {
      uartBuffer[uartIdx++] = c;
    }
  }
}