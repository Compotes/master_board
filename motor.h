#ifndef MOTOR_H_
#define MOTOR_H_

#include "hal.h"
#include "ch.h"

#define BITS_IN_CHAR 4
#define CHARS_IN_VALUE 4
#define COMMAND_CHAR_LENGTH 4
#define INIT_MESSAGE_LENGTH 14
#define MAX_MOTORS_QUANTITY 6
#define NOT_USED_MOTORS 1
#define SPEED_VALUE_CHAR_LENGTH 3

#define MOTOR_SERIAL &SD2

#define MESSAGE_CHAR_LENGTH 28

void change_motors_speeds(int16_t ms1, int16_t ms2, int16_t ms3, int16_t ms4, int16_t ms5, uint8_t sender_thread_id);
void motors_init(void);

#endif
