#include "hal.h"
#include "ch.h"
#include "motor.h"

uint8_t motor_speed_buff[CHARS_IN_VALUE];

thread_t *motor_thread;

int16_t writing_motors_speeds[MAX_MOTORS_QUANTITY - NOT_USED_MOTORS];

uint8_t message[COMMAND_CHAR_LENGTH + MAX_MOTORS_QUANTITY * CHARS_IN_VALUE];

uint8_t *motor_speed_to_char(uint8_t *buff, int16_t value) {
    int8_t i;

    for(i = 0; i < CHARS_IN_VALUE; i++) {
        buff[i] = '0';
    }

    // set number polarity
    if(value < 0) {
        value *= -1;
        buff[0] = '1';
    }

    for(i = 0; i < SPEED_VALUE_CHAR_LENGTH; i++) {
        if((value & 0xf) > 9) {
            buff[i + 1] = (value & 0xf) + 'A' - 10;
        } else {
            buff[i + 1] = (value & 0xf) + '0';
        }
        value = value >> BITS_IN_CHAR;
    }

    return buff;
}

void set_motors_speeds(int16_t motor_speed_1, int16_t motor_speed_2, int16_t motor_speed_3, int16_t motor_speed_4, int16_t motor_speed_5) {
	int16_t motors[MAX_MOTORS_QUANTITY] = {motor_speed_1, motor_speed_2, motor_speed_3, motor_speed_4, motor_speed_5, 0};
	uint8_t k, l, motor_index;

	// set not used movement motors do nothing (0)
	for(k = 0; k < NOT_USED_MOTORS; k++) {
		for (l = 0; l < CHARS_IN_VALUE; l++) {
			message[COMMAND_CHAR_LENGTH + CHARS_IN_VALUE * ((MAX_MOTORS_QUANTITY - 1) - k) + l] = 0;
		}
	}

	// movement motor command BB04 
	message[0] = 'B';
	message[1] = 'B';
	message[2] = '0';
	message[3] = '4';

	motor_index = 0;
	for(k = COMMAND_CHAR_LENGTH; k < MESSAGE_CHAR_LENGTH; k += CHARS_IN_VALUE) {
		motor_speed_to_char(motor_speed_buff, motors[motor_index]);
		for(l = 0; l < CHARS_IN_VALUE; l++) {
			message[k + l] = motor_speed_buff[l];
		}   
		motor_index++;
	}
}

void change_motors_speeds(int16_t ms1, int16_t ms2, int16_t ms3, int16_t ms4, int16_t ms5, uint8_t sender_thread_id) {
    writing_motors_speeds[0] = ms1;
    writing_motors_speeds[1] = ms2;
    writing_motors_speeds[2] = ms3;
    writing_motors_speeds[3] = ms4;
    writing_motors_speeds[4] = ms5;
    chMsgSend(motor_thread, (msg_t)sender_thread_id);
}

THD_WORKING_AREA(waMotorThread, 128);
THD_FUNCTION(MotorThread, arg) {
    (void)arg;

    thread_t *master_thread;
    int8_t i;

    while (1) {
        master_thread = chMsgWait();
        chMsgGet(master_thread);
        
        set_motors_speeds(writing_motors_speeds[0], writing_motors_speeds[1], writing_motors_speeds[2], writing_motors_speeds[3], writing_motors_speeds[4]);
        
        chMsgRelease(master_thread, MSG_OK);

        sdPut(MOTOR_SERIAL, '\n');
        for(i = 0; i < MESSAGE_CHAR_LENGTH; i++) {
            sdPut(MOTOR_SERIAL, message[i]);
        }
        sdPut(MOTOR_SERIAL, '\n');
    }
}

void motors_init(void) {
	uint8_t init_message[INIT_MESSAGE_LENGTH] = {'\n', 0x80, 0xC5, '\n', 'A', 'A', '0', '1', '1', '1', '1', '1', '1', '\n'};
    int8_t i;
    
    for(i = 0; i < INIT_MESSAGE_LENGTH; i++) {
        sdPut(MOTOR_SERIAL, init_message[i]);
    }

    motor_thread = chThdCreateStatic(waMotorThread, sizeof(waMotorThread), NORMALPRIO, MotorThread, NULL);
}
