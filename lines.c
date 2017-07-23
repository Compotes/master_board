#include "hal.h"
#include "ch.h"
#include "chprintf.h"
#include "lines.h"
#include <math.h>

#define LINES_SERIAL &SD6
#define INIT_MESSAGE_LENGTH 3
#define COMMAND_MESSAGE_LENGTH 9
#define RESET_LENGTH 1
#define COMMAND_LENGTH 4
#define VALUES_LENGTH 4
#define NUMBER_OF_SENSORS 16
#define INERTIA 0.95
#define LINES_AWAY_DISTANCE 10
#define PI 3.14159


int32_t round_values[NUMBER_OF_SENSORS];
uint8_t values[4];
uint8_t init_queue = 0;
uint8_t messages_buffer[INIT_MESSAGE_LENGTH + COMMAND_MESSAGE_LENGTH] = {'\n', 0x80, 0xC4,'\n', 'F', 'F', '8', '1', '0', '0', '0', '0'};

static double round_data_values[NUMBER_OF_SENSORS];
static int16_t dx[16] = {20,56,83,98,98,83,56,20,-20,-56,-83,-98,-98,-83,-56,-20};
static int16_t dy[16] = {98,83,56,20,-20,-56,-83,-98,-98,-83,-56,-20,20,56,83,98};
static int16_t result_degree;
static int16_t res_x, res_y;
static int16_t i_see_line = 0;

thread_t *writingp;
thread_t *readingp;
thread_t *workingp;

uint8_t char_to_value(uint8_t character) {
    if (character >= 'A') {
        character = character + 10 - 'A';
    } else {
        character = character - '0';
    }
    return character;
}
 
void determine_avoiding_direction(void) {
    int8_t i;
	res_x = 0, res_y = 0;
    
    i_see_line = 0;
    
	for(i = 0; i < NUMBER_OF_SENSORS; i++) {
		round_data_values[i] = round_data_values[i] * INERTIA + round_values[i] * (1.0 - INERTIA);
		if (round_data_values[i] > 0.99 - INERTIA) {
			res_x += dx[i];
			res_y += dy[i];
		}
		if (round_values[i] == 1 && i_see_line == 0) {
			i_see_line = 1;
		}
	}        
	result_degree = ((int16_t)(atan2(res_y, res_x) / PI * 180) + 405 ) % 360;
}
  
int16_t check_line(void) {
	if (i_see_line == 0) return -1;
    else return result_degree;
}


THD_WORKING_AREA(waLineWritingThread, 128);
THD_FUNCTION(LineWritingThread, arg) {
	(void)arg;
	
    uint8_t msg_length = 0, msg_start_index = 0;
    int8_t i;

    init_queue = 1;

	while (1) {
		if(init_queue == 1) {
			msg_length = INIT_MESSAGE_LENGTH;
			chMsgSend(readingp, (msg_t)msg_length);
			init_queue = 0;
            msg_start_index = 0;
		} else {
            msg_length = COMMAND_MESSAGE_LENGTH;
            chMsgSend(readingp, (msg_t)msg_length);
            msg_start_index = INIT_MESSAGE_LENGTH;
        }

        for(i = msg_start_index; i < (msg_start_index + msg_length); i++) {
            sdPut(LINES_SERIAL, messages_buffer[i]);
        }
	}
}


THD_WORKING_AREA(waLineReadingThread, 128);
THD_FUNCTION(LineReadingThread, arg) {
	(void)arg;

	thread_t *writing_thread;
	uint8_t msg_length, received_char;
    int8_t i;

	while (1) {
		writing_thread = chMsgWait();
		msg_length = (uint8_t)chMsgGet(writing_thread);
        chMsgRelease(writing_thread, MSG_OK);

        for(i = 0; i < msg_length; i++) {
            received_char = sdGet(LINES_SERIAL);
            if(received_char != '\n' && i == 0) {
                i--;
            } else {
                if(msg_length != INIT_MESSAGE_LENGTH) {
                    if(i > ((RESET_LENGTH + COMMAND_LENGTH) - 1)) {
                        values[i - (RESET_LENGTH + COMMAND_LENGTH)] = char_to_value(received_char);
                    }
                }
            }
        }
        
        if(msg_length != INIT_MESSAGE_LENGTH) {
            chMsgSend(workingp, (msg_t)1);
        }
	}
}


THD_WORKING_AREA(waLineWorkingThread, 128);
THD_FUNCTION(LineWorkingThread, arg) {
	(void)arg;
    
    thread_t *reading_thread;
    int8_t i, j, f;

	while (1) {
        reading_thread = chMsgWait();
        chMsgGet(reading_thread);
        f = 0;
        for(i = 0; i < 4-FERQUES; i++) {
			if(i == 1) f = FERQUES;
            for(j = 0; j < 4; j++) {
                round_values[4*(i+f) + j] = (values[i] >> j) & 1;
            }
        }

        chMsgRelease(reading_thread, MSG_OK);
        determine_avoiding_direction();
	}
}
 
void lines_init(void) {
    
    int8_t i;
	
    for (i = 0; i < NUMBER_OF_SENSORS; i++) {
		round_data_values[i] = 0;
	}
    
	i_see_line = 0; // 0 --> false, I don't see line

    readingp = chThdCreateStatic(waLineReadingThread, sizeof(waLineReadingThread), NORMALPRIO, LineReadingThread, NULL);
    writingp = chThdCreateStatic(waLineWritingThread, sizeof(waLineWritingThread), NORMALPRIO, LineWritingThread, NULL);
	workingp = chThdCreateStatic(waLineWorkingThread, sizeof(waLineWorkingThread), NORMALPRIO, LineWorkingThread, NULL);
}
