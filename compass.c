#include "ch.h"
#include "hal.h"

#include <math.h>
#include "compass.h"
#include "chprintf.h"

#define COMPASS_SERIAL &SD1

#define MAIN 400

thread_t *compass_process_thread;
thread_t *compass_read_thread;

int16_t attack_degree = 0;

int16_t actual_angle = -1, final_degree = -1;

THD_WORKING_AREA(waCompassReadThread, 128);
THD_FUNCTION(CompassReadThread, arg) {
    (void)arg;

    int16_t dir;
    uint8_t received_char;

    while (1) {
        dir = 0;
        received_char = sdGet(COMPASS_SERIAL);
        while(received_char != '\n') {
            dir += received_char - '0';
            dir *= 10;
            received_char = sdGet(COMPASS_SERIAL);
        }
        dir /= 10;

        chMsgSend(compass_process_thread, (msg_t)dir);
    }
}

THD_WORKING_AREA(waCompassProcessThread, 128);
THD_FUNCTION(CompassProcessThread, arg) {
    (void)arg;

    thread_t *master;
    msg_t msg;

    while(1) {
        master = chMsgWait();
        msg = chMsgGet(master);
        if(msg < 360) {
            actual_angle = (int16_t)msg;
        }
        chMsgRelease(master, (msg_t)actual_angle);

    }
 
}

int16_t get_compass_degree(void) {
    return (int16_t)chMsgSend(compass_process_thread, MAIN);
}

int16_t get_azimuth(void) {
    int16_t azimuth;
    azimuth = ((get_compass_degree() - attack_degree) + 360) % 360;
    if (azimuth > 180) azimuth -= 360;
    return azimuth;
}

void compass_init(void) {
    compass_read_thread = chThdCreateStatic(waCompassReadThread, sizeof(waCompassReadThread), NORMALPRIO, CompassReadThread, NULL);
    compass_process_thread = chThdCreateStatic(waCompassProcessThread, sizeof(waCompassProcessThread), NORMALPRIO, CompassProcessThread, NULL);
    chThdSleepMilliseconds(100);
    attack_degree = get_compass_degree();
}
