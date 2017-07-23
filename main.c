#include "ch.h"
#include "hal.h"
#include "test.h"
#include "camera.h"
#include "config.h"
#include "compass.h"
#include "lines.h"
#include "motor.h"
#include "kicker.h"
#include <math.h>
#include "chprintf.h"

#define MAX_SPEED 923
#define NUMBER_OF_MOTORS 3
#define MAX_ROBOT_POWER (60+FERQUES*5)
#define MIN_ROBOT_POWER (40+FERQUES*5)
#define FAR_POINT 0
#define NEER_POINT 400
#define ALIGN_POWER 100

#define LINE_REACTION_TIME 10
#define NO_LINE_DETECTED -1

#define DRIBLER_SPEED 1600
#define NUMBER_OF_DRIBLERS 2

#define MASTER_THREAD_ID 1
#define PI 3.14159

#define ALIGN_TOLLERANCE 5

#define SHOOTING_RANGE 5
#define HAVING_BALL_RANGE 5

#define OBJECT_NOT_FOUND 420
#define SEARCH_SPEED 60

#define DRIBLER_REACTION_TIME (6+FERQUES*4)

#define CAMERA_CENTER 4
#define AROUND_BALL_ALIGN 0
#define BASIC_ALIGN 1
#define CLASSIC_ALIGN 2
#define NOT_ALIGN 3

int32_t motors_speeds[NUMBER_OF_MOTORS];
int16_t driblers_speeds[NUMBER_OF_DRIBLERS];

int16_t movement_degree = 90;
int16_t line_timer = 0;
int16_t old_line = 0;
int16_t old_azimuth = 0;

int16_t dribler_timer = 0;

int16_t ball_degree = OBJECT_NOT_FOUND;
int16_t ball_distance = OBJECT_NOT_FOUND;
int16_t goal_degree = OBJECT_NOT_FOUND;
int16_t goal_distance = OBJECT_NOT_FOUND;

int8_t have_ball = 0;

int8_t using_align = CLASSIC_ALIGN;


int16_t robot_power(void){
	//return 50;
	//return ((-ball_distance+650)/4 < 100)? 100 : (-ball_distance+650)/4;
	int16_t a = MAX_ROBOT_POWER - (ball_distance - FAR_POINT)*(MAX_ROBOT_POWER - MIN_ROBOT_POWER)/(NEER_POINT-FAR_POINT); // rychlost_rastu = 4, celkova_rychlost = 650
	if(a > MAX_ROBOT_POWER) a = MAX_ROBOT_POWER;
	if(a < 0) a = 0;
	if(dribler_timer > 0  && dribler_timer < 6) a = 100;
	if(ball_degree == OBJECT_NOT_FOUND) a = MAX_ROBOT_POWER;
	return a;
}

double abs_value_double(double a) {
	if (a < 0) a *= -1;
	return a;
}

int16_t abs_value_int(int16_t a) {
	if (a < 0) a *= -1;
	return a;
}

void motors_off(void) {
	int8_t p;
	for (p = 0; p < NUMBER_OF_MOTORS; p++) {
		motors_speeds[p] = 0;
	}
}

void dribler_on(void){
	palClearPad(GPIOD, 9);
	driblers_speeds[0] = DRIBLER_SPEED;
	driblers_speeds[1] = -DRIBLER_SPEED;
}

void dribler_off(void) {
	palSetPad(GPIOD, 9);
	driblers_speeds[0] = 0;
	driblers_speeds[1] = 0;
}

int16_t set_motor_speed(int16_t relative_speed) {
	return relative_speed * MAX_SPEED / 100;
}

int16_t set_align_speed(int16_t relative_speed){
	if(ball_degree == OBJECT_NOT_FOUND) return relative_speed * set_motor_speed(140-robot_power()) / 180 / 100;
	return relative_speed * set_motor_speed(100-robot_power()) / 180 / 100;
}

void set_movement(int16_t degree) {
	int8_t p;
	if (degree == 100 || degree == 80) {
		degree = 90;
	}
	double  maximum_motor_value = 0;
	double  motors_values [NUMBER_OF_MOTORS];
	for (p = 0; p < NUMBER_OF_MOTORS; p++) {
		motors_values[p] = sin((degree - (90 * (NUMBER_OF_MOTORS - 2 + (4 * p)))
							  / NUMBER_OF_MOTORS) * PI / 180);
		if (maximum_motor_value < abs_value_double(motors_values[p])) {
			maximum_motor_value = abs_value_double(motors_values[p]);
		}
	}

	for (p = 0; p < NUMBER_OF_MOTORS; p++) {
		motors_speeds[p] = (int)(motors_values[p] * 1000 / maximum_motor_value);
	}
}

void correct_motors_speeds(int8_t align_type) {
	int8_t p;
	int32_t max = 0;
	int16_t actual_azimuth = 0;
	//actual_azimuth = -ball_degree;
	/*if(have_ball > 0)*/ actual_azimuth = -goal_degree;

	if(actual_azimuth == -OBJECT_NOT_FOUND) {
		//motors_off();
		if(have_ball > 0) align_type = AROUND_BALL_ALIGN;
		if(old_azimuth < 0) {
			actual_azimuth = -SEARCH_SPEED;
			//if(align_type == CLASSIC_ALIGN) actual_azimuth /= 3;
		} else {
			actual_azimuth = SEARCH_SPEED;
		}
	} else {
		old_azimuth = actual_azimuth;
	}
	//if(old_azimuth > 0 && align_type == CLASSIC_ALIGN) actual_azimuth /= 2;
	if(align_type == BASIC_ALIGN) {
		if(actual_azimuth > 5) actual_azimuth = SEARCH_SPEED;
		else if(actual_azimuth < -5) actual_azimuth = -SEARCH_SPEED;
	} else if(align_type == NOT_ALIGN){
		actual_azimuth = 0;
	}
	
    actual_azimuth *= set_align_speed(ALIGN_POWER);
	
	for (p = 0; p < NUMBER_OF_MOTORS; p++) {
		if (abs_value_int(motors_speeds[p]) > max) {
			max = abs_value_int(motors_speeds[p]);
		}
	}

	for (p = 0; p < NUMBER_OF_MOTORS; p++) {
		motors_speeds[p] = motors_speeds[p] * (set_motor_speed(robot_power()) - actual_azimuth) / max;
	}

	for (p = 0; p < NUMBER_OF_MOTORS; p++) {
		motors_speeds[p] += actual_azimuth;
	}
	
	if(align_type == AROUND_BALL_ALIGN) {
		motors_speeds[0] = 0;
		motors_speeds[1] = 0;
		motors_speeds[2] *= 3;
	}

	for (p = 0; p < NUMBER_OF_MOTORS; p ++) {
		if (motors_speeds[p] > 0) {
			motors_speeds[p] += 1125;
		} else if (motors_speeds[p] < 0) {
			motors_speeds[p] -= 1125;
		}
	}
}

void align(void){
	uint8_t p;
	int16_t actual_azimuth = -goal_degree;
	if(actual_azimuth == -OBJECT_NOT_FOUND) actual_azimuth = get_azimuth();

	if(abs_value_int(actual_azimuth) > ALIGN_TOLLERANCE) {
		for (p = 0; p < NUMBER_OF_MOTORS; p++) {
			motors_speeds[p] += actual_azimuth * 500 / 180;
		}
	}
}

int16_t ball_bypass(int16_t ball_degree, int16_t ball_distance){
	if (ball_distance > 50) {
		return (2*ball_degree-90+360) % 360;
	}
	return ball_degree;
}

int16_t line(void) {
	int16_t degree = check_line();
	if (degree == -1){
		if (line_timer > 0) {
			line_timer --;
			return old_line;
		} else {
			return degree;
		}
	} else {
		line_timer = LINE_REACTION_TIME;
		old_line = degree;
		return degree;
	}
}

int main(void) {
	halInit();
	chSysInit();

	board_init();

	motors_init();
	lines_init();
	compass_init();
	camera_init();

	int16_t lines_state;
	while (1) {
		get_camera_values();
		lines_state = line();
		
		ball_degree = get_ball_degree();
		ball_distance = get_ball_distance();
		goal_degree = get_goal_degree();
		goal_distance = get_goal_distance();
		
		//move to center
		if(ball_degree != OBJECT_NOT_FOUND) ball_degree -= CAMERA_CENTER;
		//if(goal_degree != OBJECT_NOT_FOUND) goal_degree -= CAMERA_CENTER;

		if(i_have_ball() && abs_value_int(ball_degree) < HAVING_BALL_RANGE){
			if(have_ball > 3) have_ball = 13;
			else have_ball = 8;
		}
		
		using_align = CLASSIC_ALIGN;
		
		if(get_start()){

			if(lines_state == NO_LINE_DETECTED) {
				if (ball_degree == OBJECT_NOT_FOUND) {
					set_movement(270);
				} else {
					set_movement(ball_bypass(90 - ball_degree, ball_distance));
				}
				/*if(have_ball > 0){
					motors_off();
					using_align = AROUND_BALL_ALIGN;
					//if(goal_degree == OBJECT_NOT_FOUND) using_align = AROUND_BALL_ALIGN;
					//else using_align = CLASSIC_ALIGN;
					//}
					if(abs_value_int(goal_degree) < 40){
						set_movement(ball_bypass(90 - ball_degree, ball_distance));
						using_align = CLASSIC_ALIGN;
					}
				}*/
				//using_align = NOT_ALIGN;
				//motors_off();
			} else {
				set_movement(lines_state);
			}

			//align();
			dribler_on();
			correct_motors_speeds(using_align);
		} else {
			dribler_off();
			motors_off();
		}
		//correct_motors_speeds();

		if((have_ball > 0 && kicker_is_ready() && /*abs_value_int(goal_degree) < SHOOTING_RANGE &&*/ get_shooting_event()) || dribler_timer > 0) {
			dribler_off();
			set_movement(90);
			correct_motors_speeds(CLASSIC_ALIGN);
			if(dribler_timer > 0 ){
				dribler_timer--;
			} else {
				dribler_timer = DRIBLER_REACTION_TIME;
			}
			//change_motors_speeds(0, 0, 0, 0, 0, MASTER_THREAD_ID);
			if(dribler_timer == 1) {
				//chThdSleepMilliseconds(200);
				kick();
			}
		} else {
			dribler_timer = 0;
		}
		change_motors_speeds(motors_speeds[0], motors_speeds[1], motors_speeds[2],
								 driblers_speeds[0], driblers_speeds[1], MASTER_THREAD_ID);
		//change_motors_speeds(2048, -2048, 0, 0, 0, MASTER_THREAD_ID); //ultimate speed
		if(have_ball > 0) have_ball--;
		chThdSleepMilliseconds(10);//musim si odichnut
	}
}
