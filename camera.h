#ifndef CAMERA_H_
#define CAMERA_H_

#define NUMBER_OF_VALUES 6

void get_camera_values(void);
int16_t get_goal_degree(void);
int16_t get_goal_distance(void);
int16_t get_ball_degree(void);
int16_t get_ball_distance(void);
int16_t get_shooting_event(void);
int16_t get_start(void);
int16_t process_value(void);
void camera_init(void);

#endif
