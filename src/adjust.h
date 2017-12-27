/*
 * adjust.h
 *
 *  Created on: 2016/10/21
 *      Author: PCーUSER
 */

#ifndef ADJUST_H_
#define ADJUST_H_

#include "wall_control.h"
#include "wall_control_define.h"
#include "motor.h"
#include "motor_define.h"
#include "serial.h"
#include "iodefine.h"
#include "rx631_init.h"
#include "shortest_run.h"
#include "search.h"


void adjust_slalom(char number);
void adjust_all_slalom();
void adjust_small_slalom();
void adjust_big_slalom();
void adjust_big_180_slalom();
void adjust_oblique_slalom();
void adjust_90_oblique_slalom();

//大回り速度
extern double big_slalom_center_velocity;
extern double left_forward_oblique_wall_control_constant;
extern double right_forward_oblique_wall_control_constant;
extern double left_oblique_wall_control_constant;
extern double right_oblique_wall_control_constant;

#endif /* ADJUST_H_ */
