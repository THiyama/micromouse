/*
 * contest.h
 *
 *  Created on: 2016/10/15
 *      Author: PCーUSER
 */

#ifndef CONTEST_H_
#define CONTEST_H_

#include "wall_control.h"
#include "wall_control_define.h"
#include "motor.h"
#include "motor_define.h"
#include "serial.h"
#include "iodefine.h"
#include "rx631_init.h"
#include "shortest_run.h"
#include "search.h"

//大会のときに使う探索と最短アルゴリズム
void contest_search_shortest();
void contest_auto_start();


void circuit(double ,double ,double );

void contest_search();

extern int calucurate_count;
extern int calucurate_mode;


#endif /* CONTEST_H_ */
