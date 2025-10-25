#ifndef MOTER_H
#define MOTER_H

#include "dji_motor.h"
#include "cybergear_motor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "string.h"
#include <stdlib.h>
#include "cmsis_os.h"
#include "trajectory_planner.h"
#include "semphr.h"

void chassis_init(void);
void chassis_control(float Angle);
void dji_init(void);
void zhuzhou_control(float height);
void xiaobi_control(float angle);
void dabi_control(float length);
void xipan_control(int open);
bool all_motors_in_position(void);
void chassic_get(void);
#endif