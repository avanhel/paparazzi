/*
 * $Id: $
 *
 * Copyright (C) 2012 TUDelft Arnaud van Helden
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */
 
#ifndef FastTrigger_H
#define FastTrigger_H
#include "Package_Delivery/Package_Delivery.h"
#include "estimator.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#ifndef FASTTRIGGERMINDIST
#define FASTTRIGGERMINDIST 2.0
#endif
#ifndef FASTTRIGGERMINANGLE
#define FASTTRIGGERMINANGLE 0.1
#endif

extern bool_t FastTriggerSwitch;
#define FastTrigger() ({FastTriggerSwitch = TRUE;FALSE;})
extern float last_gps_position_x;
extern float my_fast_position_x;
extern float last_gps_position_y;
extern float my_fast_position_y;
extern float FastTriggerMinDist;
extern float FastTriggerMinAngle;
//extern float PackageGlobWindDir;

void FastTriggerInit(void);
void FastTriggerPeriodic(void);
void FastTriggerAproach(void);
#endif // FastTrigger_H
