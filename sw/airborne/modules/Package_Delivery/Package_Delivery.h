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
 
#ifndef Package_Delivery_H
#define Package_Delivery_H

#include "subsystems/navigation/common_nav.h"
#include "estimator.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "subsystems/navigation/bomb.h"
#include "std.h"
#include "messages.h"
#include <stdio.h>
#include <math.h>
/* Package Drag Coefficient * the frontal Area  */
#ifndef  PACKAGEDEFACD
#define PACKAGEDEFACD 0.00
#endif
/* air density in kg/m3  */
#define PackageDefRHO 1.225
/* gravity in m/s2                                 */
#define PackageDefG 9.81
#ifndef PackageDefMASS
/* mass in kg                                      */
#define PackageDefMASS 0.024
#endif
/* The surface roughness factor for the wind */
#ifndef PackageDefRoughnessLength
#define PackageDefRoughnessLength 0.14
#endif
/* The drop altitude [m]*/
#ifndef PACKAGEDEFALTITUDE
#define PACKAGEDEFALTITUDE 40.0
#endif
#ifndef PACKAGEDEFALPHA
#define PACKAGEDEFALPHA 0.16765
#endif

#ifndef PACKAGEDEFSERVODELAY
#define PACKAGEDEFSERVODELAY 1.0
#endif
#ifndef PACKAGESTARTDIST
#define PACKAGESTARTDIST 30.0
#endif

#define PACKAGERELEASEDISTANCE 10

/* These functions below are the different switches that are used by the program
 * PackageData is used when the windfinder is used
 * PackagePappa is used when the internal wind program is used
 * PackageDelivery is the used to calculate the location of the three waypoints
 * PackageReset will reset the package delivery array
 * PackageWindSpeed will set the 2 align points in line with the wind to find the forward wind speed
*/
extern bool_t PackageData_Switch, PackageSpeed_Switch, PackageDelivery_Reset_Switch;
#define PackageData()({PackageData_Switch =  TRUE;FALSE;})
#define PackageSpeed()({PackageSpeed_Switch = TRUE;FALSE;})
#define PackageReset()({PackageDelivery_Reset_Switch= TRUE;FALSE;})


extern float PackageGlobWindDir;
extern float PackageGlobWindAverage ;
extern float PackageDefCd ;
extern float PackageDefAltitude ;
extern float PackageDefServoDelay ;
extern float PackageDefAltitudeReference ;
extern float PackageMass;
extern float PackageVerticalSpeed ;
extern float PackageGlobAirspeed ;
extern float PackageGlobDir;
extern float PackageStartDist;
extern float PackAlpha;
extern float PackageReleaseDistance;

void PackageDeliveryInit(void);
void PackageDeliveryReset(void);
void PackageDeliveryPeriodic(void);
void PackageAlign(void);
void PackageCalculateDistance(void);


#endif // Package_Delivery_H
