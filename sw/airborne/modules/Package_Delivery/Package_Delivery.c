//      Package_Delivery.c
//      
//      Copyright 2012 Arnaud van Helden <avanhelden@woonkamer>
//      
//      This program is free software; you can redistribute it and/or modify
//      it under the terms of the GNU General Public License as published by
//      the Free Software Foundation; either version 2 of the License, or
//      (at your option) any later version.
//      
//      This program is distributed in the hope that it will be useful,
//      but WITHOUT ANY WARRANTY; without even the implied warranty of
//      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//      GNU General Public License for more details.
//      
//      You should have received a copy of the GNU General Public License
//      along with this program; if not, write to the Free Software
//      Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
//      MA 02110-1301, USA.
//      
//      


 
#include "Package_Delivery.h"
#include "subsystems/navigation/bomb.h"
#include <stdio.h>
#include <math.h>
#include "math/pprz_geodetic_float.h"
#include "estimator.h"
#include "generated/flight_plan.h"
float PackageGlobWindDir = 0;
float PackageGlobWindAverage = 0;
float PackageDefCd = PACKAGEDEFACD;
float PackageDefAltitude = PACKAGEDEFALTITUDE;
float PackageDefServoDelay = PACKAGEDEFSERVODELAY;
float PackageDefAltitudeReference = 0;
float PackageMass = PackageDefMASS;
float PackageVerticalSpeed = 0;
float PackageGlobAirspeed = -3 ;
float PackageGlobDir = 0;
float PackageStartDist =PACKAGESTARTDIST;
float PackAlpha = PACKAGEDEFALPHA;
float PackageReleaseDistance =0;
bool_t PackageData_Switch=FALSE, PackageSpeed_Switch=FALSE, PackageDelivery_Reset_Switch=FALSE;

////////////////////////////////////////////////////////////////////////////////////////////////
//for fast debbuging, the simulation can be accelerated using the gaia software from an xterm console
//              /home/bvdp/paparazzi3/sw/simulator/gaia
////////////////////////////////////////////////////////////////////////////////////////////////
// for explanations about debugging macros:
//http://gcc.gnu.org/onlinedocs/cpp/Stringification.html#Stringification

// Be carefull not to use printf function in ap compilation, only use it in sim compilation
// the DEBUG_PRINTF should be defined only in the sim part of the makefile airframe file
#ifdef DEBUG_PD
int PDDEBUG=0;
#define PDPRTDEB(TYPE,EXP) \
printf("%5d: " #EXP ": %"#TYPE"\n",PDDEBUG,EXP);fflush(stdout);PDDEBUG++;
#define PDPRTDEBSTR(EXP) \
printf("%5d: STR: "#EXP"\n",PDDEBUG);fflush(stdout);PDDEBUG++;
#else
#define PDPRTDEB(TYPE,EXP) \
;

#define PDPRTDEBSTR(EXP) \
;
#endif

/*
 exemple of use for theese macros
 PRTDEBSTR(Init polysurvey)
 PRTDEB(u,SurveySize)

 PRTDEB(lf,PolygonCenter.x)
 PRTDEB(lf,PolygonCenter.y)
 */
////////////////////////////////////////////////////////////////////////////////////////////////


void PackageDeliveryInit(void){
}
void PackageDeliveryReset(void){
  #ifdef FastTrigger_H
    if(FastTriggerSwitch==TRUE){
    FastTriggerSwitch = FALSE;
    PDPRTDEBSTR("FastTriggerSwitch Reset")
  }
  #endif
  if(PackageSpeed_Switch == TRUE){
  PackageSpeed_Switch =FALSE;
  PDPRTDEBSTR("PackageSpeed_Switch Reset")
  }
//   PackageGlobWindDir = 0;
//   PackageGlobWindAverage = 0;
//   PackageDefCd = PACKAGEDEFACD;
//   PackageDefAltitude = PACKAGEDEFALTITUDE;
//   PackageDefServoDelay = PACKAGEDEFSERVODELAY;
//   PackageMass = PackageDefMASS;
//   PackageVerticalSpeed = -5;
//   PackageGlobAirspeed = -5;
  
}

void PackageDeliveryPeriodic(void){
  /** change the update function to work with the new forward speed which is updated each second */
  if (PackageSpeed_Switch == TRUE){
    if (estimator_hspeed_mod>PackageGlobAirspeed*1.1 || estimator_hspeed_mod<PackageGlobAirspeed*0.9){
      PackageGlobAirspeed = estimator_hspeed_mod;
      PackageGlobDir = estimator_hspeed_dir;
      PackageVerticalSpeed = estimator_z_dot;
      PackageCalculateDistance(); 
    } 
  #ifdef FastTrigger_H
    if(FastTriggerSwitch==TRUE){
      // distance to waypoint in x 
      float pw_x = waypoints[WP_RELEASE].x - estimator_x ;
      // distance to waypoint in y 
      float pw_y = waypoints[WP_RELEASE].y - estimator_y;
      // modulus of the distance
      float Package_Delivery_dist_to_wp = sqrtf(pw_x*pw_x + pw_y*pw_y);
      if (Package_Delivery_dist_to_wp<5){
	// /FIXME The distance should be dynamic based on the forward speed three times the distance covered in an update
	// If the UAV aproaches the RELEASE, stop moving the RELEASE point
	PackageSpeed_Switch =FALSE;
	PDPRTDEBSTR("PackageSpeed_Switch Distance FastTrigger")
	PDPRTDEB(f,Package_Delivery_dist_to_wp)       
      }
    }
  #else
    // distance to waypoint in x 
    float pw_x = waypoints[WP_RELEASE].x - estimator_x ;
    // distance to waypoint in y 
    float pw_y = waypoints[WP_RELEASE].y - estimator_y;
    // modulus of the distance
    float Package_Delivery_dist_to_wp = sqrtf(pw_x*pw_x + pw_y*pw_y);
    if (Package_Delivery_dist_to_wp<5){
      // /FIXME The distance should be dynamic based on the forward speed three times the distance covered in an update
      // If the UAV aproaches the RELEASE, stop moving the RELEASE point
      PackageSpeed_Switch =FALSE;
      PDPRTDEBSTR("PackageSpeed_Switch Distance")
      PDPRTDEB(f,Package_Delivery_dist_to_wp)       
    }
  #endif
  }
  if (PackageData_Switch ==TRUE){
      PackageData_Switch =FALSE;
      PackageAlign();
  }
  if (PackageDelivery_Reset_Switch==TRUE){
    PackageDelivery_Reset_Switch = FALSE;
    PackageDeliveryReset();
  }
}

void PackageAlign(void){
  /* This function will set the start and end waypoint 
   * then calculate the RELEASE point for the first time.
   * 
   */
  PackageGlobWindDir = atan2f(wind_north,wind_east)+M_PI;
  PackageGlobWindAverage = sqrtf(wind_east*wind_east + wind_north*wind_north);
  PackageDefAltitudeReference = estimator_z;
  //store the reference 
  PackageGlobAirspeed = estimator_hspeed_mod;
  PackageGlobDir = estimator_hspeed_dir;
  PackageVerticalSpeed = estimator_z_dot;
  
  PackageCalculateDistance();
    float x,y,a;
  x = waypoints[WP_TARGET].x -(PackageStartDist*cosf(PackageGlobWindDir)) + nav_utm_east0 ;
  y = waypoints[WP_TARGET].y -(PackageStartDist*sinf(PackageGlobWindDir)) + nav_utm_north0 ;
  a = PackageDefAltitude ;  
  nav_move_waypoint((WP_START), x, y, a);
  
  x = waypoints[WP_TARGET].x + (PackageStartDist*cosf(PackageGlobWindDir)) + nav_utm_east0 ;
  y = waypoints[WP_TARGET].y + (PackageStartDist*sinf(PackageGlobWindDir)) + nav_utm_north0 ;
  a = PackageDefAltitude ;  
  nav_move_waypoint((WP_END), x, y, a);
  // start the recalculate if the speed changes. 
  PackageSpeed_Switch = TRUE;
}

void PackageCalculateDistance(void){
  /* This is the  main function that calculates the distance that is travelled by the package before it hits the ground
   * The function will also put the waypoints in the correct place
   * first make sure that the file is not started again
   * initalize all the variables */
//   initialise the local variables.PackAlpha,
  float  PackStepSize, Z, Z1,Z2, Vz, Vz1, Vx, Vx1, Vg, X, X1, k1, k2, k3, k4, PackWindConstant;
  int k;
  
  if (PackageGlobAirspeed <0) PackageGlobAirspeed = estimator_hspeed_mod; 
//   Calculate the Distance travelled using Runge Kutta 4, Euler is unreliable
 PDPRTDEBSTR("Calculate Distance")
//  initial condition
  PackStepSize = 0.1;
  PackAlpha = PACKAGEDEFALPHA;
  Z  = PackageDefAltitudeReference - ground_alt;
  Vz = PackageVerticalSpeed;
  Vg = PackageGlobAirspeed;
  X  = PackageGlobAirspeed*PackageDefServoDelay;  
  
  //  calculate some constants
  PackWindConstant = PackageGlobWindAverage / logf(PackageDefAltitudeReference/PackageDefRoughnessLength);
  
//   PDPRTDEB(f,Z)
//   PDPRTDEB(f,Vz)
//   PDPRTDEB(f,Vg)
//   PDPRTDEB(f,X)
//   PDPRTDEB(f,PackWindConstant)
//   PDPRTDEB(f,PackStepSize)
//   PDPRTDEB(f,PackAlpha)
//   
  
  //  initialise the counter
  k = 1;
  while (Z >0.0 && k <500){
    // add feature, if terminal velocity is reached, the function becomes simpler due to constant speed, this should reduce the calculation steps. 
//update the airspeed
//  PDPRTDEBSTR("Runge Kutta") 
  if (Z<PackageDefRoughnessLength) Z2=PackageDefRoughnessLength;
  else Z2 = Z;
    Vx = Vg + logf(Z2/PackageDefRoughnessLength)*PackWindConstant;

//calculate the next step
    k1 = (PackAlpha * fabsf(Vz) * sqrtf(Vz*Vz+Vx*Vx) - PackageDefG);
    k2 = (PackAlpha * fabsf(Vz+0.5f*k1*PackStepSize) * sqrtf((Vz+0.5f*k1*PackStepSize)*(Vz+0.5f*k1*PackStepSize) + (Vx+0.5f*k1*PackStepSize)*(Vx+0.5f*k1*PackStepSize)) - PackageDefG);
    k3 = (PackAlpha * fabsf(Vz+0.5f*k2*PackStepSize) * sqrtf((Vz+0.5f*k2*PackStepSize)*(Vz+0.5f*k2*PackStepSize) + (Vx+0.5f*k2*PackStepSize)*(Vx+0.5f*k2*PackStepSize)) - PackageDefG);
    k4 = (PackAlpha * fabsf(Vz+k3*PackStepSize) * sqrtf((Vz+k3*PackStepSize)*(Vz+k3*PackStepSize) + (Vx+k3*PackStepSize)*(Vx+k3*PackStepSize)) - PackageDefG);
    Vz1 = Vz + (1.0f/6.0f)*(k1 + 2.0f*k2+ 2.0f*k3 + k4)*PackStepSize;

    
//       PDPRTDEB(f,k1)
//   PDPRTDEB(f,k2)
//   PDPRTDEB(f,k3)
//   PDPRTDEB(f,k4)
//       PDPRTDEB(f,Vz1)
    
    
    k1 = Vz;
    k2 = Vz + 0.5f*k1*PackStepSize;
    k3 = Vz + 0.5f*k2*PackStepSize;
    k4 = Vz + k3*PackStepSize;
    Z1 = Z + (1.0f/6.0f)*(k1 + 2.0f*k2+ 2.0f*k3 + k4)*PackStepSize;
            
    k1 = -PackAlpha * (fabsf(Vx) * sqrtf(Vz*Vz+Vx*Vx));
    k2 = -PackAlpha * (fabsf(Vx+0.5f*k1*PackStepSize) * sqrtf((Vz+0.5f*k1*PackStepSize)*(Vz+0.5f*k1*PackStepSize) + (Vx+0.5f*k1*PackStepSize)*(Vx+0.5f*k1*PackStepSize)));
    k3 = -PackAlpha * (fabsf(Vx+0.5f*k2*PackStepSize) * sqrtf((Vz+0.5f*k2*PackStepSize)*(Vz+0.5f*k2*PackStepSize)+ (Vx+0.5f*k2*PackStepSize)*(Vx+0.5f*k2*PackStepSize)));
    k4 = -PackAlpha * (fabsf(Vx+k3*PackStepSize) * sqrtf((Vz+k3*PackStepSize)*(Vz+k3*PackStepSize) + (Vx+k3*PackStepSize)*(Vx+k3*PackStepSize)));
    Vx1 = Vx + (1.0f/6.0f)*(k1 + 2.0f*k2+ 2.0f*k3 + k4)*PackStepSize;
                
    k1 = Vg;
    k2 = Vg + 0.5f*k1*PackStepSize;
    k3 = Vg + 0.5f*k2*PackStepSize;
    k4 = Vg + k3*PackStepSize;
    X1 = X + (1.0f/6.0f)*(k1 + 2.0f*k2+ 2.0f*k3 + k4)*PackStepSize;
   
 
  
//   PDPRTDEB(f,Z1)
//   PDPRTDEB(f,Vz1)
//   PDPRTDEB(f,Vx1)
//   PDPRTDEB(f,X1)
                
//preparing for the next step
    k++;
    Vz = Vz1;
    Z = Z1;
    if (Z1<PackageDefRoughnessLength) Z1=PackageDefRoughnessLength;
    Vg = Vx1-logf(Z1/PackageDefRoughnessLength)*PackWindConstant;
    X = X1; 
//      PDPRTDEB(f,Z)
//   PDPRTDEB(f,Vz)
//   PDPRTDEB(f,Vg)
//   PDPRTDEB(f,X)
//   PDPRTDEB(i,k)
} 
PDPRTDEBSTR("End Calc")
     PDPRTDEB(f,Z)
  PDPRTDEB(f,Vz)
  PDPRTDEB(f,Vg)
  PDPRTDEB(f,X)
  PDPRTDEB(i,k)
    
  if(X !=X)X=-100.0;
  float x,y,a;
  x = waypoints[WP_TARGET].x - X*cosf(PackageGlobWindDir) + nav_utm_east0 ;
  y = waypoints[WP_TARGET].y - X*sinf(PackageGlobWindDir) + nav_utm_north0 ;
  a = PackageDefAltitude ;  
  nav_move_waypoint((WP_RELEASE), x, y, a);
  PackageReleaseDistance=X;
}
