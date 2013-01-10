
/** \brief integrate the ground speed to find the distance travelled
 *  integrates the distance travelled at 60 hz and checks if the GPS has been updated.
 *  Then check if the distance between the UAV and the release point is close enough for
 *  the drop
 */

#include "estimator.h"
#include "Package_Delivery/Package_Delivery.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "subsystems/navigation/bomb.h"
#include "stdio.h"
#include "std.h"  //macros pas mal dans sw/include

float last_gps_position_x = 0;
float my_fast_position_x = 0;
float last_gps_position_y = 0;
float my_fast_position_y = 0;
float FastTriggerMinDist = FASTTRIGGERMINDIST;
float FastTriggerMinAngle = FASTTRIGGERMINANGLE;
bool_t FastTriggerSwitch = FALSE;

////////////////////////////////////////////////////////////////////////////////////////////////
//for fast debbuging, the simulation can be accelerated using the gaia software from an xterm console
//              /home/bvdp/paparazzi3/sw/simulator/gaia
////////////////////////////////////////////////////////////////////////////////////////////////
// for explanations about debugging macros:
//http://gcc.gnu.org/onlinedocs/cpp/Stringification.html#Stringification

// Be carefull not to use printf function in ap compilation, only use it in sim compilation
// the DEBUG_PRINTF should be defined only in the sim part of the makefile airframe file
#ifdef DEBUG_FT
int FTDEBUG=0;
#define FTPRTDEB(TYPE,EXP) \
printf("%5d: " #EXP ": %"#TYPE"\n",FTDEBUG,EXP);fflush(stdout);FTDEBUG++;
#define FTPRTDEBSTR(EXP) \
printf("%5d: STR: "#EXP"\n",FTDEBUG);fflush(stdout);FTDEBUG++;
#else
#define FTPRTDEB(TYPE,EXP) \
;

#define FTPRTDEBSTR(EXP) \
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
void FastTriggerInit(void){
  last_gps_position_x = 0;
  last_gps_position_y = 0;
 my_fast_position_x = 0;
 my_fast_position_y = 0;
}

void FastTriggerPeriodic(void){
  if(FastTriggerSwitch==TRUE){
    if (last_gps_position_x != estimator_x){
      // We have new GPS information: copy
      my_fast_position_x = estimator_x;
      my_fast_position_y = estimator_y;
      last_gps_position_x = estimator_x;
      last_gps_position_y = estimator_y;
    }
    else{
      // No fresh GPS data: integrate
    my_fast_position_x += estimator_hspeed_mod * sinf(estimator_hspeed_dir) * (1.0f/60.0f);
    my_fast_position_y += estimator_hspeed_mod * cosf(estimator_hspeed_dir) * (1.0f/60.0f);
    }
    // find the distance between the UAV and the Release target and release if needed
    FastTriggerAproach();
  }

}



/** \brief Decide if the UAV is at the release waypoint.
 *  FastTriggerAproach checks if UAV has passed the waypoint or is very close to the waypoint.
 *  The program opens the hatch and switches the fast trigger function off.
 */
void FastTriggerAproach(void) {
  // calculate the flight direction based on the start and end point
  float FastTriggerFlightDirection = atan2(waypoints[WP_START].y-waypoints[WP_END].y , waypoints[WP_START].x-waypoints[WP_END].x)+M_PI;  
  // distance to waypoint in x 
  float pw_x = waypoints[WP_RELEASE].x - my_fast_position_x ;
  // distance to waypoint in y 
  float pw_y = waypoints[WP_RELEASE].y - my_fast_position_y;
  // modulus of the distance
  float FastTrigger_dist_to_wp = sqrtf(pw_x*pw_x + pw_y*pw_y);

// float FastTrigger_angle_to_wp = atan2f(my_fast_position_y,my_fast_position_x)+M_PI;
  float FastTrigger_angle_to_wp = atan2f(pw_y,pw_x) - FastTriggerFlightDirection;

// PRTDEB(f,FastTrigger_angle_to_wp)
FTPRTDEB(f,FastTrigger_dist_to_wp)
  
    if (FastTrigger_angle_to_wp < 0){
        FastTrigger_angle_to_wp = FastTrigger_angle_to_wp+(2*M_PI);
   FTPRTDEBSTR("smaller than zero")
    }
    else if(FastTrigger_angle_to_wp >(2*M_PI)){
        FastTrigger_angle_to_wp = FastTrigger_angle_to_wp-(2*M_PI);
   FTPRTDEBSTR("larger than 2pi")
    }

    if (FastTrigger_angle_to_wp < 0){
        FastTrigger_angle_to_wp = FastTrigger_angle_to_wp+(2*M_PI);
   FTPRTDEBSTR("smaller than zero 2")
    }
    else if(FastTrigger_angle_to_wp >(2*M_PI)){
        FastTrigger_angle_to_wp = FastTrigger_angle_to_wp-(2*M_PI);
   FTPRTDEBSTR("larger than 2pi 2")
    }
          
FTPRTDEB(f,FastTrigger_angle_to_wp)    
    if (FastTrigger_angle_to_wp <M_PI/2*(1+FastTriggerMinAngle) && FastTrigger_angle_to_wp> M_PI/2*(1-FastTriggerMinAngle) && FastTrigger_dist_to_wp < FastTriggerMinDist){
        BombShoot() ;
 FTPRTDEBSTR("test off 90")
	FastTriggerSwitch=FALSE;
	FastTriggerInit();
    }
    else if (FastTrigger_angle_to_wp <3*M_PI/2* (1+FastTriggerMinAngle) && FastTrigger_angle_to_wp> 3*M_PI/2*(1-FastTriggerMinAngle) && FastTrigger_dist_to_wp < FastTriggerMinDist){
              BombShoot() ;
 FTPRTDEBSTR("test off 270")
	FastTriggerSwitch=FALSE;
	FastTriggerInit();
    }
    else if (FastTrigger_dist_to_wp < estimator_hspeed_mod * (1.0f/60.0f) ){
              BombShoot() ;
 FTPRTDEBSTR("test off distance")
	FastTriggerSwitch=FALSE;
	FastTriggerInit();
  }
}
  
  
  
  
