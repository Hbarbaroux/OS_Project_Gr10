//////////////
// INCLUDES //
//////////////


#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>
#include <math.h>
#include "coroutine.h"
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_sensor.h"
#include "ev3_tacho.h"
#include "communication.h"
#include "motor.h"

// WIN32 /////////////////////////////////////////
#ifdef __WIN32__

#include <windows.h>

// UNIX //////////////////////////////////////////
#else

#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )

//////////////////////////////////////////////////
#endif



//////////////////////
// GLOBAL VARIABLES //
//////////////////////


#define M_PI 3.14159265358979323846

#define M_MOTOR_PORT      OUTPUT_A
#define M_MOTOR_EXT_PORT  EXT_PORT__NONE_
#define S_MOTOR_PORT      OUTPUT_D
#define S_MOTOR_EXT_PORT  EXT_PORT__NONE_
#define L_MOTOR_PORT      OUTPUT_C
#define L_MOTOR_EXT_PORT  EXT_PORT__NONE_
#define R_MOTOR_PORT      OUTPUT_B
#define R_MOTOR_EXT_PORT  EXT_PORT__NONE_
#define IR_CHANNEL        0

#define SPEED_LINEAR      180
#define SPEED_CIRCULAR    100

const char const *color_list[] = { "?", "BLACK", "BLUE", "GREEN", "YELLOW", "RED", "WHITE", "BROWN" };
#define COLOR_COUNT  (( int )( sizeof( color_list ) / sizeof( color_list[ 0 ])))

#define MOD(a,b) ((((a)%(b))+(b))%(b))


#define RELEASING_TIME 10000
#define DETECTION_DISTANCE 100

uint8_t sonar, color;

//////////////////////
//OBSTACLE FUNCTIONS//
//////////////////////

int there_is_obstacle() {
  float us_value;
  get_sensor_value0(sonar, &us_value);
  if ( us_value <=  DETECTION_DISTANCE) return 1;
  return 0;
}

int what_kind_of_obstacle() {
    int color_value;
    if ( !get_sensor_value( 0, color, &color_value ) || ( color_value < 0 ) || ( color_value >= COLOR_COUNT )) {
        color_value = 0;
    }
    printf( "\r(%s)\n", color_list[ color_value ]);
    if(color_value==5) return 1; /* RED : movable obstacle */
    else return 0; /* Non-movable obstacle */
}

int detection() {
	float distance;
	int obstacle_variable;
	//ultrasonic shows the distance to obstacle (float)
	get_sensor_value0(sonar, &distance);
	if(distance>100) {
		obstacle_variable=0;
	}
	else {
		//wall
		if(what_kind_of_obstacle()==0){
			obstacle_variable=1;
		}
		//movable 
		else {
			obstacle_variable=2;
		}
	}
	return obstacle_variable;
}
