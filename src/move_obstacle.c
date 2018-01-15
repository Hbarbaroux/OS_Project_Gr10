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
enum { M, S };

uint8_t motorMS[2]={ DESC_LIMIT, DESC_LIMIT };

void _take_front_obstacle() {
	//printf("Grabbing\n");
	set_tacho_speed_sp( motorMS[S], -100 );
	//set_tacho_speed_sp( motorMS[ R ], r_speed );
	set_tacho_position_sp( motorMS[S], -160 );
	//set_tacho_position_sp( motorMS[ R ], r_pos );
	set_tacho_command_inx( motorMS[S], TACHO_RUN_TO_REL_POS );
	Sleep(1500);
	//printf("Stop grabbing\n");

}

void _release_front_obstacle() {
	//printf("Releasing\n");
	set_tacho_speed_sp( motorMS[S], 100 );
	//set_tacho_speed_sp( motorMS[ R ], r_speed );
	set_tacho_position_sp( motorMS[S], 170 );
	//set_tacho_position_sp( motorMS[ R ], r_pos );
	set_tacho_command_inx( motorMS[S], TACHO_RUN_TO_REL_POS );
	//printf("release inside\n");
	Sleep(1500);
	//printf("Stop releasing\n");
}

void _release_obstacle() {
	set_tacho_speed_sp( motorMS[M], -180 );
	//set_tacho_speed_sp( motorMS[ R ], r_speed );
	set_tacho_position_sp( motorMS[M], 60 );
	//set_tacho_position_sp( motorMS[ R ], r_pos );
	set_tacho_command_inx( motorMS[M], TACHO_RUN_TO_REL_POS );
	Sleep(1200);

	set_tacho_speed_sp( motorMS[M], 180 );
	//set_tacho_speed_sp( motorMS[ R ], r_speed );
	set_tacho_position_sp( motorMS[M], -60 );
	//set_tacho_position_sp( motorMS[ R ], r_pos );
	set_tacho_command_inx( motorMS[M], TACHO_RUN_TO_REL_POS );
	//printf("release inside\n");
	Sleep(1200);
}



