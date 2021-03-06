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
#include "move_functions.h"


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

#define SPEED_LINEAR      240
#define SPEED_CIRCULAR    100
#define SCAN_ANGLE	  40
int dist_to_obstacle = 100000;
int final_angle;

enum { L, R };

uint8_t ir;

uint8_t motorLR[ 2 ] = { DESC_LIMIT, DESC_LIMIT};  /* Sequence numbers of motors */


extern pthread_mutex_t mutexCompass;
extern int angle_compass;
extern int direction_us;




//////////////////////
// MOTORS FUNCTIONS //
//////////////////////


void _run_forever( int l_speed, int r_speed )
{
	set_tacho_speed_sp( motorLR[ L ], l_speed );
	set_tacho_speed_sp( motorLR[ R ], r_speed );
	multi_set_tacho_command_inx( motorLR, TACHO_RUN_FOREVER );
}

void _run_to_rel_pos( int l_speed, int l_pos, int r_speed, int r_pos )
{
	set_tacho_speed_sp( motorLR[ L ], l_speed );
	set_tacho_speed_sp( motorLR[ R ], r_speed );
	set_tacho_position_sp( motorLR[ L ], l_pos );
	set_tacho_position_sp( motorLR[ R ], r_pos );
	multi_set_tacho_command_inx( motorLR, TACHO_RUN_TO_REL_POS );
}

void _run_timed( int l_speed, int r_speed, int ms )
{
	set_tacho_speed_sp( motorLR[ L ], l_speed );
	set_tacho_speed_sp( motorLR[ R ], r_speed );
	multi_set_tacho_time_sp( motorLR, ms );
	multi_set_tacho_command_inx( motorLR, TACHO_RUN_TIMED );
}

int _is_running( void )
{
	FLAGS_T state = TACHO_STATE__NONE_;

	get_tacho_state_flags( motorLR[ L ], &state );
	if ( state != TACHO_STATE__NONE_ ) return ( 1 );
	get_tacho_state_flags( motorLR[ R ], &state );
	if ( state != TACHO_STATE__NONE_ ) return ( 1 );

	return ( 0 );
}

void _stop( void )
{
	multi_set_tacho_command_inx( motorLR, TACHO_STOP );

}

void turn_to_certain_direction (int direc) {
	_stop();
	direc = direc % 360;
	multi_set_tacho_stop_action_inx( motorLR, TACHO_BRAKE );

	pthread_mutex_lock (& mutexCompass );
	int original = angle_compass;
	pthread_mutex_unlock (& mutexCompass );

	int diff1 = 0;
	int first = 0;
	diff1 = direc - original;

	if (diff1 > 180) {
		diff1 = - (360 - diff1 );
	}

	if(diff1 < -180) {
		diff1 = 360 + diff1;
	}

	if (diff1 > 30 ) {
		first = (diff1 -10)*2;
	}

	if (diff1 < -30) {
		first = (diff1 +10)*2;
	}

	if (first > 0) {
		_run_to_rel_pos(120,first,-120,(0-first));
		while(_is_running());
		_stop();
	}

	if (first < 0) {
		_run_to_rel_pos(-120,first,120,(0-first));
		while(_is_running());
		_stop();
	}
	_stop();

	int is_pass_360 = 0;
	if (( (diff1 + original) < 0 ) || ( (original + diff1) > 360 )) {
		is_pass_360 = 1;
	}


	if(diff1 < 0 ) {

		_run_forever(-10,10);
		Sleep(1);

		if ( is_pass_360 ) {
			while( angle_compass <= 90  );
		}
		if(direc > 358) { direc = 358;}
		else if (direc == 0) {
			direc = 2;
		}
		while( angle_compass > direc  );
		_stop();
	}

	if(diff1 > 0) {
		_run_forever(10,-10);
		Sleep(1);
		if( is_pass_360 ) {
			while( angle_compass >= 270 );
		}
		if(direc>358) {
			direc = 358;
		}
		else if (direc == 0) {
			direc = 2;
		}
		while( angle_compass < direc  );
		_stop();
	}
	_stop();
}

void turn_right_certain_degree(int degree) {
	_stop();
	pthread_mutex_lock (& mutexCompass );
	int original = angle_compass;
	pthread_mutex_unlock (& mutexCompass );

	degree = degree % 360;
	multi_set_tacho_stop_action_inx( motorLR, TACHO_BRAKE );

	int direct = (degree + original) % 360;

	turn_to_certain_direction(direct);
	
	_stop();
}

void turn_left_certain_degree(int degree) {
	_stop();
	pthread_mutex_lock (& mutexCompass );
	int original = angle_compass;
	pthread_mutex_unlock (& mutexCompass );

	degree = degree % 360;
	multi_set_tacho_stop_action_inx( motorLR, TACHO_BRAKE );

	int direct = (original - degree);

	if (direct < 0) {
		direct = direct +360;
	}

	turn_to_certain_direction(direct);
	
	_stop();
}

void turn_left() {
	multi_set_tacho_stop_action_inx( motorLR, TACHO_BRAKE );
	//printf("turning left 90 start\n");

	pthread_mutex_lock (&mutexCompass);
	int original = angle_compass;
	pthread_mutex_unlock (&mutexCompass);

	_run_to_rel_pos(-150,-160,150,160);
	Sleep(1200);
	_stop();
	//printf("11111111turning left 90 , previous= %d,   now=% d\n", original,angle_compass);
	_run_forever(-10,10);
	Sleep(10);

	if (original<90) {
		while(angle_compass < 90);
		while(angle_compass - original >= 271);
		_stop();
	}
	else {
		while(abs(original - angle_compass) <= 89);
		_stop();
	}

	_stop();
	//printf("turning left 90 , previous= %d,   now=% d\n", original,angle_compass);
}

void turn_right() {
	multi_set_tacho_stop_action_inx( motorLR, TACHO_BRAKE );

	//printf("turning right 90 start\n");
	
	pthread_mutex_lock (& mutexCompass );
	int original = angle_compass;
	pthread_mutex_unlock (& mutexCompass );

	_run_to_rel_pos(150,160,-150,-160);
	Sleep(1200);
	_stop();
	//printf("11111111turning right 90 , previous= %d,   now=% d\n", original,angle_compass);	
	_run_forever(10,-10);
	Sleep(10);

	if (original>270) {
		//while( angle_compass >270);
		while(original - angle_compass >= 271);
		_stop();
	}
	else {
		while(angle_compass - original <= 89);
		_stop();
	}

	_stop();
	//printf("turning right 90 , previous= %d,   now=% d\n", original,angle_compass);
}

Pair searching_obstacle_right(){
	int angle;
	Pair mypair;
	mypair.detected_angle = 400;
	mypair.detected_distance = 1000;
	
	int original;
	pthread_mutex_lock(&mutexCompass);
        original=angle_compass;
        pthread_mutex_unlock(&mutexCompass);

	for (angle=0;angle<SCAN_ANGLE;angle=angle+5){
		turn_right_certain_degree(5);
		if (distance_to_obstacle()<mypair.detected_distance){
			mypair.detected_distance=distance_to_obstacle();
			pthread_mutex_lock(&mutexCompass);
			mypair.detected_angle=angle_compass;
			pthread_mutex_unlock(&mutexCompass);
		}
	}
	Sleep(1000);


	turn_to_certain_direction(original);
	return mypair;
}


int searching_obstacle_right_array(Pair *obstacles, int index) {
        int angle;
	int distance;
	Pair mypair;
	int i = index;

	int original;
        pthread_mutex_lock(&mutexCompass);
        original=angle_compass;
        pthread_mutex_unlock(&mutexCompass);

        for (angle=0;angle<SCAN_ANGLE;angle=angle+5){
                turn_right_certain_degree(5);
		distance = distance_to_obstacle();
                if (distance<550){
			mypair.detected_angle = angle;
			mypair.detected_distance = distance;
                        obstacles[i] = mypair;
			i+=1;
                }
        }
        Sleep(1000);
	
	turn_to_certain_direction(original);
        return i;
}

Pair searching_obstacle_left(){
	int angle;
        Pair mypair;
        mypair.detected_angle = 400;
        mypair.detected_distance = 1000;

	int original;
        pthread_mutex_lock(&mutexCompass);
        original=angle_compass;
        pthread_mutex_unlock(&mutexCompass);

	for (angle=0;angle<SCAN_ANGLE;angle=angle+5){
		turn_left_certain_degree(5);
		if (distance_to_obstacle()<mypair.detected_distance){
                        mypair.detected_distance=distance_to_obstacle();
                        pthread_mutex_lock(&mutexCompass);
                        mypair.detected_angle=angle_compass;
                        pthread_mutex_unlock(&mutexCompass);
                }
	}
	Sleep(1000);

	turn_to_certain_direction(original);
	return mypair;
}

int searching_obstacle_left_array(Pair *obstacles, int index){
        int angle;
        int distance;
        Pair mypair;
        int i = index;

        for (angle=0;angle<SCAN_ANGLE;angle=angle+5){
                turn_left_certain_degree(5);
        	distance = distance_to_obstacle();
                if (distance<550){
                        mypair.detected_angle = angle;
                        mypair.detected_distance = distance;
                        obstacles[i] = mypair;
                        i+=1;
                }
	}
        Sleep(1000);

        turn_right_certain_degree(SCAN_ANGLE);
        return i;
}
