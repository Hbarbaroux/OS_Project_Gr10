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

#define EMPTY 0
#define OBSTACLE 1
#define WALL 2
#define UNDEFINED 3

double x,y;

struct pixel {
	int x;
	int y;
	int R;
	int G;
	int B;
};

pixel map[100][100];


//////////////////////
// MAPPING FUNCTIONS//
//////////////////////


void reset_coord() {
	x = 60;
	y = 15;
}


void update_coord(double diff, int angle) {
	switch (angle) {
	case 0:
		x = x+diff;
		break;
	case 1:
		y = y+diff;
		break;
	case 2:
		x = x-diff;
		break;
	case 3:
		y = y-diff;
		break;
	}
}


void update_coord_compass(double diff, double angle) {
    double diffx;
    double diffy;
    int diff_angle = (direction[0] - angle)%360;
    double value = (double) diff_angle * M_PI / 180.0;
    diffx = diff * sin(value);
    diffy = diff * cos(value);
    x = x + diffx;
    y = y + diffy;
}


void draw_map(int i, int j, int type) {
	
	switch (type) {
	case EMPTY: 
		map[i][j]->R=0;
		map[i][j]->G=0;
		map[i][j]->B=0;

	case OBSTACLE:
		map[i][j]->R=0;
		map[i][j]->G=100;
		map[i][j]->B=0;

	case WALL:
		map[i][j]->R=0;
		map[i][j]->G=0;
		map[i][j]->B=100;

	case UNDEFINED:
		map[i][j]->R=100;
		map[i][j]->G=0;
		map[i][j]->B=0;

}





