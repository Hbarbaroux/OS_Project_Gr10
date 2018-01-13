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

#define EMPTY 0
#define OBSTACLE 1
#define WALL 2
#define UNDEFINED 3
#define SMALL 0
#define BIG 1

double x,y;

struct pixel {
	int x;
	int y;
	int R;
	int G;
	int B;
	int type;
};

pixel map_small [40][24];
pixel map_big [100][100];

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
	// i is y and j is x
	switch (type) {
	case EMPTY: 
		map[i][j].R=0;
		map[i][j].G=0;
		map[i][j].B=254;
		map[i][j].type=type;

	case OBSTACLE_WALL:
		map[i][j].R=0;
		map[i][j].G=254;
		map[i][j].B=0;
		map[i][j].type=type;


	case UNDEFINED:
		map[i][j].R=0;
		map[i][j].G=0;
		map[i][j].B=0;
		map[i][j].type=type;

}


void initialize_small_map(void) {
	int i,j;
	for (i=0;i<40;i++) {
		draw_map(i,0,OBSTACLE_WALL);
		draw_map(i,23,OBSTACLE_WALL);
	}
	
	for (j=0;j<24;j++) {
		draw_map(0,j,OBSTACLE_WALL);
		draw_map(39,j,OSBTACLE_WALL);
	}
}

void print_map (pixel map, int size) {
	int i,j;
	int row, column;
	if (size==SMALL) {
		row = 24;
		column = 40;
	}
	else {
		row = 100;
		column = 100;
	}

	for (i=0;i<column;i++) {

		for (j=0;j<row;j++) {
		
			if (j==0) { printf("\n"); }

			switch (map[i][j].type) {
				case EMPTY : 
					printf(" ");
				case OSBTACLE_WALL : 
					printf("x");
				case UNDEFINED : 
					printf("u");
		}
	}
		
}	









