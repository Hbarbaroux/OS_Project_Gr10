#include <stdio.h>
#include "coroutine.h"
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_sensor.h"
#include "ev3_tacho.h"
#include <unistd.h>

if(ev3_search_sensor( LEGO_EV3_US, &sonar, 0 )) printf("%s\n", "US found");

uint8_t sonar, color, compas;  /* Sequence  of sensors */


int there_is_obstacle()
{
  float us_value;
  get_sensor_value0(sonar, &us_value);
  if ( us_value <=  10) 
    return 1;
  return 0;
}

int what_kind_of_obstacle() 
{       
    int color_value;
    if ( !get_sensor_value( 0, color, &color_value ) || ( color_value < 0 ) || ( color_value >= COLOR_COUNT )) {
        color_value = 0;
    }
    printf( "\r(%s)\n", color_list[ color_value ]);
    if(color_value==5) return 1;
    else return 0;
}

int detection(){
	float distance;
	int obstacle_variable;
	//ultrasonic shows the distance to obstacle (float)
	get_sensor_value0(sonar, &distance);
	if(distance>10){
		obstacle_variable=0;
		}
		else{
			//wall
			if(what_kind_of_obstacle()==0){
			obstacle_variable=1;
			}
			//movable 
			else{result=2;}
	}
}