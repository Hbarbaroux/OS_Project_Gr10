//Functions written by Virgile Uytterhaegen

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <time.h>
#include <math.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>


#ifndef MAPPING
#define MAPPING

#include "mapping_functions.h"

#endif


void read_from_server(int sock, char *buffer, size_t maxSize);

//Send the position of the robot to the server
void send_position(double x, double y);

//Send a MSG_OBSTACLE to the server with the position of the obstacle and if it has been released (act = 0) or picked (act = 1)
void send_obstacle(double x, double y, int act);

//Send a MSG_MAPDONE to the server
void send_map_done(void);

//Send the map to the server
void send_map(int map_type);

int connect_server(void);

int str2ba(const char *str,bdaddr_t *ba);

void baswap(bdaddr_t * dst,const bdaddr_t * src); 	 	
