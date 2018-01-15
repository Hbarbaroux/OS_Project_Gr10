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

void send_position(double x, double y);

void send_obstacle(double x, double y, int act);

void send_map_done(void);

void send_map(int map_type);

int connect_server(void);

int str2ba(const char *str,bdaddr_t *ba);

void baswap(bdaddr_t * dst,const bdaddr_t * src); 	 	
