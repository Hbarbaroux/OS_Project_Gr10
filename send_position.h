#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <time.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#define MSG_START 1
#define MSG_STOP 2

void read_from_server(int sock, char *buffer, size_t maxSize);

void send_position(double x, double y);

int connect_server(void);

int str2ba(const char *str,bdaddr_t *ba);

void baswap(bdaddr_t * dst,const bdaddr_t * src); 	 	
