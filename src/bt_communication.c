#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <time.h>
#include <sys/socket.h>
#include <math.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <sys/types.h>
#include "mapping_functions.h"


//#define SERV_ADDR   "30:E3:7A:10:9F:28"
#define TEAM_ID     10

#define MSG_ACK     0
#define MSG_START    1
#define MSG_STOP   2
#define MSG_KICK    3
#define MSG_POSITION 4
#define MSG_MAPDATA     5
#define MSG_MAPDONE 6
#define MSG_OBSTACLE 7
#define Sleep( msec ) usleep(( msec ) * 1000 )

int s;
uint16_t msgId = 0;

void debug (const char *fmt, ...) {
  va_list argp;
  va_start (argp, fmt);
  vprintf (fmt, argp);
  va_end (argp);
}

int str2ba(const char *str,bdaddr_t *ba) {
  uint8_t b[6];
  const char *ptr = str;
  int i;

  for (i = 0; i < 6; i++) {
    b[i] = (uint8_t) strtol(ptr, NULL, 16);
    if (i != 5 && !(ptr = strchr(ptr, ':')))
    ptr = ":00:00:00:00:00";
    ptr++;
  }

  baswap(ba, (bdaddr_t *) b);

  return 0;
}

void baswap(bdaddr_t * dst,const bdaddr_t * src) {
  register unsigned char *d = (unsigned char *) dst;
  register const unsigned char *s = (const unsigned char *) src;
  register int i;

  for (i = 0; i < 6; i++) d[i] = s[5-i];
}

int read_from_server (int sock, char *buffer, size_t maxSize) {
  int bytes_read = read (sock, buffer, maxSize);

  if (bytes_read <= 0) {
    fprintf (stderr, "Server unexpectedly closed connection...\n");
    close (s);
    exit (EXIT_FAILURE);
  }

  printf ("[DEBUG] received %d bytes\n", bytes_read);

  return bytes_read;
}


void send_position (double x, double y) {
  printf("I'm sending my position...\n");
  char string[9];
  int i = floor(x);
  int j = floor(y);
  char ci[2];
  char cj[2];
  *((int16_t *) ci) = (int16_t) i;
  *((int16_t *) cj) = (int16_t) j;
  *((uint16_t *) string) = msgId++;
  string[2] = TEAM_ID;
  string[3] = 0xFF;
  string[4] = MSG_POSITION;
  string[5] = ci[0];
  string[6] = ci[1];
  string[7] = cj[0];           
  string[8] = cj[1];         
  write(s, string, 9);
  printf("Done sending my position...\n");
}

void send_obstacle(double x, double y, int act) {
  if act==0{
  	printf("I've released an osbtacle at %d,%d\n", x,y);
  }
  else {
	printf("I've picked an osbtacle at %d,%d\n", x,y);
  }

  char string[9];
  int i = floor(x);
  int j = floor(y);
  char ci[2];
  char cj[2];
  *((int16_t *) ci) = (int16_t) i;
  *((int16_t *) cj) = (int16_t) j;
  *((uint16_t *) string) = msgId++;
  string[2] = TEAM_ID;
  string[3] = 0xFF;
  string[4] = MSG_OBSTACLE;
  string[5] = act;
  string[6] = ci[0];
  string[7] = ci[1];
  string[8] = cj[0];           
  string[9] = cj[1];         
  write(s, string, 9);
  printf("I've sent the position of the obstacle...\n");
}
  

void send_map_done(void) {
  char string[4];
  *((uint16_t *) string) = msgId++;
  string[2] = TEAM_ID;
  string[3] = 0xFF;
  string[4] = MSG_MAPDONE;
  write(s, string, 4);
  printf("My map is complete, I will send it to the server\n");
}

void send_map(pixel map[100][100]) {
  printf("Sending my map...\n")
  char string[11];
  string[2] = TEAM_ID;
  string[3] = 0xFF;
  string[4] = MSG_MAPDATA;
  int i,j
  for (i=0 ; i<100 ; i++) {
	for (j=0 ; j<100 ; j++) {
		int x = floor(map[i][j]->x);
  		int y = floor(map[i][j]->y);
        	char ci[2];
        	char cj[2];
  		*((int16_t *) ci) = (int16_t) i;
  	 	*((int16_t *) cj) = (int16_t) j;
  	 	*((uint16_t *) string) = msgId++;
		string[5] = ci[0];
  		string[6] = ci[1];
  	 	string[7] = cj[0];           
  	 	string[8] = cj[1];  		
		string[9] = map[i][j]->R;
		string[10] = map[i][j]->G;
		string[11] = map[i][j]->B;
		write(s,string,11);
	}
  }
  printf("Done sending my map !\n");
}

