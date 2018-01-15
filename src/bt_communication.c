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

#ifndef MAPPING
#define MAPPING

#include "mapping_functions.h"

#endif


//#define SERV_ADDR "00:19:0E:10:72:CB"
#define SERV_ADDR "00:1A:7D:DA:71:06"
//#define SERV_ADDR "DC:53:60:AD:61:90"
#define TEAM_ID     10

#define SMALL 0
#define BIG 1



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

extern Pixel big_map[78][100];
extern Pixel small_map[40][24];

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
  //printf("I'm sending my position...\n");
  char string[9];
  int i = floor(x/5.0);
  int j = floor(y/5.0);
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
  //printf("Done sending my position...\n");
}

void send_obstacle(double x, double y, int act) {
  if (act==0){
  	printf("I've released an osbtacle at %d,%d\n", x,y);
  }
  else {
	printf("I've picked an osbtacle at %d,%d\n", x,y);
  }

  char string[10];
  int i = floor(x/5.0);
  int j = floor(y/5.0);
  char ci[2];
  char cj[2];
  *((int16_t *) ci) = (int16_t) i;
  *((int16_t *) cj) = (int16_t) j;
  *((uint16_t *) string) = msgId++;
  string[2] = TEAM_ID;
  string[3] = 0xFF;
  string[4] = MSG_OBSTACLE;
  string[5] = (char)act;
  string[6] = ci[0];
  string[7] = ci[1];
  string[8] = cj[0];           
  string[9] = cj[1];         
  write(s, string, 10);
  printf("I've sent the position of the obstacle...\n");
}
  

void send_map_done(void) {
  char string[5];
  *((uint16_t *) string) = msgId++;
  string[2] = TEAM_ID;
  string[3] = 0xFF;
  string[4] = MSG_MAPDONE;
  write(s, string, 5);
  printf("My map is complete, I sent it to the server\n");
}

void send_map(int map_type) {
  printf("Sending my map...\n");
  char string[12];
*((uint16_t *) string) = msgId++;
  string[2] = TEAM_ID;
  string[3] = 0xFF;
  string[4] = MSG_MAPDATA;
  int i,j;
  int lim_i;
  int lim_j;
  if (map_type == SMALL) {
	lim_i = 40;
	lim_j = 24;
  }
  else {
	lim_i = 78;
	lim_j = 78;	
  }
  for (i=0 ; i<lim_i ; i++) {
	for (j=0 ; j<lim_j ; j++) {
		if (((map_type == SMALL) && (small_map[i][j].G == 100)) || ((map_type == BIG) && (big_map[i][j].G == 100))) {
		int x = j;
  		int y = i;
		char ci[2];
        	char cj[2];
  		*((int16_t *) ci) = (int16_t) i;
  	 	*((int16_t *) cj) = (int16_t) j;
		string[5] = cj[0];
  		string[6] = cj[1];
  	 	string[7] = ci[0];           
  	 	string[8] = ci[1];
		if (map_type == SMALL) {  		
			string[9] = small_map[i][j].R;
			string[10] = small_map[i][j].G;
			string[11] = small_map[i][j].B;
		}
		else {
			string[9] = big_map[i][j].R;
                        string[10] = big_map[i][j].G;
                        string[11] = big_map[i][j].B;
		}
		write(s,string,12);
		Sleep(1);
		}
	}
  }
  printf("Done sending my map !\n");
}

