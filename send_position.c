#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <time.h>
#include <sys/socket.h>
#include <math.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>

#define SERV_ADDR   "30:E3:7A:10:9F:28"     /* Whatever the address of the server is */
#define TEAM_ID     10                       /* Your team ID */

#define MSG_ACK     0
#define MSG_START    1
#define MSG_STOP   2
#define MSG_KICK    3
#define MSG_POSITION 4
#define MSG_MAPDATA     5
#define MSG_MAPDONE 6
#define Sleep( msec ) usleep(( msec ) * 1000 )

void debug (const char *fmt, ...) {
  va_list argp;

  va_start (argp, fmt);

  vprintf (fmt, argp);

  va_end (argp);
}

int str2ba(const char *str,bdaddr_t *ba)


{
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

void baswap(bdaddr_t * dst,const bdaddr_t * src)
{
       register unsigned char *d = (unsigned char *) dst;
       register const unsigned char *s = (const unsigned char *) src;
       register int i;

       for (i = 0; i < 6; i++)
              d[i] = s[5-i];
}

int s;

uint16_t msgId = 0;

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


void send_position (double x, double y){

  printf("I'm sending my position...\n");

  char string[9];
  char type;

  *((uint16_t *) string) = msgId++;
  string[2] = TEAM_ID;
  string[3] = 0xFF;
  string[4] = MSG_POSITION;
  string[5] = x;          /* x */
  string[6] = 0x00;
  string[7] = y;              /* y */
  string[8]= 0x00;
  write(s, string, 9);

  printf("Done sending my position...\n");

}

int connect_server(void){

  struct sockaddr_rc addr = { 0 };
  int status;
  char string[9];

  /* allocate a socket */
  s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

  /* set the connection parameters (who to connect to) */
  addr.rc_family = AF_BLUETOOTH;
  addr.rc_channel = (uint8_t) 1;
  str2ba (SERV_ADDR, &addr.rc_bdaddr);

  /* connect to server */
  status = connect(s, (struct sockaddr *)&addr, sizeof(addr));

  /* if connected */
  if( status == 0 ) {

   /* Wait for START message */
    read_from_server (s, string, 9);
    if (string[4] == MSG_START) {
      printf ("Received start message!\n");
    }

    while(1){
    //Wait for stop message
    read_from_server (s, string, 58);
    if (string[4]==MSG_STOP){
      close(s);
      return;
    }

    close (s);

    sleep (5);

  }
   }
    else {
    fprintf (stderr, "Failed to connect to server...\n");
    sleep (2);
    exit (EXIT_FAILURE);
  }

  close(s);
  return 0;
}

