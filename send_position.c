#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <time.h>
#include <sys/socket.h>
#include <math.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <compass_sensor.h>

#define SERV_ADDR   "dc:53:60:ad:61:90"     /* Whatever the address of the server is */
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

void send_position (int x, int y){

  printf("I'm sending my position...\n", );

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
  Sleep(20);

  printf("Done sending my position...\n", );

}

int connect_and_send(){
/*int main(int argc, char **argv)*/
  struct sockaddr_rc addr = { 0 };
  int status;

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
    int x = get_x_coord();
    int y = get_y_coord();

    send_begin_position(x,y);

    sleep(2);

    //Wait for stop message
    read_from_server (s, string, 58);
    type = string[4];
    if (type ==MSG_STOP){
      break;
    }
    close (s);

    sleep (5);

  } else {
    fprintf (stderr, "Failed to connect to server...\n");
    sleep (2);
    exit (EXIT_FAILURE);
  }

  close(s);
  return 0;
}
